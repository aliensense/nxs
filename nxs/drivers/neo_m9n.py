"""
NEO-M9N driver for NXS VM.

u-blox NEO-M9N GNSS receiver module (u-blox M9 platform, UBX-M9140),
streaming PVT position/velocity/time epochs over UART. The binary UBX
protocol is the default and publishes typed geodetic fields; NMEA is a
secondary `protocol` variant that publishes raw sentence strings.

Configuration uses the u-blox M9 configuration database (UBX-CFG-VALSET,
class 0x06 / ID 0x8A) targeting the volatile RAM layer, so the driver
reconfigures the receiver on every load and never wears the battery-backed
or flash layers. The epoch rate is the `rate` param, patched into the
CFG-RATE-MEAS frame with the 8-bit Fletcher checksum recomputed at runtime.

Probe sends a fixed CFG-VALSET and verifies the UBX-ACK-ACK response —
receivers have no WHO_AM_I register.

mikroBUS pin mapping (from mikroE GNSS 7 Click C driver):
    TX  (PB6)  -> RXD1:      UART host -> module (data in)
    RX  (PB7)  -> TXD1:      UART module -> host (data out)
    INT (PA9)  -> TIMEPULSE: 1PPS time pulse (input)
    RST (PB2)  -> RESET_N:   Module reset (output, active low)
    PWM (PA10) -> EXTINT:    External interrupt / wake (input)
    AN  (PA0)  -> D_SEL:     Interface select; high/open selects UART+I2C,
                             the mode this driver uses (a static board fact)

Interface: UART only. Default baud 38400 (GNSS 7 Click default and NEO-M9N
production default; the pre-production R01 datasheet lists 9600). Baud is
fixed here — a runtime change would desync the host link, which the
fixed-baud probe handshake cannot recover. 3.3V rail (module spec 2.7-3.6V).

Config keys:
    protocol : binary | nmea   (default binary)   -- compile-time; UBX binary
               (typed PVT fields) vs NMEA sentence strings.

Params:
    rate : {1, 5, 10, 25} Hz   (default 1)  -- GNSS measurement/nav rate
               (CFG-RATE-MEAS). NEO-M9N supports up to 30 Hz single-GNSS;
               the set is capped conservatively for multi-constellation.

Datasheet: u-blox NEO-M9N Data sheet (UBX-19014285)
Interface: u-blox M9 SPG 4.04 Interface description (UBX-21022436, protocol v32)
mikroE Click: https://www.mikroe.com/gnss-7-click
"""

from nxs import StreamDriver
from nxs.compiler import ChecksumFletcher
from nxs.drivers._ubx_nav_pvt import (UbxNavPvtDriver, K_MSGOUT_PVT_UART1,
                                      K_RATE_MEAS, valset_frame)


class NeoM9n(UbxNavPvtDriver):
    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'pps': 'mkbus_int',        # 1PPS time pulse (input)
    }

    DEFAULT_BAUD = 38400

    # CFG-RATE-MEAS is the measurement period in ms: rate_hz -> 1000 // rate_hz.
    RATES = [1, 5, 10, 25]

    # ── M9-specific config-database key IDs (L-typed protocol enables;
    #    the shared rate/MSGOUT keys live in _ubx_nav_pvt).
    _K_UART1OUT_UBX  = 0x10740001   # L  (enable UBX output on UART1)
    _K_UART1OUT_NMEA = 0x10740002   # L  (enable NMEA output on UART1)

    # ── Probe: fixed CFG-VALSET (RATE-MEAS = 1000 ms), verify UBX-ACK-ACK ──
    def probe(self):
        self.set_baud(self.DEFAULT_BAUD)
        self.sleep_ms(100)
        # Fixed CFG-VALSET: CFG-RATE-MEAS = 1000 ms (1 Hz), RAM layer.
        self.write(valset_frame([(K_RATE_MEAS, 1000, 2)]))
        # A factory-default receiver streams NMEA at power-on, so sync to the
        # UBX header (0xB5 never occurs in NMEA text) before matching the ACK.
        self.read_until(b'\xb5\x62', timeout_ms=2000)
        ack = self.read_n(8, timeout_ms=2000)         # 05 01 02 00 06 8A CK_A CK_B
        ack.expect(b'\x05\x01\x02\x00\x06\x8a')        # UBX-ACK-ACK of CFG-VALSET

    def configure(self, config):
        protocol = config.get('protocol', 'binary')

        self.declare_param("rate", values=self.RATES, default=1, unit="Hz")
        rate_hz = config.get('rate', 1)
        meas_rate_ms = 1000 // rate_hz

        # UART sensors poll; the driver pins the VM's outer cadence.
        config['sample_rate'] = 100
        config['trigger'] = 'poll'

        # ── Message routing per protocol (fixed writes, no runtime param) ──
        if protocol == 'binary':
            # Enable UBX + NAV-PVT on UART1, silence all NMEA so the sync
            # search never wades through foreign sentences.
            self.write(valset_frame([
                (self._K_UART1OUT_UBX, 1, 1),
                (self._K_UART1OUT_NMEA, 0, 1),
                (K_MSGOUT_PVT_UART1, 1, 1),
            ]))
        else:
            # Enable NMEA on UART1 and silence the binary PVT so the line is
            # pure NMEA sentences.
            self.write(valset_frame([
                (self._K_UART1OUT_NMEA, 1, 1),
                (K_MSGOUT_PVT_UART1, 0, 1),
            ]))

        # ── Rate command: patchable value + runtime-recomputed checksum ──
        # Frame: B5 62 06 8A 0A 00 | ver layers res res | key(4) | rate(2) | CK CK
        # rate value at frame offset 14 (size 2); Fletcher over [2,16) -> [16,18).
        frame = valset_frame([(K_RATE_MEAS, meas_rate_ms, 2)])
        self.stage(frame, patch=("rate", rate_hz, meas_rate_ms, 14, 2))
        self.compute_checksum(ChecksumFletcher(), start_off=2, length=14, dst_off=16)
        self.send_staged(len(frame))
        self.sleep_ms(100)

        # ── Output vector ──
        if protocol == 'binary':
            self.apply_nav_pvt_outputs()
        else:
            self.set_output([
                {'name': 'nmea', 'type': 'string', 'count': 82,
                 'scale': 1.0, 'unit': ''},
            ])
            self.set_sample_size(82)     # NMEA max sentence length

    # ── NMEA sentences: secondary protocol variant. ──
    @StreamDriver.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "nmea"))
    def measure_nmea(self):
        self.read_until(b'\n', max=82)           # one sentence
        self.store_sample_n()
