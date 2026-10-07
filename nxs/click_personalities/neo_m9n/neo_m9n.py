"""
NEO-M9N personality for NXS VM.

u-blox NEO-M9N GNSS receiver (M9 platform, UBX-M9140); datasheet UBX-19014285,
    M9 SPG 4.04 interface description UBX-21022436 (protocol v32).
Bus: UART1, 38400 baud (fixed); UBX binary by default, NMEA variant; CFG-VALSET to the RAM layer.
Config: protocol binary|nmea (binary), compile-time. Params: rate 1|5|10|25 Hz (1), CFG-RATE-MEAS.
Outputs: UBX-NAV-PVT fields (see UbxNavPvtDriver), or nmea (string, 82 bytes).
Pins: INT -> TIMEPULSE (1PPS); AN -> D_SEL (UART select). RST -> RESET_N, PWM -> EXTINT: unused.
mikroE Click: https://www.mikroe.com/gnss-7-click
"""

from nxs import StreamClickPersonality
from nxs.compiler import ChecksumFletcher
from nxs.click_personalities._ubx_nav_pvt import (UbxNavPvtDriver, K_MSGOUT_PVT_UART1,
                                      K_RATE_MEAS, valset_frame)


class NeoM9n(UbxNavPvtDriver):
    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'pps': 'mkbus_int',        # 1PPS time pulse (input)
    }

    # Production default (the pre-production R01 datasheet lists 9600); fixed,
    # since a runtime change would desync the fixed-baud probe handshake.
    DEFAULT_BAUD = 38400

    # CFG-RATE-MEAS period = 1000 // rate_hz ms; capped at 25 Hz for
    # multi-constellation use (30 Hz single-GNSS maximum).
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

        self.declare_params_from_descriptor()
        rate_hz = config.get('rate', 1)
        meas_rate_ms = 1000 // rate_hz

        # UART sensors poll; the personality pins the VM's outer cadence.
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
    @StreamClickPersonality.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "nmea"))
    def measure_nmea(self):
        self.read_until(b'\n', max=82)           # one sentence
        self.store_sample_n()
