"""
ZED-F9P driver for NXS VM.

u-blox ZED-F9P high-precision multi-band RTK GNSS receiver module.
UART (mikroBUS TX/RX) speaking the u-blox UBX binary protocol, with
NMEA 0183 as a secondary text protocol. Streams the full PVT epoch
(position / velocity / time) as typed geodetic fields decoded from
the UBX-NAV-PVT record (class 0x01, id 0x07, 92-byte payload).

Protocol (compile-time config key `protocol`):
    binary (default) - UBX-NAV-PVT, typed geodetic + quality fields
    nmea             - one NMEA sentence per sample as a string blob

The receiver is configured through the gen-9 configuration database
(UBX-CFG-VALSET, class 0x06 id 0x8A) targeting the volatile RAM layer
only - the driver reconfigures on every load and never burns the
battery-backed / flash layers (avoids part wear and cross-session
state leak). The epoch rate is set through CFG-RATE-MEAS with the
frame checksum recomputed at runtime so `set rate N` stays valid.

UART1 baud is fixed at the module power-on default 38400; retuning it
needs a coordinated chip + host baud change out of scope for the poll
loop, so it is not exposed as a knob.

Carrier: MikroE GNSS RTK Click (MIKROE-4456), module ZED-F9P-05B.
mikroBUS pin mapping (from mikroE GNSS RTK `gnssrtk` C driver):
    TX  (PB6)  -> RXD:     receiver UART1 input  (host TX)
    RX  (PB7)  -> TXD:     receiver UART1 output (host RX)
    INT (PA9)  -> TIMEPULSE (PPS): pulse-per-second output (input)
    AN  (PA0)  -> TXD_READY: data-ready / TX-ready (input)
    RST (PB2)  -> RTK status output -> RTK LED (input; NOT module reset)
    PWM (PA10) -> RESET_N: module reset (output, active low; unused —
                  the driver reconfigures the RAM layer on every load)

Only PPS is declared in PINS; UART is the data path (poll-paced).

Config keys:
    protocol : binary | nmea            (default binary)

Params:
    rate     : 1 | 2 | 5 | 10 Hz        (default 1)

Datasheet: u-blox ZED-F9P Integration Manual + Interface Description
    (UBX-NAV-PVT layout, UBX-CFG-VALSET config-DB keys)
mikroE Click: https://www.mikroe.com/gnss-rtk-click
    github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/gnssrtk
"""

from nxs import StreamDriver
from nxs.compiler import ChecksumFletcher

from nxs.drivers._ubx_nav_pvt import (UbxNavPvtDriver, UBX_CLS_CFG,
                                      UBX_ID_VALSET, K_MSGOUT_PVT_UART1,
                                      K_RATE_MEAS, ubx_frame, valset_body,
                                      valset_frame)

# F9-specific config-database key IDs: the NMEA sentence routing this
# driver silences (group 0x2091 = CFG-MSGOUT, per-port output rate, U1).
# The shared rate/PVT keys live in _ubx_nav_pvt.
_KEY_MSGOUT_GGA_UART1 = 0x209100BB  # U1, NMEA-GGA out on UART1
_KEY_MSGOUT_GLL_UART1 = 0x209100CA  # U1, NMEA-GLL
_KEY_MSGOUT_GSA_UART1 = 0x209100C0  # U1, NMEA-GSA
_KEY_MSGOUT_GSV_UART1 = 0x209100C5  # U1, NMEA-GSV
_KEY_MSGOUT_RMC_UART1 = 0x209100AC  # U1, NMEA-RMC  (…AD is UART2)
_KEY_MSGOUT_VTG_UART1 = 0x209100B1  # U1, NMEA-VTG  (…B2 is UART2)

_NMEA_KEYS = (_KEY_MSGOUT_GGA_UART1, _KEY_MSGOUT_GLL_UART1,
              _KEY_MSGOUT_GSA_UART1, _KEY_MSGOUT_GSV_UART1,
              _KEY_MSGOUT_RMC_UART1, _KEY_MSGOUT_VTG_UART1)


class ZedF9p(UbxNavPvtDriver):
    """u-blox ZED-F9P RTK GNSS receiver - UBX binary (default) + NMEA."""

    DEFAULT_BAUD = 38400
    PINS = {'pps': 'mkbus_int'}

    RATES = {1: 1000, 2: 500, 5: 200, 10: 100}   # Hz -> meas period [ms]

    # Fixed probe: set CFG-RATE-MEAS=1000 ms on the RAM layer, expect ACK.
    PROBE_FRAME = valset_frame([(K_RATE_MEAS, 1000, 2)])
    # UBX-ACK-ACK body (after the b5 62 sync): class 05, id 01, len 0002,
    # acked class 06, acked id 8A. CK_A/CK_B are chip-computed, unchecked.
    ACK_PREFIX = bytes([0x05, 0x01, 0x02, 0x00, UBX_CLS_CFG, UBX_ID_VALSET])

    def probe(self):
        self.set_baud(self.DEFAULT_BAUD)
        self.sleep_ms(100)
        self.write(self.PROBE_FRAME)
        # Power-on default is NMEA broadcast, so sync to the UBX header
        # first (0xB5 never occurs in NMEA text) before matching the ACK.
        self.read_until(b'\xb5\x62', timeout_ms=2000)
        ack = self.read_n(len(self.ACK_PREFIX) + 2, timeout_ms=2000)
        ack.expect(self.ACK_PREFIX)

    def configure(self, config):
        protocol = config.get('protocol', 'binary')
        self.declare_param("rate", values=[1, 2, 5, 10],
                           default=1, unit="Hz")

        # UART part: the driver pins the VM poll cadence; the receiver's
        # own epoch rate keeps the `rate` name.
        config['sample_rate'] = 100
        config['trigger'] = 'poll'

        rate_hz = config.get('rate', 1)
        meas_rate_ms = self.RATES[rate_hz]

        # One VALSET: rate + enable exactly the framed record and disable
        # the other protocol's chatter so the sync search never wades
        # through foreign frames.
        if protocol == 'nmea':
            kvs = [(K_RATE_MEAS, meas_rate_ms, 2),
                   (K_MSGOUT_PVT_UART1, 0, 1)]
            for k in _NMEA_KEYS:
                kvs.append((k, 1, 1))
        else:
            kvs = [(K_RATE_MEAS, meas_rate_ms, 2),
                   (K_MSGOUT_PVT_UART1, 1, 1)]
            for k in _NMEA_KEYS:
                kvs.append((k, 0, 1))

        payload = valset_body(kvs)
        frame = ubx_frame(UBX_CLS_CFG, UBX_ID_VALSET, payload)
        # rate value bytes: sync(2)+cls/id/len(4)+valset-hdr(4)+key(4) = 14
        rate_off = 14
        # checksum covers cls..end-of-payload = [2, 6+len(payload)); ck after.
        ck_len = 4 + len(payload)
        ck_dst = 6 + len(payload)

        self.stage(frame, patch=("rate", rate_hz, meas_rate_ms, rate_off, 2))
        self.compute_checksum(ChecksumFletcher(),
                              start_off=2, length=ck_len, dst_off=ck_dst)
        self.send_staged(len(frame))
        self.sleep_ms(100)

        if protocol == 'nmea':
            self.set_output([
                {'name': 'nmea', 'type': 'string', 'count': 96,
                 'scale': 1.0, 'unit': ''},
            ])
            self.set_sample_size(96)
            return

        self.apply_nav_pvt_outputs()

    @StreamDriver.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "nmea"))
    def measure_nmea(self):
        self.read_until(b'\n', max=96)
        self.store_sample_n()
