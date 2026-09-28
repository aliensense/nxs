"""
ZED-F9P driver for NXS VM.

u-blox ZED-F9P multi-band RTK GNSS receiver (module ZED-F9P-05B); ZED-F9P Integration
    Manual and Interface Description (UBX-NAV-PVT layout, UBX-CFG-VALSET keys).
Bus: UART1, 38400 baud (fixed); UBX binary by default, NMEA variant; CFG-VALSET to the RAM layer.
Config: protocol binary|nmea (binary), compile-time. Params: rate 1|2|5|10 Hz (1), CFG-RATE-MEAS.
Outputs: UBX-NAV-PVT fields (see UbxNavPvtDriver), or nmea (string, 96 bytes).
Pins: INT -> TIMEPULSE (PPS). AN -> TXD_READY, RST -> RTK LED (not reset), PWM -> RESET_N: unused.
mikroE Click: https://www.mikroe.com/gnss-rtk-click (MIKROE-4456)
"""

from nxs import StreamDriver
from nxs.compiler import ChecksumFletcher

from nxs.drivers._ubx_nav_pvt import (UbxNavPvtDriver, UBX_CLS_CFG,
                                      UBX_ID_VALSET, K_MSGOUT_PVT_UART1,
                                      K_RATE_MEAS, ubx_frame, valset_body,
                                      valset_frame)

# F9 CFG-MSGOUT keys (group 0x2091, per-port output rate, U1) for the NMEA
# sentences this driver routes; the shared rate/PVT keys are in _ubx_nav_pvt.
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
    """u-blox ZED-F9P RTK GNSS receiver, UBX binary (default) and NMEA."""

    # Power-on default; fixed, since retuning needs a coordinated chip and
    # host baud change the poll loop cannot make.
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
        self.declare_params_from_descriptor()

        # UART part: the driver pins the VM poll cadence; the receiver's
        # own epoch rate keeps the `rate` name.
        config['sample_rate'] = 100
        config['trigger'] = 'poll'

        rate_hz = config.get('rate', 1)
        meas_rate_ms = self.RATES[rate_hz]

        # One VALSET: the rate, the framed record enabled, and the other
        # protocol's output disabled so the sync search sees no foreign frames.
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
