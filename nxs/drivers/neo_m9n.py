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


_PI = 3.141592653589793
_DEG_TO_RAD = _PI / 180.0


class NeoM9n(StreamDriver):
    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'pps': 'mkbus_int',        # 1PPS time pulse (input)
    }

    DEFAULT_BAUD = 38400

    # CFG-RATE-MEAS is the measurement period in ms: rate_hz -> 1000 // rate_hz.
    RATES = [1, 5, 10, 25]

    # ── UBX config-database key IDs (M9). Value size is encoded in bits
    #    30:28 of the key ID: 0x1..=bit(L), 0x2..=U1, 0x3..=U2, 0x4..=U4.
    _K_RATE_MEAS        = 0x30210001   # U2, ms  (nominal time between measurements)
    _K_UART1OUT_UBX     = 0x10740001   # L       (enable UBX output on UART1)
    _K_UART1OUT_NMEA    = 0x10740002   # L       (enable NMEA output on UART1)
    _K_MSGOUT_PVT_UART1 = 0x20910007   # U1      (UBX-NAV-PVT output rate on UART1)

    # ── UBX framing helpers. Plain Python, evaluated at compile time inside
    #    probe()/configure() (traced, not AST-compiled), so a loop and byte
    #    assembly are fine here — they never reach measure().
    @staticmethod
    def _ubx(msg_class, msg_id, payload):
        """Wrap a UBX frame: B5 62 | class id len(LE) | payload | CK_A CK_B.
        8-bit Fletcher checksum over class + id + len + payload."""
        body = bytes([msg_class, msg_id,
                      len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + bytes(payload)
        ck_a = 0
        ck_b = 0
        for b in body:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return b'\xb5\x62' + body + bytes([ck_a, ck_b])

    @staticmethod
    def _key(key_id):
        """4-byte little-endian config key ID."""
        return bytes([key_id & 0xFF, (key_id >> 8) & 0xFF,
                      (key_id >> 16) & 0xFF, (key_id >> 24) & 0xFF])

    @classmethod
    def _valset(cls, keyvals):
        """CFG-VALSET frame (0x06 0x8A) targeting the RAM/volatile layer.
        `keyvals` is a list of (key_id, value_bytes)."""
        payload = bytes([0x00, 0x01, 0x00, 0x00])     # version, layers=RAM(0x01), reserved
        for key_id, val in keyvals:
            payload += cls._key(key_id) + bytes(val)
        return cls._ubx(0x06, 0x8A, payload)

    # ── Probe: fixed CFG-VALSET (RATE-MEAS = 1000 ms), verify UBX-ACK-ACK ──
    def probe(self):
        self.set_baud(self.DEFAULT_BAUD)
        self.sleep_ms(100)
        # Fixed CFG-VALSET: CFG-RATE-MEAS = 1000 ms (1 Hz), RAM layer.
        self.write(self._valset([(self._K_RATE_MEAS, bytes([0xE8, 0x03]))]))
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
            self.write(self._valset([
                (self._K_UART1OUT_UBX, bytes([0x01])),
                (self._K_UART1OUT_NMEA, bytes([0x00])),
                (self._K_MSGOUT_PVT_UART1, bytes([0x01])),
            ]))
        else:
            # Enable NMEA on UART1 and silence the binary PVT so the line is
            # pure NMEA sentences.
            self.write(self._valset([
                (self._K_UART1OUT_NMEA, bytes([0x01])),
                (self._K_MSGOUT_PVT_UART1, bytes([0x00])),
            ]))

        # ── Rate command: patchable value + runtime-recomputed checksum ──
        # Frame: B5 62 06 8A 0A 00 | ver layers res res | key(4) | rate(2) | CK CK
        # rate value at frame offset 14 (size 2); Fletcher over [2,16) -> [16,18).
        frame = (b'\xb5\x62' +
                 bytes([0x06, 0x8A, 0x0A, 0x00,               # class, id, len=10 (LE)
                        0x00, 0x01, 0x00, 0x00]) +            # version, layers=RAM, reserved
                 self._key(self._K_RATE_MEAS) +               # key CFG-RATE-MEAS (LE)
                 bytes([meas_rate_ms & 0xFF, (meas_rate_ms >> 8) & 0xFF,
                        0x00, 0x00]))                         # rate (LE) + placeholder checksum
        self.stage(frame, patch=("rate", rate_hz, meas_rate_ms, 14, 2))
        self.compute_checksum(ChecksumFletcher(), start_off=2, length=14, dst_off=16)
        self.send_staged(len(frame))
        self.sleep_ms(100)

        # ── Output vector ──
        if protocol == 'binary':
            lla = 1e-7 * _DEG_TO_RAD     # 1e-7 deg -> rad (lat/lon)
            hdg = 1e-5 * _DEG_TO_RAD     # 1e-5 deg -> rad (heading)
            mm = 0.001                    # mm -> m, mm/s -> m/s
            # UBX-NAV-PVT frame committed in sample_buf as:
            #   [class, id, len_lo, len_hi, payload(92), CK_A, CK_B]
            # so payload byte P sits at sample offset 4 + P.
            self.set_output([
                {'name': 'fix_type', 'type': 'uint8', 'at': 24,          # pl 20
                 'scale': 1.0, 'unit': ''},
                {'name': 'num_sv', 'type': 'uint8', 'at': 27,            # pl 23
                 'scale': 1.0, 'unit': ''},
                {'name': 'longitude', 'type': 'int32', 'byte_order': 'little',
                 'at': 28, 'scale': lla},                                 # pl 24, deg 1e-7 -> rad
                {'name': 'latitude', 'type': 'int32', 'byte_order': 'little',
                 'at': 32, 'scale': lla},                                 # pl 28
                {'name': 'alt_ellipsoid', 'type': 'int32', 'byte_order': 'little',
                 'at': 36, 'scale': mm, 'unit': 'm'},                     # pl 32, height above ellipsoid
                {'name': 'altitude', 'type': 'int32', 'byte_order': 'little',
                 'at': 40, 'scale': mm},                                  # pl 36, hMSL (mean sea level)
                {'name': 'pos_h_acc', 'type': 'uint32', 'byte_order': 'little',
                 'at': 44, 'scale': mm},                                  # pl 40, hAcc
                {'name': 'pos_v_acc', 'type': 'uint32', 'byte_order': 'little',
                 'at': 48, 'scale': mm},                                  # pl 44, vAcc
                {'name': 'vel_north', 'type': 'int32', 'byte_order': 'little',
                 'at': 52, 'scale': mm},                                  # pl 48, velN
                {'name': 'vel_east', 'type': 'int32', 'byte_order': 'little',
                 'at': 56, 'scale': mm},                                  # pl 52, velE
                {'name': 'vel_down', 'type': 'int32', 'byte_order': 'little',
                 'at': 60, 'scale': mm},                                  # pl 56, velD
                {'name': 'speed', 'type': 'int32', 'byte_order': 'little',
                 'at': 64, 'scale': mm},                                  # pl 60, gSpeed (2-D)
                {'name': 'heading', 'type': 'int32', 'byte_order': 'little',
                 'at': 68, 'scale': hdg, 'unit': 'rad'},                  # pl 64, headMot
                {'name': 'vel_s_acc', 'type': 'uint32', 'byte_order': 'little',
                 'at': 72, 'scale': mm},                                  # pl 68, sAcc
                {'name': 'pdop', 'type': 'uint16', 'byte_order': 'little',
                 'at': 80, 'scale': 0.01, 'unit': ''},                    # pl 76
            ])
            self.set_sample_size(98)     # class+id+len(4) + payload(92) + CK(2)
        else:
            self.set_output([
                {'name': 'nmea', 'type': 'string', 'count': 82,
                 'scale': 1.0, 'unit': ''},
            ])
            self.set_sample_size(82)     # NMEA max sentence length

    # ── Binary UBX-NAV-PVT: the default protocol. ──
    @StreamDriver.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "binary"), default=True)
    def measure_binary(self):
        self.read_until(b'\xb5\x62')            # sync to a UBX frame
        self.read_n(98)                          # class+id+len+payload(92)+CK
        m = self.match(0x01, 0x07, 0x5c, 0x00)   # NAV-PVT (0x01 0x07), len=92 (LE)
        if m != 0:
            return None                          # not NAV-PVT -> resync next pass
        bad = self.verify_checksum(ChecksumFletcher(), 0, 96, 96)
        if bad != 0:
            return None                          # corrupt frame -> drop
        self.store_sample()                      # fixed 98-B frame at buf[0]

    # ── NMEA sentences: secondary protocol variant. ──
    @StreamDriver.measure_loop(trigger="poll", sample_rate=100,
                               when=("protocol", "nmea"))
    def measure_nmea(self):
        self.read_until(b'\n', max=82)           # one sentence
        self.store_sample_n()
