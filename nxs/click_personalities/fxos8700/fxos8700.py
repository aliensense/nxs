"""
FXOS8700 personality for NXS VM.

NXP FXOS8700CQ 6-axis accelerometer and magnetometer; datasheet Rev. 8.0.
Bus: I2C 400 kHz at 0x1E/0x1D/0x1C/0x1F (SA1:SA0 straps), default; SPI mode 0, 1 MHz max.
Config: sample_rate 400|200|100|50|25 Hz (100); accel_fs 2|4|8 g (4);
    trigger drdy|poll (drdy); bus i2c|spi (i2c). Mag range fixed at +/-1200 uT.
Outputs: accel_x/y/z (m/s^2), mag_x/y/z (T), temp (K, uncalibrated die sensor).
Pins: INT -> INT1 (accelerometer data ready); RST -> RST (active high).
mikroE Click: https://www.mikroe.com/6dof-imu-3-click
"""

from nxs import RegisterClickPersonality, Sample, SpiProfile


class Fxos8700(RegisterClickPersonality):
    # ── Buses ───────────────────────────────────────────
    # Silicon supports both I2C and SPI; the Click is pre-soldered to I2C.
    BUSES = ('i2c', 'spi')
    BUS = 'i2c'

    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'drdy': 'mkbus_int',   # INT1: accelerometer data ready (input)
    }

    # ── Device ID ───────────────────────────────────────
    WHO_AM_I_REG = 0x0D
    WHO_AM_I_VALUES = [0xC7]           # 0xC7 = production silicon

    # ── I2C strap options ───────────────────────────────
    # SA1:SA0 straps; scanned on load, the first WHO_AM_I match is latched.
    I2C_ADDRS = [0x1E, 0x1D, 0x1C, 0x1F]

    # ── Reset polarity ──────────────────────────────────
    # RST is active high (DS pin table, pin 16). The Click routes mikroBUS
    # RST to it, so the platform hard-resets the part before every probe.
    RESET_ACTIVE = 'high'

    # ── Communication profile (SPI wire protocol) ───────
    # DS 10.2: the address spans two bytes, [R/W|A6..A0][A7|x..x], read = 0,
    # mode 0. SCLK max 1 MHz (DS Table 12) equals the baseline, so no max_hz.
    SPI_PROFILE = SpiProfile(addr_bytes=2, rw_read_level=0)   # DS 10.2 / Table 12

    # ── Accelerometer full-scale ────────────────────────
    # XYZ_DATA_CFG (0x0E) fs[1:0], DS Table 61. The 14-bit output is left-
    # justified in int16, so the base scale is per int16 LSB (32768 per range).
    ACCEL_FS = {2: 0x00, 4: 0x01, 8: 0x02}       # g
    ACCEL_BASE_SCALE = 9.80665 / 32768.0         # m/s^2 per int16 LSB, per g

    # ── Magnetometer scale ──────────────────────────────
    # Fixed +/-1200 uT, 0.1 uT/LSB (DS Table 4), int16. Accel and mag share
    # one body frame (DS Fig 4), so no axis sign correction.
    MAG_SCALE = 0.1e-6                            # tesla per LSB (0.1 uT)

    # ── Output data rate (hybrid mode) ──────────────────
    # CTRL_REG1 (0x2A) dr[2:0] pre-shifted to bits[5:3], DS Table 35 hybrid
    # column. The fractional rows (6.25/3.125/0.7813 Hz) are dropped: integer Hz.
    SAMPLE_RATE = {400: 0x00, 200: 0x08, 100: 0x10, 50: 0x18, 25: 0x20}   # Hz

    # ── Die-temperature transfer ────────────────────────
    # TEMP (0x51): int8, 0.96 C/LSB, zero at 0 C (DS 14.3.1); the sensor is
    # uncalibrated and valid only while the magnetometer runs (always, here).
    TEMP_SCALE = 0.96
    TEMP_OFFSET = 273.15

    def __init__(self):
        super().__init__()
        self._read_responses = {
            self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who in self.WHO_AM_I_VALUES

    def configure(self, config):
        # 1. Declare runtime parameters.
        self.declare_params_from_descriptor()

        sample_rate = config.get('sample_rate', 100)
        accel_fs = config.get('accel_fs', 4)

        # 2. Config writes in the post-reset standby (CTRL_REG1.active = 0);
        #    CTRL_REG1 goes last to select the rate and activate the part.

        # XYZ_DATA_CFG fs[1:0]: accelerometer full-scale (DS Table 61).
        self.write(0x0E, self.ACCEL_FS[accel_fs],           # XYZ_DATA_CFG
                   param=("accel_fs", accel_fs))
        # CTRL_REG2 mods=0b10: high-resolution oversampling, rst=0.
        self.write(0x2B, 0x02)                              # CTRL_REG2
        # M_CTRL_REG1: hybrid mode (m_hms=0b11), m_os=0b111, auto-calibration off.
        self.write(0x5B, 0x1F)                              # M_CTRL_REG1
        # M_CTRL_REG2 hyb_autoinc: a burst from 0x01 rolls 0x06 -> 0x33, so
        # accel (6) and mag (6) read as one 12-byte block.
        self.write(0x5C, 0x20)                              # M_CTRL_REG2
        # CTRL_REG3 ipol=1: active-high push-pull interrupt pins.
        self.write(0x2C, 0x02)                              # CTRL_REG3 ipol=1
        # CTRL_REG4/5: data-ready interrupt routed to INT1 (mikroBUS INT).
        self.write(0x2D, 0x01)                              # CTRL_REG4 int_en_drdy
        self.write(0x2E, 0x01)                              # CTRL_REG5 int_cfg_drdy=INT1
        # CTRL_REG1: rate select and active=1, the last write.
        self.write(0x2A, self.SAMPLE_RATE[sample_rate] | 0x01,   # CTRL_REG1
                   param=("sample_rate", sample_rate))

        # 3. Outputs in measure() buffer order: accel 0..5, mag 6..11, temp 12.
        #    The accel scale tracks the live accel_fs.
        self.set_output([
            {'name': 'accel_x', 'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'accel_y', 'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'accel_z', 'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'mag_x', 'scale': self.MAG_SCALE},
            {'name': 'mag_y', 'scale': self.MAG_SCALE},
            {'name': 'mag_z', 'scale': self.MAG_SCALE},
            {'name': 'temp', 'type': 'int8',
             'scale': self.TEMP_SCALE, 'offset': self.TEMP_OFFSET},
        ])
        self.set_sample_size(13)   # accel 6 + mag 6 + temp 1

    @RegisterClickPersonality.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x00)             # STATUS / DR_STATUS
        if not (status & 0x08):                  # zyxdr: new accel data
            return None
        raw = self.read_burst(0x01, 12)      # accel XYZ + mag XYZ (auto-inc)
        self.read_burst(0x51, 1, into=12)    # die temperature
        return Sample(raw)
