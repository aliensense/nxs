"""
IAM-20680 personality for NXS VM.

TDK InvenSense IAM-20680 3-axis gyroscope, 3-axis accelerometer; datasheet DS-000196 rev 1.1.
Bus: I2C 0x68/0x69 (SA0 strap, JP5 ADD), default; SPI modes 0 and 3, 8 MHz max.
Config: sample_rate 10|25|50|100|200|250 Hz (100); accel_fs 2|4|8|16 g (8);
    gyro_fs 250|500|1000|2000 dps (2000); accel_bw 5|10|21|45|99|218|420 Hz (420);
    gyro_bw 5|10|20|41|92|176 Hz (176); trigger drdy|poll (drdy); bus i2c|spi (i2c).
Outputs: accel_x/y/z (m/s^2), temp (K), gyro_x/y/z (rad/s).
Pins: INT -> mikroBUS INT (data ready); FSYNC -> mikroBUS PWM (unused); RST not routed.
mikroE Click: https://www.mikroe.com/6dof-imu-9-click
"""

from nxs import RegisterDriver, Sample, SpiProfile


class Iam20680(RegisterDriver):
    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'drdy': 'mkbus_int',   # INT pin 6, data ready (input)
    }

    # ── Device ID (DS 9.31) ─────────────────────────────
    WHO_AM_I_REG = 0x75
    WHO_AM_I_VALUES = [0xA9]

    # ── I²C strap options (DS 6.2) ──────────────────────
    # Both SA0 levels of the JP5 ADD strap; scanned on load, the first
    # WHO_AM_I match is latched.
    I2C_ADDRS = [0x68, 0x69]

    # ── Communication profiles ──────────────────────────
    # Conventional register bus on both interfaces: SPI read sets bit 7 of
    # the address byte, MSB first, modes 0 and 3 (DS 6.5, Table 5).
    BUSES = ('i2c', 'spi')
    SPI_PROFILE = SpiProfile(max_hz=8_000_000)   # DS Table 7, fSPC max
    BUS = 'i2c'                                  # COMM SEL jumpers ship on I2C

    # ── Full-scale tables ───────────────────────────────
    # GYRO_CONFIG FS_SEL (DS 9.11). Sensitivity is 32768 / FS LSB per dps;
    # Table 1 lists the rounded 131, 65.5, 32.8, 16.4.
    GYRO_FS = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}   # dps
    GYRO_BASE_SCALE = (3.141592653589793 / 180.0) / 32768.0    # rad/s per LSB, per dps

    # ACCEL_CONFIG ACCEL_FS_SEL (DS 9.12); 16384 LSB/g at 2 g (Table 2).
    ACCEL_FS = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}   # g
    ACCEL_BASE_SCALE = 9.80665 / 32768.0                # m/s^2 per LSB, per g

    # ── Filter tables ───────────────────────────────────
    # CONFIG DLPF_CFG, gyro 3-dB bandwidth at the 1 kHz internal rate (DS Table 16).
    # Codes 0 and 7 (250 Hz, 3281 Hz) run the 8 kHz rate that SMPLRT_DIV cannot divide.
    GYRO_BW = {176: 0x01, 92: 0x02, 41: 0x03, 20: 0x04, 10: 0x05, 5: 0x06}   # Hz

    # ACCEL_CONFIG2 A_DLPF_CFG, accel 3-dB bandwidth at the 1 kHz rate (DS Table 17).
    # Code 0 duplicates code 1 (218.1 Hz); ACCEL_FCHOICE_B=1 (1046 Hz) runs the 4 kHz rate.
    ACCEL_BW = {420: 0x07, 218: 0x01, 99: 0x02, 45: 0x03, 21: 0x04, 10: 0x05, 5: 0x06}   # Hz

    # ── Temperature (DS Table 4, 9.22) ──────────────────
    # 326.8 LSB/degC, 0 LSB at 25 degC; the Celsius zero folds into kelvin.
    TEMP_SCALE = 1.0 / 326.8
    TEMP_OFFSET = 298.15

    def __init__(self):
        super().__init__()
        self._read_responses = {
            self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who in self.WHO_AM_I_VALUES

    def configure(self, config):
        self.declare_params_from_descriptor()

        sample_rate = config.get('sample_rate', 100)
        accel_fs = config.get('accel_fs', 8)
        gyro_fs = config.get('gyro_fs', 2000)
        accel_bw = config.get('accel_bw', 420)
        gyro_bw = config.get('gyro_bw', 176)

        # SMPLRT_DIV divides the 1 kHz internal rate: ODR = 1000 / (1 + div) (DS 9.9).
        rate_div = 1000 // sample_rate - 1

        # PWR_MGMT_1 DEVICE_RESET (DS 9.27): the part has no reset pin and the
        # Click leaves mikroBUS RST unrouted, so this is the only reset it gets.
        self.write(0x6B, 0x80)
        self.sleep_ms(200)   # 2x start-up for register read/write, 100 ms max (DS Table 4)

        # PWR_MGMT_1: SLEEP=0, CLKSEL=1, auto-select PLL (DS 9.27 requires 001 for
        # full gyro performance). Duty-cycled low-power modes and wake-on-motion stay off.
        self.write(0x6B, 0x01)
        self.sleep_ms(70)    # 2x gyro start-up from sleep, 35 ms (DS Table 1)

        # The rate, filter and full-scale registers are not writable in sleep
        # mode (DS 8), so every write below follows the wake and its settle.
        self.write(0x19, rate_div,                      # SMPLRT_DIV (DS 9.9)
                   param=("sample_rate", sample_rate))
        self.write(0x1A, self.GYRO_BW[gyro_bw],         # CONFIG (DS 9.10): DLPF_CFG,
                   param=("gyro_bw", gyro_bw))          # EXT_SYNC_SET=0 leaves FSYNC unsampled
        self.write(0x1B, self.GYRO_FS[gyro_fs],         # GYRO_CONFIG (DS 9.11): FS_SEL, FCHOICE_B=00
                   param=("gyro_fs", gyro_fs))
        self.write(0x1C, self.ACCEL_FS[accel_fs],       # ACCEL_CONFIG (DS 9.12)
                   param=("accel_fs", accel_fs))
        self.write(0x1D, self.ACCEL_BW[accel_bw],       # ACCEL_CONFIG2 (DS 9.13): A_DLPF_CFG,
                   param=("accel_bw", accel_bw))        # ACCEL_FCHOICE_B=0, DEC2_CFG=0
        # INT_ENABLE DATA_RDY_INT_EN (DS 9.19); the reset value is 0, so the pin
        # needs this. INT_PIN_CFG keeps its reset value: active-high 50 us pulse (DS 9.18).
        self.write(0x38, 0x01)

        # Field order is the data-register order, big-endian int16 each
        # (DS 9.21 to 9.23). scale_param tracks the live full-scale.
        self.set_output([
            {'name': 'accel_x', 'type': 'int16', 'byte_order': 'big',
             'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'accel_y', 'type': 'int16', 'byte_order': 'big',
             'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'accel_z', 'type': 'int16', 'byte_order': 'big',
             'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'temp', 'type': 'int16', 'byte_order': 'big',
             'scale': self.TEMP_SCALE, 'offset': self.TEMP_OFFSET},
            {'name': 'gyro_x', 'type': 'int16', 'byte_order': 'big',
             'scale': self.GYRO_BASE_SCALE, 'scale_param': 'gyro_fs'},
            {'name': 'gyro_y', 'type': 'int16', 'byte_order': 'big',
             'scale': self.GYRO_BASE_SCALE, 'scale_param': 'gyro_fs'},
            {'name': 'gyro_z', 'type': 'int16', 'byte_order': 'big',
             'scale': self.GYRO_BASE_SCALE, 'scale_param': 'gyro_fs'},
        ])
        self.set_sample_size(14)   # ACCEL_XOUT_H .. GYRO_ZOUT_L

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x3A)          # INT_STATUS, cleared on read (DS 9.20)
        if not (status & 0x01):           # DATA_RDY_INT
            return None
        raw = self.read_burst(0x3B, 14)   # ACCEL_XOUT_H .. GYRO_ZOUT_L (DS 9.21 to 9.23)
        return Sample(raw)
