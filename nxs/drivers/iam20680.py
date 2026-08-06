"""
IAM-20680 driver for NXS VM.

TDK InvenSense IAM-20680 — automotive 6-axis MotionTracking IMU
(3-axis gyroscope + 3-axis accelerometer + die temperature) in a
conventional InvenSense register bus. Dual interface: I2C (up to
400 kHz) and SPI mode 0 (up to 8 MHz, read = reg | 0x80). Signed
16-bit big-endian sample registers; on-chip 512-byte headerless FIFO
(accel -> temp -> gyro, fixed slot order) and a RAW_DATA_RDY interrupt
on the INT pin.

Data path: one 14-byte packet (accel 6 + temp 2 + gyro 6). At the top
declared rate of 1000 Hz the direct-burst wire time on I2C 400 kHz
(~500 us) exceeds the period/3 budget (333 us), so the headerless FIFO
path is used — it freezes each sample and turns a late DRDY service
into backlog instead of a torn read. Gate: DRDY (mikroBUS INT).

Config keys:
    sample_rate : 10 | 25 | 50 | 100 | 200 | 250 | 500 | 1000  (Hz, default 100)
    accel_fs    : 2 | 4 | 8 | 16                                (g, default 8)
    gyro_fs     : 250 | 500 | 1000 | 2000                       (dps, default 2000)
    accel_bw    : 5 | 10 | 21 | 45 | 99 | 218 | 420             (Hz, default 420)
    gyro_bw     : 5 | 10 | 20 | 41 | 92 | 176                   (Hz, default 176)
    trigger     : drdy | poll                                   (default drdy)
    bus         : i2c | spi                                     (default i2c)

mikroBUS pin mapping (from mikroE 6DOF IMU 9 Click / IAM-20680 C driver):
    INT (PA9)  -> INT:  RAW_DATA_RDY data-ready interrupt (input)
    SCL/SDA    -> I2C (AD0 strap selects 0x68 / 0x69)
    SCK/MISO/MOSI/CS -> SPI mode 0
    PWM (PA10) -> FSYNC: external frame-sync input (unused here)
    RST: not routed to the sensor — the IAM-20680 has no reset pin;
         reset is the PWR_MGMT_1 DEVICE_RESET bit, issued in configure().

Datasheet: TDK InvenSense IAM-20680 (DS-000196)
mikroE Click: https://www.mikroe.com/6dof-imu-9-click
"""

from nxs import RegisterDriver, Sample, SpiProfile

_G = 9.80665                          # m/s^2, standard gravity
_DEG_TO_RAD = 0.017453292519943295    # pi / 180


class Iam20680(RegisterDriver):
    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'drdy': 'mkbus_int',   # RAW_DATA_RDY interrupt (input)
    }

    # ── Device ID ───────────────────────────────────────
    WHO_AM_I_REG = 0x75           # WHO_AM_I
    WHO_AM_I_VALUES = [0xA9]      # IAM-20680 identity

    # ── I2C strap options ───────────────────────────────
    # AD0 pin strap selects the address LSB. NXS scans both on load and
    # latches the first WHO_AM_I match.
    I2C_ADDRS = [0x68, 0x69]

    # ── Communication profile ───────────────────────────
    # Dual-bus part (BUSES defaults to ('i2c', 'spi')). Default transport
    # is the Click's pre-soldered I2C; switch with --config bus=spi or
    # `nxs set bus spi`, no recompile. SPI is a conventional InvenSense
    # register bus (mode 0, read = reg | 0x80); only the clock ceiling
    # deviates from the 1 MHz firmware baseline. I2C max is 400 kHz =
    # the board bus, so no I2cProfile is needed (it would be a no-op).
    SPI_PROFILE = SpiProfile(max_hz=8_000_000)   # DS SPI timing: 8 MHz, all registers
    BUS = 'i2c'                                   # compile-time default transport

    # ── Register map (IAM-20680) ────────────────────────
    REG_SMPLRT_DIV    = 0x19
    REG_CONFIG        = 0x1A      # FIFO_MODE(6) | EXT_SYNC(5:3) | DLPF_CFG(2:0)
    REG_GYRO_CONFIG   = 0x1B      # FS_SEL(4:3) | FCHOICE_B(1:0)
    REG_ACCEL_CONFIG  = 0x1C      # ACCEL_FS_SEL(4:3)
    REG_ACCEL_CONFIG2 = 0x1D      # DEC2_CFG(5:4) | ACCEL_FCHOICE_B(3) | A_DLPF_CFG(2:0)
    REG_FIFO_EN       = 0x23
    REG_INT_PIN_CFG   = 0x37
    REG_INT_ENABLE    = 0x38
    REG_USER_CTRL     = 0x6A      # FIFO_EN(6) | FIFO_RST(2)
    REG_PWR_MGMT_1    = 0x6B      # DEVICE_RESET(7) | SLEEP(6) | CLKSEL(2:0)
    REG_PWR_MGMT_2    = 0x6C
    REG_FIFO_COUNTH   = 0x72
    REG_FIFO_R_W      = 0x74

    # ── Full-scale lookup tables (register value only) ──
    # ACCEL_FS_SEL / GYRO_FS_SEL live in bits [4:3]; the base scale below
    # is multiplied on-device by the live full-scale param (scale_param).
    ACCEL_FS = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}                # g
    GYRO_FS  = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}       # dps

    # Signed 16-bit reading spans +/- the full-scale range, so sensitivity
    # is 32768/range LSB per unit and the range-independent base scale is
    # unit/32768; the live range param multiplies it back per sample.
    ACCEL_BASE_SCALE = _G / 32768.0          # m/s^2 per LSB, per g
    GYRO_BASE_SCALE  = _DEG_TO_RAD / 32768.0  # rad/s  per LSB, per dps

    # Temperature: Temp_degC = TEMP_OUT / 326.8 + 25  (RoomTemp_Offset 0).
    # kelvin -> scale 1/326.8, offset 25 + 273.15.
    TEMP_SCALE  = 1.0 / 326.8
    TEMP_OFFSET = 298.15

    # ── Digital LPF cutoff -> register code ─────────────
    # GYRO DLPF (CONFIG DLPF_CFG, valid when GYRO_CONFIG FCHOICE_B=00):
    # only the codes that keep the 1 kHz base rate the SMPLRT_DIV divider
    # divides are exposed. Codes 0 (250 Hz) and 7 (3281 Hz) switch the
    # base to 8 kHz and are excluded so every declared sample_rate holds.
    GYRO_DLPF  = {176: 0x01, 92: 0x02, 41: 0x03, 20: 0x04, 10: 0x05, 5: 0x06}  # Hz
    # ACCEL DLPF (ACCEL_CONFIG2 A_DLPF_CFG, ACCEL_FCHOICE_B=0): all codes
    # keep the 1 kHz base rate. Codes 0 and 1 are both 218 Hz (deduped).
    ACCEL_DLPF = {420: 0x07, 218: 0x01, 99: 0x02, 45: 0x03, 21: 0x04, 10: 0x05, 5: 0x06}  # Hz

    def __init__(self):
        super().__init__()
        self._read_responses = {
            self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who in self.WHO_AM_I_VALUES

    def configure(self, config):
        # 1. Declare every runtime-tunable parameter.
        self.declare_param("sample_rate",
                           values=[10, 25, 50, 100, 200, 250, 500, 1000],
                           default=100, unit="Hz")
        self.declare_param("accel_fs", values=[2, 4, 8, 16],
                           default=8, unit="g")
        self.declare_param("gyro_fs", values=[250, 500, 1000, 2000],
                           default=2000, unit="dps")
        self.declare_param("accel_bw", values=[5, 10, 21, 45, 99, 218, 420],
                           default=420, unit="Hz")
        self.declare_param("gyro_bw", values=[5, 10, 20, 41, 92, 176],
                           default=176, unit="Hz")

        # 2. Read config with defaults.
        sample_rate = config.get('sample_rate', 100)
        accel_fs = config.get('accel_fs', 8)
        gyro_fs = config.get('gyro_fs', 2000)
        accel_bw = config.get('accel_bw', 420)
        gyro_bw = config.get('gyro_bw', 176)

        # SMPLRT_DIV divides the 1 kHz base rate: div = 1000/rate - 1.
        rate_div = max(0, min(255, 1000 // sample_rate - 1))

        # 3. Reset. The Click does not route mikroBUS RST to the sensor and
        #    the IAM-20680 has no reset pin, so the platform's bind-time
        #    pulse never reaches it — issue the documented software reset
        #    first, standalone, then wait. 200 ms = 2x the datasheet's 100 ms
        #    power-up-ready maximum (DS-000196 start-up spec), not a vendor
        #    driver's delay constant.
        self.write(self.REG_PWR_MGMT_1, 0x80)   # DEVICE_RESET
        self.sleep_ms(200)
        self.write(self.REG_PWR_MGMT_1, 0x01)   # wake (SLEEP=0), CLKSEL=auto/PLL
        self.sleep_ms(2)
        self.write(self.REG_PWR_MGMT_2, 0x00)   # enable all accel + gyro axes

        # 4. Config-dependent writes — each param owns one register.
        self.write(self.REG_SMPLRT_DIV, rate_div,
                   param=("sample_rate", sample_rate))
        self.write(self.REG_CONFIG, self.GYRO_DLPF[gyro_bw],
                   param=("gyro_bw", gyro_bw))          # FIFO_MODE=0 (overwrite), EXT_SYNC=0
        self.write(self.REG_GYRO_CONFIG, self.GYRO_FS[gyro_fs],
                   param=("gyro_fs", gyro_fs))          # FCHOICE_B=00 -> gyro DLPF active
        self.write(self.REG_ACCEL_CONFIG, self.ACCEL_FS[accel_fs],
                   param=("accel_fs", accel_fs))
        self.write(self.REG_ACCEL_CONFIG2, self.ACCEL_DLPF[accel_bw],
                   param=("accel_bw", accel_bw))        # ACCEL_FCHOICE_B=0 -> accel DLPF active

        # 5. Data-ready interrupt routing (fixed). INT_PIN_CFG default 0x00
        #    (active-high, push-pull, pulse). INT_ENABLE bit0 = RAW_RDY_EN,
        #    off at power-on, so it is enabled here for the DRDY trigger.
        self.write(self.REG_INT_PIN_CFG, 0x00)
        self.write(self.REG_INT_ENABLE, 0x01)

        # 6. FIFO: queue accel + temp + gyro (fixed packet order), then
        #    flush stale bytes and enable in one write.
        self.write(self.REG_FIFO_EN, 0xF8)      # TEMP | GYRO_XYZ | ACCEL
        self.write(self.REG_USER_CTRL, 0x44)    # FIFO_RST | FIFO_EN

        # 7. Output vector — FIFO packet order: accel XYZ, temp, gyro XYZ
        #    (14 bytes, signed 16-bit big-endian). Accel/gyro carry the base
        #    scale + scale_param so SI tracks a runtime full-scale change.
        self.set_output([
            {'name': 'accel_x', 'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'accel_y', 'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'accel_z', 'scale': self.ACCEL_BASE_SCALE, 'scale_param': 'accel_fs'},
            {'name': 'temp',    'scale': self.TEMP_SCALE, 'offset': self.TEMP_OFFSET},
            {'name': 'gyro_x',  'scale': self.GYRO_BASE_SCALE, 'scale_param': 'gyro_fs'},
            {'name': 'gyro_y',  'scale': self.GYRO_BASE_SCALE, 'scale_param': 'gyro_fs'},
            {'name': 'gyro_z',  'scale': self.GYRO_BASE_SCALE, 'scale_param': 'gyro_fs'},
        ])
        self.set_sample_size(14)

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        # Headerless FIFO, one 14-byte packet per DRDY, tiered count gate:
        # wait while short, read through a bounded backlog, resync only when
        # the queue runs away or a read tears. Register addresses are
        # literals here (measure() compiles to bytecode).
        count = self.read(0x72, 2)          # FIFO_COUNTH:L — bytes buffered, big-endian
        if count < 14:
            return None                     # frame incomplete — wait for next DRDY
        if count > 56:
            self.write(0x6A, 0x44)          # >4 frames behind — resync (USER_CTRL: FIFO_RST | FIFO_EN)
            return None
        raw = self.read_burst(0x74, 14)     # FIFO_R_W — atomic packet -> sample buffer
        after = self.read(0x72, 2)
        left = count - 14
        d = after - left
        if d == 0:
            return Sample(raw)              # clean read
        d = d - 14
        if d == 0:
            return Sample(raw)              # one frame landed mid-read
        self.write(0x6A, 0x44)             # torn read — drop + resync
        return None
