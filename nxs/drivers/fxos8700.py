"""
FXOS8700 driver for NXS VM.

NXP FXOS8700CQ — 6-axis sensor: 3-axis linear accelerometer (14-bit,
+/-2/4/8 g) plus 3-axis magnetometer (16-bit, fixed +/-1200 uT). Single
die, one I2C address / SPI chip-select. Runs in hybrid mode so both
accelerometer and magnetometer sample every cycle; a hardware
auto-increment lets one 12-byte burst from OUT_X_MSB (0x01) read the 6
accel bytes and then roll over to M_OUT_X_MSB (0x33) for the 6 mag bytes.
The uncalibrated die-temperature register (0x51) is published as `temp`.

Both I2C (default, Click pre-soldered) and SPI are supported. SPI frames
an 8-bit register address across two bytes ([R/W|A6:0][A7|xxxxxxx]) with
R/W=0 for read, mode 0, 1 MHz max.

Bus / trigger:
    bus       i2c (default) | spi          (nxs set bus / --config bus=spi)
    trigger   drdy (default) | poll        (INT1 data-ready on mikroBUS INT)

Config keys:
    sample_rate  400 | 200 | 100 | 50 | 25   Hz   (default 100)  hybrid ODR
    accel_fs     2 | 4 | 8                    g    (default 4)
    (magnetometer range is fixed at +/-1200 uT in silicon — no param.)

Fixed operating modes (stated for capability parity):
    - Accelerometer oversampling fixed to high-resolution (CTRL_REG2
      mods=0b10) for lowest accel noise.
    - Magnetometer oversampling fixed to max (M_CTRL_REG1 m_os=0b111).
    - Magnetometer auto-calibration (m_acal, hard-iron offset) left OFF —
      raw field is published; hard-iron calibration is a host concern.
    - Fast-read (8-bit) mode left OFF: 14-bit data is required for the
      hybrid auto-increment 12-byte burst.
    - The embedded motion functions (FIFO, tap, orientation, freefall,
      transient, vector-magnitude) are outside the streaming datapath and
      are not configured; the FIFO is accelerometer-only and unavailable
      in hybrid mode, so the datapath uses status-gated direct reads.

Sub-25 Hz hybrid rates (6.25 / 3.125 / 0.7813 Hz) are omitted:
`sample_rate` is an integer-Hz parameter and these rows are fractional.

Note: the die-temperature sensor is uncalibrated (device-to-device
variation, no factory zero) and is only valid while the magnetometer is
active — which it always is here (hybrid mode). It is published raw-scaled
per the datasheet transfer function (0.96 C/LSB).

mikroBUS pin mapping (from mikroE 6DOF IMU 3 Click C driver):
    INT (PA9)  -> INT1: accelerometer data-ready interrupt (input)
    RST (PB2)  -> RST:  device reset (output, active HIGH)
    SCL/SDA          -> I2C
    SCK/MISO/MOSI/CS -> SPI

Datasheet: NXP FXOS8700CQ Rev. 8.0 (25 Apr 2017)
mikroE Click: https://www.mikroe.com/6dof-imu-3-click
"""

from nxs import RegisterDriver, Sample, SpiProfile


class Fxos8700(RegisterDriver):
    # ── Buses ───────────────────────────────────────────
    # Silicon supports both I2C and SPI; the Click is pre-soldered to I2C.
    BUSES = ('i2c', 'spi')
    BUS = 'i2c'

    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'drdy': 'mkbus_int',   # INT1 — accelerometer data ready (input)
    }

    # ── Device ID ───────────────────────────────────────
    WHO_AM_I_REG = 0x0D
    WHO_AM_I_VALUES = [0xC7]           # 0xC7 = production silicon

    # ── I2C strap options ───────────────────────────────
    # SA1:SA0 pin straps select one of four addresses. NXS scans these
    # on load and latches the first WHO_AM_I match.
    I2C_ADDRS = [0x1E, 0x1D, 0x1C, 0x1F]

    # ── Reset polarity ──────────────────────────────────
    # RST pin is active HIGH (DS pin table, pin 16: "Reset input, active
    # high"). The Click routes mikroBUS RST to this pin, so the platform
    # hard-resets the part before every probe — configure() issues no
    # software reset and writes config registers in the post-reset standby.
    RESET_ACTIVE = 'high'

    # ── Communication profile (SPI wire protocol) ───────
    # DS 10.2 (SPI): address framed across two bytes —
    #   byte0 = [R/W, A6..A0]  (R/W: read=0, write=1)
    #   byte1 = [A7, x..x]     (A7=0 for all regs <= 0x78, so byte1=0x00)
    # Mode 0, MSB first. DS Table 12 SPI timing: SCLK max = 1 MHz
    # (== firmware SPI baseline, so max_hz is a no-op and omitted).
    # I2C tops out at 400 kHz (DS Table 10) == board bus, so no I2cProfile.
    SPI_PROFILE = SpiProfile(addr_bytes=2, rw_read_level=0)   # DS 10.2 / Table 12

    # ── Accelerometer full-scale ────────────────────────
    # XYZ_DATA_CFG (0x0E) fs[1:0]. DS Table 61. Output is 14-bit,
    # left-justified, 2's complement in 16 bits → reading as int16 gives
    # count14 << 2, which spans +/-full-scale across the int16 range. So the
    # standard IMU base scale applies: sensitivity = 32768/range LSB per g,
    # base = 9.80665 / 32768 m/s^2 per int16 LSB, scaled live by accel_fs.
    ACCEL_FS = {2: 0x00, 4: 0x01, 8: 0x02}       # g
    ACCEL_BASE_SCALE = 9.80665 / 32768.0         # m/s^2 per int16 LSB, per g

    # ── Magnetometer scale ──────────────────────────────
    # Fixed +/-1200 uT range, 0.1 uT/LSB (DS Table 4), 16-bit signed.
    # Frame: DS Fig 4 labels the axis arrows "+Ax,+Mx" / "+Ay,+My" /
    # "+Az,+Mz" — accelerometer and magnetometer share ONE body frame with
    # matching signs, so no per-axis sign correction is applied.
    MAG_SCALE = 0.1e-6                            # tesla per LSB (0.1 uT)

    # ── Output data rate (hybrid mode) ──────────────────
    # CTRL_REG1 (0x2A) dr[2:0] in bits[5:3]. DS Table 35 hybrid column
    # (single-sensor ODR halved in hybrid mode). Values are the register
    # field pre-shifted to bits[5:3]; the active bit (0x01) is OR'd in at
    # the write.
    SAMPLE_RATE = {400: 0x00, 200: 0x08, 100: 0x10, 50: 0x18, 25: 0x20}   # Hz

    # ── Die-temperature transfer ────────────────────────
    # TEMP (0x51): 8-bit signed, 0.96 C/LSB, zero = 0 C (DS 14.3.1).
    # kelvin = raw * 0.96 + 273.15.
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
        self.declare_param("sample_rate", values=[400, 200, 100, 50, 25],
                           default=100, unit="Hz")
        self.declare_param("accel_fs", values=[2, 4, 8],
                           default=4, unit="g")

        sample_rate = config.get('sample_rate', 100)
        accel_fs = config.get('accel_fs', 4)

        # 2. Config-register writes. The Click routes mikroBUS RST to the
        #    part, so the platform hard-resets it before every probe/reload
        #    and the part is in STANDBY here (CTRL_REG1.active = 0 at reset).
        #    All config registers below are standby-writable; CTRL_REG1 is
        #    written LAST to select the rate and drive the part active.

        # Accelerometer full-scale (2/4/8 g).
        self.write(0x0E, self.ACCEL_FS[accel_fs],           # XYZ_DATA_CFG
                   param=("accel_fs", accel_fs))
        # Accelerometer oversampling: high-resolution (mods=0b10), lowest
        # noise. rst=0 (no software reset — the platform hard-resets).
        self.write(0x2B, 0x02)                              # CTRL_REG2
        # Magnetometer: hybrid mode (m_hms=0b11) + max oversample
        # (m_os=0b111); auto-calibration off (raw field).
        self.write(0x5B, 0x1F)                              # M_CTRL_REG1
        # Hybrid auto-increment: a burst from 0x01 rolls 0x06 -> 0x33 so
        # accel(6) + mag(6) read out as one 12-byte block.
        self.write(0x5C, 0x20)                              # M_CTRL_REG2
        # Interrupt pins: active-high push-pull.
        self.write(0x2C, 0x02)                              # CTRL_REG3 ipol=1
        # Enable data-ready interrupt and route it to INT1 (the pin the
        # Click wires to mikroBUS INT). Harmless when trigger=poll.
        self.write(0x2D, 0x01)                              # CTRL_REG4 int_en_drdy
        self.write(0x2E, 0x01)                              # CTRL_REG5 int_cfg_drdy=INT1
        # Rate select + leave standby (active=1) — LAST write.
        self.write(0x2A, self.SAMPLE_RATE[sample_rate] | 0x01,   # CTRL_REG1
                   param=("sample_rate", sample_rate))

        # 3. Output vector: accel (SI, live-scaled by accel_fs) + mag (SI,
        #    fixed) + die temperature (kelvin). Fields pack sequentially and
        #    match the measure() buffer layout: accel 0..5, mag 6..11,
        #    temp 12.
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

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x00)             # STATUS / DR_STATUS
        if not (status & 0x08):                  # zyxdr — new accel data
            return None
        raw = self.read_burst(0x01, 12)      # accel XYZ + mag XYZ (auto-inc)
        self.read_burst(0x51, 1, into=12)    # die temperature
        return Sample(raw)
