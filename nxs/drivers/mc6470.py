"""
MC6470 driver for NXS VM.

mCube MC6470 — 6-axis eCompass: a 3-axis accelerometer die and a
3-axis magnetometer die co-resident in one 2x2mm package, each an
independent I2C slave. I2C only, up to 400 kHz, 3.3V. One driver, one
fused sample: accel_x/y/z + mag_x/y/z.

Package model (see "Companion I2C devices"):
    primary die  = accelerometer, strap-selectable address 0x4C / 0x6C
    companion    = magnetometer,  fixed address 0x0C  (dev='mag')

Data path (both dies): output registers hold a sign-extended 16-bit
2's-complement value, little-endian (low address = LSB). Datasheet
p.54 (accel) "the MSB is the sign bit, sign extended to the higher
bits"; p.38 (mag) range 1FFFh(+8191)~E000h(-8192) — 0xE000 = -8192
confirms full 16-bit sign extension. So a plain signed int16 LE read
is correct for every axis regardless of the resolution setting; no
masking is needed on-device.

Trigger: the accelerometer drives INTA at its sample rate (ACQ_INT,
datasheet 6.4.2 "frequency ... is always the same as the sample
rate"). INTA -> mikroBUS INT -> the firmware DRDY input, so DRDY is
the default; poll is selectable. The magnetometer runs in Normal
(continuous) state so its registers stay fresh; the fused sample is
paced by the accelerometer, and the mag bytes stale-hold between mag
updates. INTA is configured push-pull, active-high (rising edge on
new data); flip MODE bit IAH (0xC1 -> 0x41) if the platform's DRDY
expects active-low.

Axis frame: datasheet Figure 3 defines the magnetometer axes
anti-parallel to the accelerometer axes on all three axes (accel +1g
<-> mag -|B|). The mag scale is therefore negated so accel and mag
share one right-handed device body frame (matching the mCube
reference driver's -1.0 orientation coefficient) — required for the
6-axis vectors to be usable together for tilt-compensated heading.

Config keys (rb_config.yaml `config:`):
    bus:         i2c            (MC6470 is I2C-only)
    trigger:     drdy (default) | poll
    accel_res:   6 | 7 | 8 | 10 | 12 | 14   (bit, default 14) — accel
                 output resolution; compile-time (it sets the output
                 scale, so it cannot ride the runtime accel_fs scale)
Runtime params (nxs set ...):
    sample_rate: 1 2 4 8 16 32 64 128 256   (Hz, default 64) — accel ODR
    accel_fs:    2 4 8 16                     (g,  default 8)
    mag_odr:     10 20 100                    (Hz, default 100) — mag continuous ODR
    mag_res:     14 15                        (bit, default 15)

    The parts' sub-hertz rate rows (accel 0.25/0.5 Hz, mag 0.5 Hz) are
    excluded: parameter values ride the descriptor wire as integers, so
    fractional hertz is not representable as an enum value.

Not exposed (documented but outside the streaming-vector model):
  - Accelerometer tap / double-tap detection (TAPEN/TTTRx/SRTFR taps):
    an interrupt-event feature, not a streamed measurand.
  - Accelerometer & magnetometer offset/gain trim (XOFF/XGAIN,
    OFFX..OFFZ) and self-test: per-chip factory calibration / BIST —
    left at factory defaults.
  - Accelerometer I2C watchdog timer (MODE I2C_WDT): bus-stall
    recovery, left disabled (default).
  - Magnetometer Force-State single-shot mode: an on-demand low-power
    mode; the driver uses Normal continuous for streaming fusion.
  - Magnetometer die temperature (TEMP, 1 C/LSB): only updates from a
    Force-State TCS-triggered measurement, incompatible with the
    Normal-continuous streaming mode used here; it feeds the mag's
    internal gain compensation. Excluded.
  - Magnetometer INTM data-ready pin: routes to mikroBUS AN, which is
    not the firmware DRDY input (only INT is); the fused sample is
    paced by the accelerometer's ACQ_INT.

mikroBUS pin mapping (from mikroE 6DOF IMU 13 Click C driver):
    INT (PA9)  -> INTA: accelerometer ACQ_INT data ready (input)
    AN  (PA0)  -> INTM: magnetometer data ready (input, unused by NXS)
    SDA        -> SDA:  I2C data
    SCL        -> SCL:  I2C clock
    RST/PWM/CS -> not connected (MC6470 is I2C-only, no reset pin)

Datasheet: MC6470 6-Axis Sensor, mCube APS-048-0033v1.7
mikroE Click: https://www.mikroe.com/6dof-imu-13-click
"""

from nxs import RegisterDriver, Sample


class Mc6470(RegisterDriver):
    # ── Bus ─────────────────────────────────────────────
    # Companion parts are I2C-only; the MC6470 has no SPI/UART variant.
    BUSES = ('i2c',)

    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'drdy': 'mkbus_int',   # accelerometer INTA / ACQ_INT (input)
    }

    # ── Primary die = accelerometer ─────────────────────
    # A5 strap selects the LSB of the accel address (GND -> 0x4C,
    # VDD -> 0x6C); NXS scans both on load and latches the one that
    # answers. The magnetometer companion is at a fixed 0x0C.
    I2C_ADDRS = [0x4C, 0x6C]

    # Accel identity is the PCODE register. Bits [3:1] are factory-
    # variable ("ignore" per the datasheet); silicon reads PCODE with
    # bit 4 set (0x10 observed at reg 0x3B on hardware), contradicting
    # the datasheet's fixed-zero claim on bits [7:4] — the set therefore
    # enumerates the 0x1_ family, not the datasheet-derived 0x0_ one
    # (which also contained 0x00, a value indistinguishable from an
    # empty read). The strong identity anchor is the magnetometer
    # companion's WHO_AM_I (0x49), asserted in the probe prologue.
    WHO_AM_I_REG = 0x3B          # PCODE
    WHO_AM_I_VALUES = [0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E]

    # ── Companion die = magnetometer (fixed 0x0C) ───────
    I2C_COMPANIONS = {
        'mag': {'addr': 0x0C,
                'who_am_i_reg': 0x0F,    # WIA = 0x49 (fixed identity)
                'who_am_i_values': [0x49]},
    }

    # ── Accelerometer register map ──────────────────────
    # (measure() reads SR 0x03 and XOUT_EX..ZOUT_EX 0x0D..0x12 as
    #  literals, as the AST compiler requires.)
    ACC_INTEN   = 0x06           # interrupt enable (ACQ_INT_EN bit 7)
    ACC_MODE    = 0x07           # OPCON / INTA polarity+drive
    ACC_SRTFR   = 0x08           # sample-rate (ODR) + tap features
    ACC_OUTCFG  = 0x20           # range + resolution

    ACC_MODE_STANDBY = 0x00
    # WAKE (OPCON=01) | IPP push-pull (0x40) | IAH active-high (0x80)
    ACC_MODE_WAKE    = 0xC1
    ACC_ACQ_INT_EN   = 0x80      # INTEN: pulse INTA after each sample

    # SRTFR RATE[3:0]: output data rate code (datasheet Table 21).
    # The sub-hertz rows (0.25 Hz -> 0x07, 0.5 Hz -> 0x06) are excluded:
    # enum values are integers on the descriptor wire.
    ACCEL_RATE = {
        1: 0x05, 2: 0x04, 4: 0x03, 8: 0x02,
        16: 0x01, 32: 0x00, 64: 0x08, 128: 0x09, 256: 0x0A,
    }
    # OUTCFG RANGE[6:4]: full-scale range (datasheet Table 25).
    ACCEL_FS = {2: 0x00, 4: 0x10, 8: 0x20, 16: 0x30}     # g
    # OUTCFG RES[2:0]: output resolution (datasheet Table 25).
    ACCEL_RES_BITS = {6: 0x00, 7: 0x01, 8: 0x02, 10: 0x03,
                      12: 0x04, 14: 0x05}
    # Base scale = SI per LSB, per unit of range. A res-bit signed
    # reading spans +/- 2^(res-1) counts across the +/- range, so
    # scale = (g_range / 2^(res-1)) * 9.80665 = base * g_range, where
    # base = 9.80665 / 2^(res-1). accel_fs (the g range) is the live
    # scale_param, so the m/s^2 output tracks a runtime range change.
    ACCEL_BASE_SCALE = {
        6:  9.80665 / 32.0,      # 2^5
        7:  9.80665 / 64.0,      # 2^6
        8:  9.80665 / 128.0,     # 2^7
        10: 9.80665 / 512.0,     # 2^9
        12: 9.80665 / 2048.0,    # 2^11
        14: 9.80665 / 8192.0,    # 2^13  (default)
    }

    # ── Magnetometer register map (companion, dev='mag') ─
    # (measure() reads OUTX..OUTZ 0x10..0x15 as literals.)
    MAG_CTRL1   = 0x1B           # PC | ODR | FS
    MAG_CTRL3   = 0x1D           # SRST soft reset
    MAG_CTRL4   = 0x1E           # MMD | RS (resolution)

    MAG_SRST    = 0x80           # CTRL3: soft reset (reload compensation)
    MAG_CTRL1_PC = 0x80          # CTRL1: active mode (FS=0 -> Normal/continuous)
    MAG_CTRL4_MMD = 0x80         # CTRL4: MMD=10, the mandated default

    # CTRL1 ODR[4:3]: Normal-state output data rate (datasheet 9, CTRL1).
    # The 0.5 Hz row (0x00) is excluded: enum values are integers on the
    # descriptor wire.
    MAG_ODR = {10: 0x08, 20: 0x10, 100: 0x18}   # Hz
    # CTRL4 RS (bit 4): dynamic range / resolution.
    #   14-bit: -8192..8191 (+/-1.23 mT)   15-bit: -16384..16383 (+/-2.46 mT)
    MAG_RES_BITS = {14: 0x00, 15: 0x10}
    # Sensitivity 0.15 uT/LSB (datasheet Table 4) -> tesla. Negated to
    # align the mag axes with the accel body frame (datasheet Fig 3).
    MAG_SCALE = -0.15e-6         # tesla / LSB

    def __init__(self):
        super().__init__()
        self._read_responses = {
            self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]],   # accel PCODE
            ('mag', 0x0F): [0x49],                          # mag WHO_AM_I
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who in self.WHO_AM_I_VALUES

    def configure(self, config):
        # 1. Runtime params.
        self.declare_param(
            "sample_rate",
            values=[1, 2, 4, 8, 16, 32, 64, 128, 256],
            default=64, unit="Hz")
        self.declare_param("accel_fs", values=[2, 4, 8, 16],
                           default=8, unit="g")
        self.declare_param("mag_odr", values=[10, 20, 100],
                           default=100, unit="Hz")
        self.declare_param("mag_res", values=[14, 15],
                           default=15, unit="bit")

        # 2. Read config (accel_res is a compile-time key: it sets the
        #    output scale, so it cannot ride the runtime accel_fs
        #    scale_param — it is baked per upload).
        sample_rate = config.get('sample_rate', 64)
        accel_fs    = config.get('accel_fs', 8)
        accel_res   = config.get('accel_res', 14)
        mag_odr     = config.get('mag_odr', 100)
        mag_res     = config.get('mag_res', 15)

        # 3. Accelerometer (primary die). Config registers are writable
        #    only in STANDBY; only MODE is writable in WAKE.
        self.write(self.ACC_MODE, self.ACC_MODE_STANDBY)   # ensure standby
        self.write(self.ACC_SRTFR, self.ACCEL_RATE[sample_rate],
                       param=("sample_rate", sample_rate))
        # OUTCFG carries range (runtime accel_fs) | resolution (fixed
        # per-upload accel_res). accel_fs is the only param on it.
        self.write(self.ACC_OUTCFG,
                       self.ACCEL_FS[accel_fs] | self.ACCEL_RES_BITS[accel_res],
                       param=("accel_fs", accel_fs))
        self.write(self.ACC_INTEN, self.ACC_ACQ_INT_EN)    # enable DRDY on INTA

        # 4. Magnetometer (companion die). Soft-reset reloads defaults +
        #    factory compensation (the platform does not hard-reset this
        #    part — RST is not routed and it has no reset pin), then
        #    configure Normal continuous mode.
        self.write(self.MAG_CTRL3, self.MAG_SRST, dev='mag')
        self.sleep_ms(10)                                  # >= 2x POR ton (3 ms), floor 10 ms
        self.write(self.MAG_CTRL4,
                       self.MAG_CTRL4_MMD | self.MAG_RES_BITS[mag_res],
                       dev='mag', param=("mag_res", mag_res))
        # CTRL1: PC=1 (active), FS=0 (Normal/continuous), ODR = mag_odr.
        self.write(self.MAG_CTRL1,
                       self.MAG_CTRL1_PC | self.MAG_ODR[mag_odr],
                       dev='mag', param=("mag_odr", mag_odr))

        # 5. Accelerometer -> WAKE last, so INTA starts pulsing once
        #    both dies are configured.
        self.write(self.ACC_MODE, self.ACC_MODE_WAKE)

        # 6. Fused output: accel (bytes 0..5) + mag (bytes 6..11), each
        #    int16 little-endian. Accel scale tracks the live accel_fs;
        #    mag scale is fixed. SI units inherited from the field names.
        base = self.ACCEL_BASE_SCALE[accel_res]
        self.set_output([
            {'name': 'accel_x', 'type': 'int16', 'byte_order': 'little',
             'scale': base, 'scale_param': 'accel_fs'},
            {'name': 'accel_y', 'type': 'int16', 'byte_order': 'little',
             'scale': base, 'scale_param': 'accel_fs'},
            {'name': 'accel_z', 'type': 'int16', 'byte_order': 'little',
             'scale': base, 'scale_param': 'accel_fs'},
            {'name': 'mag_x', 'type': 'int16', 'byte_order': 'little',
             'scale': self.MAG_SCALE},
            {'name': 'mag_y', 'type': 'int16', 'byte_order': 'little',
             'scale': self.MAG_SCALE},
            {'name': 'mag_z', 'type': 'int16', 'byte_order': 'little',
             'scale': self.MAG_SCALE},
        ])
        self.set_sample_size(12)   # 6 x int16

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x03)               # SR: read clears + re-arms ACQ_INT
        if not (status & 0x80):                # ACQ_INT — new accel sample?
            return None
        raw = self.read_burst(0x0D, 6, into=0)          # accel XOUT..ZOUT -> 0..5
        self.read_burst(0x10, 6, into=6, dev='mag')     # mag OUTX..OUTZ -> 6..11
        return Sample(raw)
