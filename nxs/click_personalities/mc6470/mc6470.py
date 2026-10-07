"""
MC6470 personality for NXS VM.

mCube MC6470 6-axis eCompass (accel die + magnetometer die); datasheet APS-048-0033v1.7.
Bus: I2C only, 400 kHz; accel at 0x4C/0x6C (A5 strap), magnetometer companion fixed at 0x0C.
Config: accel_res 6|7|8|10|12|14 bit (14), compile-time; trigger drdy|poll (drdy); bus i2c.
Params: sample_rate 1|2|4|8|16|32|64|128|256 Hz (64); accel_fs 2|4|8|16 g (8);
    mag_odr 10|20|100 Hz (100); mag_res 14|15 bit (15).
Outputs: accel_x/y/z (m/s^2), mag_x/y/z (T), int16 little-endian, one body frame.
Pins: INT -> INTA (accelerometer data ready); AN -> INTM (unused); RST not connected.
mikroE Click: https://www.mikroe.com/6dof-imu-13-click
"""

from nxs import RegisterClickPersonality, Sample


class Mc6470(RegisterClickPersonality):
    # ── Bus ─────────────────────────────────────────────
    # I2C-only part; no SPI or UART variant.
    BUSES = ('i2c',)

    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'drdy': 'mkbus_int',   # accelerometer INTA / ACQ_INT (input)
    }

    # ── Primary die = accelerometer ─────────────────────
    # A5 strap selects the accel address LSB (GND 0x4C, VDD 0x6C); both are
    # scanned on load. The magnetometer companion is fixed at 0x0C.
    I2C_ADDRS = [0x4C, 0x6C]

    # PCODE bits [3:1] are factory-variable ("ignore" per the datasheet); silicon
    # reads bit 4 set, so the set is 0x10..0x1E. The mag WIA (0x49) is the hard anchor.
    WHO_AM_I_REG = 0x3B          # PCODE
    WHO_AM_I_VALUES = [0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E]

    # ── Companion die = magnetometer (fixed 0x0C) ───────
    I2C_COMPANIONS = {
        'mag': {'addr': 0x0C,
                'who_am_i_reg': 0x0F,    # WIA = 0x49 (fixed identity)
                'who_am_i_values': [0x49]},
    }

    # ── Accelerometer register map ──────────────────────
    # measure() reads SR (0x03) and XOUT_EX..ZOUT_EX (0x0D..0x12) as literals.
    ACC_INTEN   = 0x06           # interrupt enable (ACQ_INT_EN bit 7)
    ACC_MODE    = 0x07           # OPCON / INTA polarity+drive
    ACC_SRTFR   = 0x08           # sample-rate (ODR) + tap features
    ACC_OUTCFG  = 0x20           # range + resolution

    ACC_MODE_STANDBY = 0x00
    # WAKE (OPCON=01) | IPP push-pull (0x40) | IAH active-high (0x80)
    ACC_MODE_WAKE    = 0xC1
    ACC_ACQ_INT_EN   = 0x80      # INTEN: pulse INTA after each sample

    # SRTFR RATE[3:0] (datasheet Table 21). The sub-hertz rows (0.25 Hz 0x07,
    # 0.5 Hz 0x06) are dropped: enum values are integers.
    ACCEL_RATE = {
        1: 0x05, 2: 0x04, 4: 0x03, 8: 0x02,
        16: 0x01, 32: 0x00, 64: 0x08, 128: 0x09, 256: 0x0A,
    }
    # OUTCFG RANGE[6:4]: full-scale range (datasheet Table 25).
    ACCEL_FS = {2: 0x00, 4: 0x10, 8: 0x20, 16: 0x30}     # g
    # OUTCFG RES[2:0]: output resolution (datasheet Table 25).
    ACCEL_RES_BITS = {6: 0x00, 7: 0x01, 8: 0x02, 10: 0x03,
                      12: 0x04, 14: 0x05}
    # A res-bit signed reading spans +/- 2^(res-1) counts across +/- the range:
    # base = 9.80665 / 2^(res-1) per g, times the live accel_fs via scale_param.
    ACCEL_BASE_SCALE = {
        6:  9.80665 / 32.0,      # 2^5
        7:  9.80665 / 64.0,      # 2^6
        8:  9.80665 / 128.0,     # 2^7
        10: 9.80665 / 512.0,     # 2^9
        12: 9.80665 / 2048.0,    # 2^11
        14: 9.80665 / 8192.0,    # 2^13  (default)
    }

    # ── Magnetometer register map (companion, dev='mag') ─
    # measure() reads OUTX..OUTZ (0x10..0x15) as literals.
    MAG_CTRL1   = 0x1B           # PC | ODR | FS
    MAG_CTRL3   = 0x1D           # SRST soft reset
    MAG_CTRL4   = 0x1E           # MMD | RS (resolution)

    MAG_SRST    = 0x80           # CTRL3: soft reset (reload compensation)
    MAG_CTRL1_PC = 0x80          # CTRL1: active mode (FS=0 -> Normal/continuous)
    MAG_CTRL4_MMD = 0x80         # CTRL4: MMD=10, the mandated default

    # CTRL1 ODR[4:3], Normal-state rate (datasheet 9, CTRL1). The 0.5 Hz row
    # (0x00) is dropped: enum values are integers.
    MAG_ODR = {10: 0x08, 20: 0x10, 100: 0x18}   # Hz
    # CTRL4 RS (bit 4): dynamic range / resolution.
    #   14-bit: -8192..8191 (+/-1.23 mT)   15-bit: -16384..16383 (+/-2.46 mT)
    MAG_RES_BITS = {14: 0x00, 15: 0x10}
    # 0.15 uT/LSB (datasheet Table 4) in tesla, negated: the mag axes are
    # anti-parallel to the accel axes (datasheet Fig 3, vendor coefficient -1.0).
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
        self.declare_params_from_descriptor()

        # 2. accel_res is compile-time: it sets the output scale, which
        #    already rides the runtime accel_fs.
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
        # OUTCFG: range (runtime accel_fs) | resolution (compile-time accel_res).
        self.write(self.ACC_OUTCFG,
                       self.ACCEL_FS[accel_fs] | self.ACCEL_RES_BITS[accel_res],
                       param=("accel_fs", accel_fs))
        self.write(self.ACC_INTEN, self.ACC_ACQ_INT_EN)    # enable DRDY on INTA

        # 4. Magnetometer: CTRL3 SRST reloads the defaults and factory
        #    compensation (no reset pin), then Normal continuous mode.
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

        # 6. Outputs: accel bytes 0..5, mag bytes 6..11, int16 little-endian;
        #    the accel scale tracks the live accel_fs.

        # Both dies sign-extend to 16 bits (datasheet p.54 accel; p.38 mag,
        # 0xE000 = -8192), so no masking per resolution.
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
        # Not published: the mag die temperature (TEMP, 1 C/LSB) updates only
        # from a Force-State TCS measurement, not in Normal continuous mode.

    @RegisterClickPersonality.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x03)               # SR: read clears + re-arms ACQ_INT
        if not (status & 0x80):                # ACQ_INT: new accel sample
            return None
        raw = self.read_burst(0x0D, 6, into=0)          # accel XOUT..ZOUT -> 0..5
        # The mag bytes hold the last magnetometer update between mag_odr ticks.
        self.read_burst(0x10, 6, into=6, dev='mag')     # mag OUTX..OUTZ -> 6..11
        return Sample(raw)
