"""
IIM-20670 personality for NXS VM.

TDK InvenSense IIM-20670 6-axis IMU (gyro, accel, die temperature); DS-000183 Rev 1.0,
    https://invensense.tdk.com/download-pdf/iim-20670-datasheet/
Bus: SPI only (mikroBUS CS), mode 0, 10 MHz max; 32-bit CRC-8 words, reads pipelined one frame.
Config: bus spi; trigger drdy|poll (drdy). Params: accel_fs 2|4|16|32 g (16);
    gyro_fs 41|61|82|123|164|218|246|328|437|492|655|874|1311|1966 dps (655);
    filter_hz 10|46|60 Hz (60); sample_rate 10|25|50|100|200|250 Hz (100), exact 8000/N.
Outputs: gyro_x/y/z (rad/s), temp (K), accel_x/y/z (m/s^2).
Pins: INT -> ODR (8 kHz data-ready sync); RST -> RESETN (active low, platform-driven).
mikroE Click: https://www.mikroe.com/6dof-imu-23-click (MIKROE-5999)
"""

from nxs import RegisterClickPersonality, Sample, SpiProfile
from nxs.framing import Crc, SpiFrame


def _bits(value, pos, width):
    """(set_bits, clear_bits) forcing the `width`-bit field at `pos` to
    `value` under write_modify: set its 1s, clear its 0s (never overlapping)."""
    mask = (1 << width) - 1
    v = value & mask
    return (v << pos, (mask & ~v) << pos)


def _field(code, pos, width):
    """(set_bits, clear_bits) for a param-tagged write_modify: set the field to
    `code`, clear the whole field, so only the OR immediate varies (one patch site)."""
    mask = (1 << width) - 1
    return ((code & mask) << pos, mask << pos)


class Iim20670(RegisterClickPersonality):
    # -- Buses ------------------------------------------------------------
    # SPI-only silicon; there is no I2C variant (DS Sec 5.1).
    BUSES = ('spi',)

    # -- mikroBUS pin mapping ---------------------------------------------
    PINS = {
        'drdy': 'mkbus_int',   # ODR pin 12 -> firmware DRDY input (8 kHz sync)
    }

    # -- Device ID (DS Sec 6.6) -------------------------------------------
    # FIXED_VALUE (bank 0, 0x0B) always reads 0xAA55 and needs no bank switch
    # (whoami lives in bank 1); the FRAME prologue compares the full 16 bits.
    WHO_AM_I_REG = 0x0B
    WHO_AM_I_VALUES = [0xAA55]

    # -- Communication profile (DS Table 6 / Table 7) ---------------------
    # SPI SCLK ceiling 10 MHz, SPI mode 0. The FRAME below composes address
    # and data words itself, so the profile carries only clock + mode.
    SPI_PROFILE = SpiProfile(max_hz=10_000_000, mode=0)

    # -- SPI frame (DS Sec 5.2, Fig 8): RW(1) | addr(5) | status(2) | data(16) | CRC(8)
    # CRC-8 poly 0x1D, init 0xFF, inverted out, over the first 24 bits; 'input-lsb'
    # reproduces the DS examples 0xA0CA85 -> 0x2F, 0x3B007C -> 0xC2, 0xD07A65 -> 0x88.
    FRAME = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('rw', 'addr', 'rs', 'data'), feedback_style='input-lsb'),
        read_pipeline=1,          # response to request K clocks out in frame K+1
        # One 8 kHz internal update (125 us) with 2x margin; the self-test
        # tables' back-to-back frames are a different transaction.
        inter_frame_sleep_us=250,
        status_ok=('rs', 0b01),   # DS Table 13: 01 = successful read/write
    )

    # -- ODR-sync bank unlock (DS Sec 4.11 / 6.13) ------------------------
    # Six tcode words to MODE (0x19) compose to the datasheet's 0xE4.. words
    # and open bank_select and non-bank-0 writes until the next reset.
    UNLOCK_WORDS = (0x0002, 0x0001, 0x0004, 0x0300, 0x0180, 0x0280)

    # -- Accelerometer full-scale: accel_fs_sel[2:0] (bank 6, 0x14), DS Table 17.
    # One code per high-res range (paired codes differ only in the unpublished
    # low-res range; 16 g keeps the default 001). True range = 1.024 x label.
    ACCEL_FS = {2: 0b100, 4: 0b110, 16: 0b001, 32: 0b010}

    # -- Gyroscope full-scale: gyro_fs_sel[3:0] (bank 7, 0x14), DS Table 18.
    # Codes 1111 and 0111 duplicate 0000 and 0010 and are dropped; 655 dps is the
    # default. Labels are rounded ranges: scale within ~0.7% (61 dps, true 61.44).
    GYRO_FS = {
          41: 0b1100,    61: 0b1000,   82: 0b1101,  123: 0b1001,
         164: 0b1110,   218: 0b0100,  246: 0b1010,  328: 0b0000,
         437: 0b0101,   492: 0b1011,  655: 0b0001,  874: 0b0110,
        1311: 0b0010,  1966: 0b0011,
    }

    # A signed 16-bit reading spans +/- the range: base = range unit / 32768 per
    # LSB, times the live label via scale_param. ACCEL_BASE folds the 1.024 factor.
    ACCEL_BASE = 1.024 * 9.80665 / 32768.0    # m/s^2 per LSB, per g label
    GYRO_BASE = (3.141592653589793 / 180.0) / 32768.0  # rad/s per LSB, per dps label

    # -- Low-pass cutoff -> 6-bit code (DS Tables 14-16), the same code per axis:
    # flt_y[5:0], flt_z[11:6] at 0x0C; flt_x[13:8] at 0x0E. A code pairs a gyro
    # and an accel cutoff; the matched pairs are 10/46/60 Hz (60 = register default).
    FILTER_CODES = {10: 0b000001, 46: 0b011101, 60: 0b100000}

    def __init__(self):
        super().__init__()
        # Trace-time seed so probe()'s FIXED_VALUE assert clears at compile time.
        self._read_responses = {
            self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)      # FIXED_VALUE (bank 0, 0x0B)
        assert who == 0xAA55, "IIM-20670 FIXED_VALUE mismatch"

    def configure(self, config):
        trigger = config.get('trigger', 'drdy')

        # Reload params; each tags a write_modify below, and the SI scale
        # tracks the live range via scale_param.
        self.declare_params_from_descriptor()

        # No ODR divider (fixed 8 kHz): in drdy mode sample_rate divides the ODR
        # sync at the source (drdy_base_hz); in poll mode it patches the interval.

        accel_fs = config.get('accel_fs', 16)
        gyro_fs = config.get('gyro_fs', 655)
        filter_hz = config.get('filter_hz', 60)

        # An unknown value raises KeyError at compile time.
        accel_code = self.ACCEL_FS[accel_fs]
        gyro_code = self.GYRO_FS[gyro_fs]
        fcode = self.FILTER_CODES[filter_hz]

        # 1. Unlock bank_select + non-bank-0 writes (fixed tcode words to MODE
        #    reg 0x19). Composes to the DS Sec 4.11 words 0xE4000288 ... 0xE4028030.
        self.write(0x19, 0x0002)            # tcode 010 (unlock step 1)
        self.write(0x19, 0x0001)            # tcode 001 (unlock step 2)
        self.write(0x19, 0x0004)            # tcode 100 (unlock step 3)
        self.write(0x19, 0x0300)            # DS Sec 4.11 unlock word 4
        self.write(0x19, 0x0180)            # DS Sec 4.11 unlock word 5
        self.write(0x19, 0x0280)            # DS Sec 4.11 unlock word 6

        # 2. Accel full-scale (bank 6, 0x14, accel_fs_sel[2:0]): read-modify-write
        #    over reserved factory bits (DS Sec 6.17); _field keeps one patch site.
        self.write(0x1F, 0x0006)            # bank_select = 6 (fixed)
        a_set, a_clr = _field(accel_code, 0, 3)
        self.write_modify(0x14, set_bits=a_set, clear_bits=a_clr,
                          param=("accel_fs", accel_fs))

        # 3. Gyroscope full-scale (bank 7, reg 0x14, gyro_fs_sel[3:0]).
        self.write(0x1F, 0x0007)            # bank_select = 7 (fixed)
        g_set, g_clr = _field(gyro_code, 0, 4)
        self.write_modify(0x14, set_bits=g_set, clear_bits=g_clr,
                          param=("gyro_fs", gyro_fs))

        # 4. ODR sync routing on pin 12 (bank 3, DS Sec 4.11), read-modify-write
        #    over reserved bits; skipped in poll mode, where the pin is unused.
        if trigger != 'poll':
            self.write(0x1F, 0x0003)        # bank_select = 3 (fixed)
            self.write_modify(0x14, set_bits=0x0200)            # ODR_Config_4: 0x14 bit 9
            self.write_modify(0x17, set_bits=0x1000)            # ODR_Config_6: 0x17 bit 12
            s, c = _bits(0x21, 8, 6)
            self.write_modify(0x11, set_bits=s, clear_bits=c)   # ODR_Config_1: 0x11[13:8]=0x21
            s, c = _bits(0x08, 4, 4)
            self.write_modify(0x13, set_bits=s, clear_bits=c)   # ODR_Config_2: 0x13[7:4]=0x08
            self.write_modify(0x14, set_bits=0x0020)            # ODR_Config_3: 0x14 bit 5
            self.write_modify(0x16, set_bits=0x0001)            # ODR_Config_5: 0x16 bit 0

        # 5. Return to bank 0 for data reads and filter config.
        self.write(0x1F, 0x0000)            # bank_select = 0 (fixed)

        # 6. Low-pass filter (bank 0): flt_z[11:6] | flt_y[5:0] at 0x0C, flt_x[13:8]
        #    at 0x0E (DS Sec 6.7/6.8). filter_hz owns both writes: two patch sites.
        fy_s, fy_c = _field(fcode, 0, 6)
        fz_s, fz_c = _field(fcode, 6, 6)
        self.write_modify(0x0C, set_bits=fy_s | fz_s, clear_bits=fy_c | fz_c,
                          param=("filter_hz", filter_hz))
        fx_s, fx_c = _field(fcode, 8, 6)
        self.write_modify(0x0E, set_bits=fx_s, clear_bits=fx_c,
                          param=("filter_hz", filter_hz))

        # 7. Outputs: registers 0x00-0x06 in address order, int16 big-endian;
        #    accel and gyro scales track the live range via scale_param.

        # Not published: temp2/temp12_delta (DS Sec 6.4, 6.9, check copies of
        # temp1) and the low-resolution accel outputs (DS Sec 6.5).
        self.set_output([
            {'name': 'gyro_x', 'scale': self.GYRO_BASE, 'scale_param': 'gyro_fs'},
            {'name': 'gyro_y', 'scale': self.GYRO_BASE, 'scale_param': 'gyro_fs'},
            {'name': 'gyro_z', 'scale': self.GYRO_BASE, 'scale_param': 'gyro_fs'},
            {'name': 'temp', 'scale': 0.05, 'offset': 298.15},  # 25 C zero -> K (DS Sec 4.9)
            {'name': 'accel_x', 'scale': self.ACCEL_BASE, 'scale_param': 'accel_fs'},
            {'name': 'accel_y', 'scale': self.ACCEL_BASE, 'scale_param': 'accel_fs'},
            {'name': 'accel_z', 'scale': self.ACCEL_BASE, 'scale_param': 'accel_fs'},
        ])
        self.set_sample_size(14)            # 7 x int16

    @RegisterClickPersonality.measure_loop(trigger="from_config", sample_rate=100,
                                 drdy_base_hz=8000)
    def measure(self):
        self.sleep_us(5)                 # DS Fig 6: wait 5 us after the ODR sync edge
        raw = self.read_words(0x00, 7)   # gyro XYZ, temp1, accel XYZ (regs 0x00-0x06)
        return Sample(raw)
