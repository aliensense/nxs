"""
IIM-20670 driver for NXS VM.

TDK InvenSense IIM-20670 SmartIndustrial 6-axis IMU: 3-axis gyroscope +
3-axis accelerometer + die temperature. SPI-only, via a 32-bit command
word carrying an 8-bit CRC and a one-frame pipelined read (the response
to request K is clocked out during frame K+1).

Data path: direct register reads, no FIFO (the part has none, DS Sec
4.14). The datapath runs at a fixed 8 kHz internal rate with no ODR
divider. Pin 12 (ODR) emits the 8 kHz sync that anchors each read burst
(DS Sec 4.11, Fig 6) -- the data-ready path and the default trigger. In
poll mode the host sets an asynchronous read cadence instead. One
7-register burst (gyro XYZ, temp, accel XYZ) is 8 pipelined frames with
a 250 us response-staging gap between frames (one 8 kHz internal update
with 2x margin -- bench-swept, see FRAME), so a burst lands at ~3 ms
including the VM's per-chunk pacing overhead: the burst-side ceiling is
~250 Hz (the top declared rate), and the fields within one
sample span a handful of successive internal updates (sub-ms register-
to-register skew). The capture_mode output freeze (DS Sec 6.13, left at
its factory default -- see "Not published") is the part's documented
mechanism for a fully coherent snapshot if a use case ever needs one.

Integrity: the FRAME declares the protocol's CRC and its return-status
field (status_ok, RS == 01 per DS Table 13), so the compiler verifies
every harvested measure response on-device — CRC recomputed over the
received word and compared, status mask-checked. A corrupted or
unprepared frame (RS = 10, the staged-response miss) drops that tick
instead of publishing; sustained corruption starves the sample
watchdog, which escalates as a loud VM error.

Register banks: data + control live in bank 0 (default); accel full-scale
is bank 6, gyro full-scale bank 7, and the ODR-sync routing bank 3 -- all
gated behind a fixed tcode unlock walk (DS Sec 6.13). Every config register
mixes the setting with undocumented reserved bits, so writes are read-
modify-write (write_modify). The full-scale and filter fields are runtime
params anyway: each write_modify is tagged with param=, so its OR set-bits
immediate is a bytecode patch site -- `nxs set` rewrites the field code and
reloads (re-running the RMW, reserved bits read fresh), and the SI scale
tracks the live range via scale_param. filter_hz writes two registers, so
it owns two patch sites.

Reset: the mikroBUS RST line is wired to the active-low RESETN pin (mikroE
6DOF IMU 23 Click), so the platform hard-resets the part at every bind and
configure() issues no software reset (RESETN active-low = default polarity).

mikroBUS pin mapping (from mikroE 6DOF IMU 23 Click C driver, MIKROE-5999):
    INT (PA9)  -> ODR:    8 kHz data-ready sync (input)
    RST (PB2)  -> RESETN: reset/enable (output, active low, platform-driven)
    SCK/MISO/MOSI/CS      -> SPI bus (4-wire, mode 0, <=10 MHz)
    AN, PWM: not connected

Config keys (compile-time, `nxs upload ... --config key=value`):
    bus         spi                                    (SPI-only silicon)
    trigger     drdy (default) | poll                  (drdy = 8 kHz ODR sync)

Runtime params (`nxs set key value`, reload):
    accel_fs    2 | 4 | 16 (default) | 32              g   (DS Table 17; the
                labels' true ranges are 1.024x -- scale stays exact)
    gyro_fs     41|61|82|123|164|218|246|328|437|492|655 (default)|874|1311|1966
                                                        dps (DS Table 18; scale
                within ~0.7% of exact, inside the sensitivity tolerance)
    filter_hz   10 | 46 | 60 (default)                 Hz  (joint gyro+accel LPF)
    sample_rate 10|25|50|100 (default)|200|250       Hz  (exact 8000/N; drdy
                divides the ODR sync at the source -- bench-verified exact
                tick multiples; poll patches the loop interval)

Not published (documented but outside the streaming-vector model):
  - temp2 / temp12_delta (DS Sec 6.4, 6.9): redundant check copies of temp1,
    which is "used by all temperature calculations" (DS Sec 4.9).
  - Low-resolution accel outputs (DS Sec 6.5): a coarser copy of the same
    acceleration (larger low-res full scale); the high-res output is published.
  - Accel / gyro self-test (DS Sec 5.3, 5.4, 6.10): a production/BIST mode,
    not a streamed measurand.
  - register_write_lock / capture_mode (DS Sec 6.13): write-protection and
    output-freeze features; left at their factory defaults.

Datasheet: TDK InvenSense DS-000183, IIM-20670 Rev 1.0
    (https://invensense.tdk.com/download-pdf/iim-20670-datasheet/)
mikroE Click: https://www.mikroe.com/6dof-imu-23-click (MIKROE-5999)
"""

from nxs import RegisterDriver, Sample, SpiProfile
from nxs.framing import Crc, SpiFrame


def _bits(value, pos, width):
    """(set_bits, clear_bits) that force the `width`-bit field at bit `pos`
    to `value` under write_modify -- set the field's 1s, clear its 0s. The
    two masks never overlap, satisfying write_modify's disjointness rule."""
    mask = (1 << width) - 1
    v = value & mask
    return (v << pos, (mask & ~v) << pos)


def _field(code, pos, width):
    """(set_bits, clear_bits) for a write_modify tagged with `param=`: set the
    field to `code`, clear the WHOLE field. Unlike _bits (which clears only
    code's zero bits), clear_bits here is the constant field mask -- the same
    for every value -- so only the OR set_bits immediate varies, giving one
    stable patch site the runtime `set` can rewrite."""
    mask = (1 << width) - 1
    return ((code & mask) << pos, mask << pos)


class Iim20670(RegisterDriver):
    # -- Buses ------------------------------------------------------------
    # SPI-only silicon; there is no I2C variant (DS Sec 5.1).
    BUSES = ('spi',)

    # -- mikroBUS pin mapping ---------------------------------------------
    PINS = {
        'drdy': 'mkbus_int',   # ODR pin 12 -> firmware DRDY input (8 kHz sync)
    }

    # -- Device ID (DS Sec 6.6) -------------------------------------------
    # No conventional WHO_AM_I in the default bank (the whoami register lives
    # in bank 1). The FIXED_VALUE register (bank 0, 0x0B) always reads 0xAA55
    # and needs no bank switch, so it is the probe anchor. The FRAME prologue
    # compares the full 16-bit data field on-device.
    WHO_AM_I_REG = 0x0B
    WHO_AM_I_VALUES = [0xAA55]

    # -- Communication profile (DS Table 6 / Table 7) ---------------------
    # SPI SCLK ceiling 10 MHz, SPI mode 0. The FRAME below composes address
    # and data words itself, so the profile carries only clock + mode.
    SPI_PROFILE = SpiProfile(max_hz=10_000_000, mode=0)

    # -- SPI frame (DS Sec 5.2, Fig 8) ------------------------------------
    # 32-bit word: RW(1) | addr(5) | status(2) | data(16) | CRC(8), MSB-first.
    # CRC-8 poly 0x1D, init 0xFF, inverted out, over the first 24 bits.
    # feedback_style 'input-lsb' reproduces the datasheet's worked examples
    # (CRC(0xA0CA85)=0x2F, CRC(0x3B007C)=0xC2, CRC(0xD07A65)=0x88) and composes
    # the Sec 4.11 unlock words verbatim. Reads are pipelined: the response to
    # request K is clocked out during frame K+1 (read_pipeline=1). A sampled
    # data register stages its response on the part's next 8 kHz internal
    # update (125 us period) -- bench-swept: a 125 us inter-frame gap reads
    # clean, 63 us returns garbage (the clock-out frame beats the tick), and
    # back-to-back frames read stale/idle at any SPI clock. 250 us = one tick
    # with 2x margin. The datasheet's self-test tables show back-to-back
    # frames, but that is a different transaction; do not read them as license
    # to drop this settle. Always-ready constants (FIXED_VALUE/WHO_AM_I) do
    # clock out back-to-back, so the gap applies to the measure burst, not
    # the probe read.
    FRAME = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x1D, init=0xFF, xor_out=0xFF,
                covers=('rw', 'addr', 'rs', 'data'), feedback_style='input-lsb'),
        read_pipeline=1,
        inter_frame_sleep_us=250,
        status_ok=('rs', 0b01),   # DS Table 13: 01 = successful read/write
    )

    # -- ODR-sync bank unlock (DS Sec 4.11 / 6.13) ------------------------
    # Six fixed tcode words to the MODE register (0x19); they compose to the
    # datasheet's 0xE4.. words verbatim and unlock bank_select + non-bank-0
    # writes until the next power-on/hardware reset.
    UNLOCK_WORDS = (0x0002, 0x0001, 0x0004, 0x0300, 0x0180, 0x0280)

    # -- Accelerometer full-scale: g label -> accel_fs_sel[2:0] (bank 6, 0x14).
    # DS Table 17. The 8 codes collapse to 4 high-res ranges -- paired codes
    # differ only in the low-res full scale, which this driver does not publish
    # -- so one code is kept per high-res range (the 16 g label keeps the
    # factory-default code 001). The true range is exactly 1.024x the label
    # (2.048/4.096/16.384/32.768 g), a constant factor folded into ACCEL_BASE
    # so the label is the clean scale_param value and the SI scale stays exact.
    ACCEL_FS = {2: 0b100, 4: 0b110, 16: 0b001, 32: 0b010}

    # -- Gyroscope full-scale: dps label -> gyro_fs_sel[3:0] (bank 7, 0x14).
    # DS Table 18. 16 codes -> 14 distinct ranges (codes 1111 and 0111 duplicate
    # 0000 and 0010, so are dropped). 655 dps (code 0001) is the factory
    # default. The label is the scale_param value (scale = GYRO_BASE x label);
    # the labels are the datasheet's rounded ranges, so the runtime scale is
    # within ~0.7% of the exact per-range sensitivity (worst at 61 dps, whose
    # true range is 61.44) -- inside the part's sensitivity tolerance.
    GYRO_FS = {
          41: 0b1100,    61: 0b1000,   82: 0b1101,  123: 0b1001,
         164: 0b1110,   218: 0b0100,  246: 0b1010,  328: 0b0000,
         437: 0b0101,   492: 0b1011,  655: 0b0001,  874: 0b0110,
        1311: 0b0010,  1966: 0b0011,
    }

    # Signed 16-bit reading spans +/- the full-scale range, so rad/s (or m/s^2)
    # per LSB = (range x unit-conv) / 32768. scale_param multiplies these bases
    # by the live range label per sample. ACCEL_BASE folds the 1.024 range/label
    # factor so accel is exact; GYRO_BASE takes the label directly (see GYRO_FS).
    ACCEL_BASE = 1.024 * 9.80665 / 32768.0    # m/s^2 per LSB, per g label
    GYRO_BASE = (3.141592653589793 / 180.0) / 32768.0  # rad/s per LSB, per dps label

    # -- Digital low-pass cutoff -> 6-bit filter code (DS Tables 14-16). The
    # code is the same for all three axes (only the field position differs:
    # flt_y[5:0], flt_z[11:6] in 0x0C; flt_x[13:8] in 0x0E). Each code encodes
    # a joint (gyro cutoff, accel cutoff) pair; a single SI-clean filter knob
    # projects onto the diagonal where the two match -- 10/10, 46/46, 60/60 Hz.
    # 60 Hz is the register default and the widest matched (least filtering).
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

        # Runtime-tunable ranges + filter (reload params). Each tags its
        # read-modify-write below with param=, so `nxs set` rewrites the field
        # code and reloads; the SI scale tracks the live range via scale_param
        # (see set_output). Declared unconditionally so `caps` lists them.
        self.declare_param("accel_fs", values=[2, 4, 16, 32], default=16, unit="g")
        self.declare_param("gyro_fs",
                           values=[41, 61, 82, 123, 164, 218, 246, 328, 437,
                                   492, 655, 874, 1311, 1966],
                           default=655, unit="dps")
        self.declare_param("filter_hz", values=[10, 46, 60], default=60, unit="Hz")

        # sample_rate: the part has no ODR divider register (fixed 8 kHz
        # internal rate), so the knob is loop pacing. In drdy mode the
        # compiler divides the ODR sync at the source (OP_EVENT_DIV, see
        # measure_loop's drdy_base_hz): every value is an exact 8000/N, so
        # the delivered spacing is crystal-derived from the sensor. In poll
        # mode the same values patch the loop's SLEEP_MS interval.
        self.declare_param("sample_rate",
                           values=[10, 25, 50, 100, 200, 250],
                           default=100, unit="Hz")

        accel_fs = config.get('accel_fs', 16)
        gyro_fs = config.get('gyro_fs', 655)
        filter_hz = config.get('filter_hz', 60)

        # Compile-time table lookups: an unknown value raises KeyError and
        # fails the compile rather than silently falling back to a default.
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

        # 2. Accelerometer full-scale (bank 6, reg 0x14, accel_fs_sel[2:0]).
        #    Read-modify-write: the register's other bits are reserved factory
        #    state (DS Sec 6.17), so only the field bits are touched. _field
        #    clears the whole 3-bit field (constant) so the OR set is the one
        #    patch site accel_fs rewrites at runtime.
        self.write(0x1F, 0x0006)            # bank_select = 6 (fixed)
        a_set, a_clr = _field(accel_code, 0, 3)
        self.write_modify(0x14, set_bits=a_set, clear_bits=a_clr,
                          param=("accel_fs", accel_fs))

        # 3. Gyroscope full-scale (bank 7, reg 0x14, gyro_fs_sel[3:0]).
        self.write(0x1F, 0x0007)            # bank_select = 7 (fixed)
        g_set, g_clr = _field(gyro_code, 0, 4)
        self.write_modify(0x14, set_bits=g_set, clear_bits=g_clr,
                          param=("gyro_fs", gyro_fs))

        # 4. ODR-sync routing on pin 12 (bank 3) -- drdy only (DS Sec 4.11).
        #    All read-modify-write: every listed register's remaining bits are
        #    reserved. In poll mode the pin is unused, so the routing is skipped.
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

        # 6. Digital low-pass filter (bank 0): flt_z[11:6] + flt_y[5:0] @0x0C,
        #    flt_x[13:8] @0x0E. Same cutoff code on all three axes (read-modify-
        #    write; the registers' other bits are reserved, DS Sec 6.7/6.8).
        #    filter_hz writes two registers, so it owns two patch sites -- both
        #    tagged with the same param, each clearing its whole field.
        fy_s, fy_c = _field(fcode, 0, 6)
        fz_s, fz_c = _field(fcode, 6, 6)
        self.write_modify(0x0C, set_bits=fy_s | fz_s, clear_bits=fy_c | fz_c,
                          param=("filter_hz", filter_hz))
        fx_s, fx_c = _field(fcode, 8, 6)
        self.write_modify(0x0E, set_bits=fx_s, clear_bits=fx_c,
                          param=("filter_hz", filter_hz))

        # 7. Output vector -- registers 0x00-0x06 in address order: gyro XYZ
        #    (rad/s), temp1 (K), accel XYZ (m/s^2). Each int16 big-endian 2's-
        #    complement; fields pack sequentially onto the 14-byte sample.
        #    accel/gyro scales track the live range via scale_param (base x the
        #    range label); SI-semantic fields inherit their canonical unit.
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

    @RegisterDriver.measure_loop(trigger="from_config", sample_rate=100,
                                 drdy_base_hz=8000)
    def measure(self):
        self.sleep_us(5)                 # DS Fig 6: wait 5 us after the ODR sync edge
        raw = self.read_words(0x00, 7)   # gyro XYZ, temp1, accel XYZ (regs 0x00-0x06)
        return Sample(raw)
