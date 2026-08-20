---
name: nxs-generate-sensor-driver
description: >
  Generate a new NXS sensor driver from a datasheet or mikroE Click board.
  Use when a user plugs in a new sensor, provides a datasheet PDF/URL,
  or names a sensor IC. Creates a complete Python SensorDriver subclass
  with probe(), configure(), and @measure_loop methods.
allowed-tools: Read WebFetch WebSearch Write Bash Glob Grep
argument-hint: [sensor-name-or-click-board-url]
---

# Generate NXS Sensor Driver

You are creating a sensor driver for the NXS sensor VM. The driver is
a "datasheet in code" — a Python class that describes every register,
mode, and scale factor of the sensor IC. The YAML config then selects
which mode to use at deployment time.

**Plug-and-play — resolve, don't ask.** Every choice is a datasheet fact
(registers, modes, scales, WHO_AM_I, bus protocol, clock ceilings) or a fixed
NXS-board fact (mikroBUS I²C 400 kHz; SPI 1 MHz baseline, no DTS ceiling;
RST/INT/AN/PWM on the mikroBUS socket). Resolve them and emit a complete
driver: declare every bus the silicon supports plus the datasheet clocks,
default `BUS` to the pre-soldered transport, and leave each a knob (`BUSES`,
`BUS`, profiles, params). Ask only when a *required* fact is absent from the
datasheet or Click materials — never to pick between a default and its
alternative.

**Not expressible — stop, don't probe.** The documented surface (this
skill + the authors guide) is the whole contract. If the part's wire
behavior needs a construct neither documents — or a compile attempt
answers `not yet supported` — the part is not expressible today: STOP
immediately. Do not reverse-engineer the compiler by experiment (no
field-to-wire test matrices, no probe modules, no reading compiler
source), and do not ship a workaround that leans on undocumented
behavior — behavior that happens to work is not a contract and breaks
without notice. Output the NOT EXPRESSIBLE card (Step 7) naming the gap
and end the run there.

The same stop applies to a **gating fact you cannot read**: when the
datasheet section that decides a required behavior (a data-ready
enable procedure, a power-on default) fails to extract or is missing
from your copy, name the section in the card and stop — a driver built
on an assumed default ships a lie that only surfaces on hardware. A
part whose datasheet cannot be retrieved **at all** is not generatable:
the Click sources are wire-protocol-normative, never a capability
source, so stop with the card instead of shipping a driver with
silently reduced capabilities (dropped fields, guessed clock ceilings,
omitted parameters).

## Step 1: Read the Reference Materials

**ALWAYS start here.** Read the driver development guide to understand
the exact APIs, patterns, and constraints — whichever of these exists in
your checkout:

```
docs/specs/nxs-driver-development.md    # inside the firmware repository
docs/nxs-driver-development.md          # standalone SDK checkout / release bundle
```

That guide is the single source of truth for what a driver can do
and how `RegisterDriver` / `StreamDriver` behave. The templates and
rules in this skill summarise it, but when the two disagree, the
guide wins.

**You will not see the compiler source** — that's deliberate. Write
against the documented surface, not against any imagined internal
behaviour. You do run the compiler as a black box: Step 6 compiles
the driver per declared bus, and a `CompileError` is the feedback
loop.

## Step 2: Identify the Sensor and Check Compatibility

From the user's input ($ARGUMENTS), identify:
- The sensor IC (part number from the datasheet or Click silkscreen)
- Communication protocol: **I2C, SPI, or UART**
- If a mikroE Click board URL is given, fetch it to identify the IC

**3.3V only.** NXS provides 3.3V on the mikroBUS power rail.

**Classify the part's measurement classes and read the matching
reference files.** The class decides conventions the wire protocol
doesn't: canonical parameter names, the expected output vector,
reference frames and datums, and the class self-checks. The files
live under `references/` in this skill's directory; a combo
part covers several classes and reads every matching file:

| Class | Read |
|---|---|
| Accelerometer / gyro / IMU | `references/inertial.md` |
| Magnetometer / eCompass | `references/magnetometer.md` |
| GNSS / positioning receiver | `references/gnss.md` |
| Barometer / pressure | `references/barometer.md` |
| Humidity / environmental | `references/environmental.md` |

A part whose class has no reference file follows the general rules
of this skill alone.

Then match the part's manufacturer and interface style against
`references/vendors.md` and read the matching section as **priors to
verify** — family conventions say where that vendor's traps usually
hide (CRC feedback variants, pipelined-read staging, unlock walks,
weak product codes) and which datasheet section settles each. Facts
still enter the driver only from the documents in front of you; a
part that contradicts its family prior wins, stated in a comment.

## Step 3: Get the mikroE C Driver and Pin Mapping

Search for the Click board driver on GitHub:

```
https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/<click-name>
```

Read: `lib_<name>/include/<name>.h` (register map, pin defs) and
`lib_<name>/src/<name>.c` (init sequence, read functions).

The Click driver is normative for the **wire protocol** (framing, CRC
parameters, register addresses, strap options) — never for
**capability**: vendor examples are minimal demos that skip reset,
rate programming, filters, FIFO, and interrupts, so the feature bar
comes from the datasheet filtered through this skill's rules (reset
handling, trigger handling, datapath budget, runtime params). Vendor
**delays** are an upper bound, not a spec: a demo's fixed inter-frame
delay proves the part works *with* it, never that the part needs it —
when the datasheet documents the transaction timing (a post-edge delay
in the trigger section, a response-preparation time in the read-timing
diagram), the datasheet's numbers win. The inverse is not true: a table
elsewhere showing frames *back-to-back* (self-test, BIST, a command
sequence) does not license dropping a pipelined read's staging settle —
different transaction, different timing (see `inter_frame_sleep_ms`).

Record whether the Click wires mikroBUS RST to the sensor's
reset/enable pin — many Clicks leave RST unconnected or repurpose it,
and many parts have no reset pin at all. This fact drives the
"Reset handling" decision under Critical API Rules.

Map mikroBUS pins to NXS GPIO aliases:

| mikroBUS pin | NXS alias | MCU pin | Typical use |
|---|---|---|---|
| AN | `mkbus_an` | PA0 | Analog / GPIO |
| RST | `mkbus_rst` | PB2 | Reset (output, active low) |
| INT | `mkbus_int` | PA9 | DRDY / PPS (input) |
| PWM | `mkbus_pwm` | PA10 | PWM / wake-up (input) |
| TX | USART1 | PB6 | UART TX |
| RX | USART1 | PB7 | UART RX |
| SDA | I2C2 | PA8 | I2C data |
| SCL | I2C2 | PC4 | I2C clock |

## Step 4: Read the Sensor Datasheet

Extract:
- **Device ID register** (WHO_AM_I / CHIP_ID) and expected values
- **Register map**: configuration and data registers
- **Reset / power-up sequence** and timing requirements
- **RST-pin active level** (active-low vs active-high) from the pin
  description — sets `RESET_ACTIVE` (see the register template)
- **Interface mode selection** for parts that share I2C/SPI pins: if
  the datasheet selects the interface by sampling a shared pin at
  reset exit, no driver attribute is needed — the platform floats
  MISO through the detection window
- **Interface timing tables**: the exact per-bus clock ceiling (SPI
  SCLK maximum, I2C mode limit) — copy the number digit-for-digit
  from the timing table and cite the table in the profile comment;
  never quote a bus speed from the feature summary or from memory
- **Data-register justification**: whether output values are left- or
  right-justified and their native bit width — a left-justified N-bit
  value in 16 bits reads as `count × 2^(16−N)`, so express the base
  scale per int16 LSB and no shift is needed anywhere
- **All configuration modes**: sample rate, full-scale, filter, etc.
- **Sensitivity tables**: LSB per physical unit at each range
- **Data registers**: start address, byte count, byte order (big/little)
- **Data ready**: status register + bit mask, or interrupt pin
- **Data path**: direct reads vs the on-chip FIFO is a computed call —
  the wire-time budget under "On-chip FIFO datapath" decides it from
  the sample size, the slowest declared bus clock, and the fastest
  `sample_rate` value you declare. Extract the inputs now: whether the
  part has a FIFO, whether it is **headerless** (fixed packet layout,
  which the VM can consume) or **tagged/headered** (per-frame header —
  the VM can't parse it; direct reads only), and its
  count/enable/reset registers
- **For UART sensors**: default baud rate, command protocol

**Timing declarations from the datasheet.** If the part is a stream receiver whose message carries a solution computed earlier (GNSS), call `self.stamp_frame()` at the measure loop's frame-sync point — it stamps the raw first-byte arrival. If the datasheet documents how far the measurement predates the stamp, declare `ACQUISITION_LATENCY_US`: a ΔΣ conversion counts as its window midpoint (start-to-read interval − conversion_time/2), a filtered IMU as its group delay, a receiver as its solution latency; compute it in `configure()` when it depends on the chosen ODR/OSR. Leave it 0 when a downstream fusion filter models the delay. Event-sampled parts need nothing — the delivered DRDY edge stamps them automatically.

## Step 5: Create the Driver File

The driver is one module in the `nxs.drivers` package — `nxs upload <name>`
loads it as `nxs.drivers.<name>`. The package path differs between checkouts
(`nxs/drivers/` under the SDK root) and the sandbox (`nxs/nxs/drivers/`), so
find it from the installed package instead of hardcoding it:

```bash
python -c "import nxs.drivers, os; print(os.path.dirname(nxs.drivers.__file__))"
```

Write `<sensor_name_lowercase>.py` into that directory. The file name is the
snake_case sensor name, also the YAML `driver:` value.

**Check for a family base first.** An underscore-prefixed module in the
same directory (`_ubx_nav_pvt.py`) is a driver-family base: the shared
epoch message, measure loop, output table, and framing helpers for every
part that speaks that protocol. List them before writing anything:

```bash
python -c "import nxs.drivers, os; d=os.path.dirname(nxs.drivers.__file__); print([f for f in os.listdir(d) if f.startswith('_') and not f.startswith('__')])"
```

Read the base's docstring: it names the family and states what a
subclass still owns. When the part belongs to that family, subclass it
and implement only the part-specific surface — probe expectations,
configuration keys, rate tables, protocol variants — instead of
re-emitting the shared machinery. A standalone driver for a part the
base already covers silently detaches from every later fix to the base,
which is the failure this check exists to prevent.

When a part is the **second** member of a family whose first member is
still standalone, extract the shared machinery into a new `_family.py`
base and make both drivers subclass it, rather than copying the first
driver. Two concrete implementations are the threshold for extracting a
base — one is not.

**Name the driver after the part, never the carrier board.** The file
and class carry the part number of the silicon or module whose
datasheet defines the wire protocol — the name on the customer's BOM —
not the Click board's marketing name (a carrier is one of many homes
for the same part; a module product like a GNSS receiver uses the
module part number, which is its datasheet-bearing part). File: the
part number in lower_snake (`xyz9000.py`); class: the same in
PascalCase with digits kept (`class Xyz9000`), matching the sibling
drivers already in the package.

### I2C/SPI (Register-Based) Sensor Template

```python
"""
<SENSOR> driver for NXS VM.

<Description. Protocol. Key features.>

mikroBUS pin mapping (from mikroE <Click Name> C driver):
    INT (PA9)  → DRDY: Data ready interrupt (input)
    RST (PB2)  → RST:  Module reset (output, active low)
    ...

Datasheet: <URL>
mikroE Click: <URL>
"""

from nxs import RegisterDriver, Sample


class <SensorName>(RegisterDriver):
    # ── mikroBUS pin mapping ────────────────────────────
    PINS = {
        'drdy': 'mkbus_int',   # Data ready (input)
    }

    # ── Device ID ───────────────────────────────────────
    WHO_AM_I_REG = 0x__
    WHO_AM_I_VALUES = [0x__]

    # ── I²C strap options ───────────────────────────────
    # Every address the sensor can land on given the AD0/SDO/ADDR
    # strap pin (or jumper / 0Ω resistor). NXS scans these on load
    # by reading WHO_AM_I at each and latching the first match, so
    # users never have to know which strap is on their board.
    # For rare peripherals with >8 address options, omit this
    # list and declare `i2c_addr` as a param instead — the param
    # path skips the scan.
    # I2C_ADDRS models ONE die's strap alternatives. A package with
    # a SECOND co-resident die at its own address declares it in
    # I2C_COMPANIONS instead — see "Companion I²C devices" under
    # Critical API Rules.
    I2C_ADDRS = [0x__, 0x__]

    # ── Reset polarity (optional) ───────────────────────
    # The shared mikroBUS RST line is pulsed before each probe. Default
    # is active-low (release on physical HIGH). Set 'high' ONLY if the
    # datasheet's RST pin description says reset asserts HIGH. Omit otherwise.
    RESET_ACTIVE = 'high'

    # ── Communication profile (bus interface, optional) ──
    # Declare one profile per bus ONLY when the wire protocol deviates
    # from the conventional register bus — a standard part omits all of
    # this (the firmware default IS the conventional bus). Import the
    # profile types you use: `from nxs import ..., SpiProfile, I2cProfile`.
    # Clock ceilings are COPIED from the datasheet's interface-timing
    # table, cited in a trailing comment — never recalled or rounded.
    #   SPI_PROFILE = SpiProfile(addr_bytes=2, rw_read_level=0,
    #                            max_hz=1_000_000)   # DS SPI timing table
    #   I2C_PROFILE = I2cProfile(max_hz=400_000)     # DS I2C timing table
    #   BUS = 'i2c'   # compile-time default; switch at upload/runtime, no recompile
    # See "Communication Profile layer" under Critical API Rules.

    # ── Configuration lookup tables ─────────────────────
    # Map EVERY supported mode: YAML config key → register value.
    #
    # For a RUNTIME-TUNABLE full-scale range, the output scale must track
    # the live range, not the compile-time default. Split it: the table
    # holds only the register value, and the field's scale is a BASE
    # (per-unit-of-range) constant that NXS multiplies by the live param
    # (see set_output's scale_param). A signed 16-bit reading spans +/- the
    # range, so sensitivity = 32768/range and base = unit/32768.
    #
    # Emit SI at the source: every field with a known SI semantic (accel,
    # gyro, mag, temperature, pressure, the scalar block) publishes its
    # CANONICAL unit from constants/field_semantics.yaml — m/s^2, rad/s,
    # tesla, kelvin, pascal. Omit 'unit' for those fields (the compiler
    # inherits the canonical; declaring a conflicting one is a compile
    # error). Temperature is kelvin: fold the datasheet's Celsius zero
    # point into the offset (+273.15). Only non-SI fields (humidity %RH,
    # NMEA, custom/GENERIC) carry an authored 'unit'.
    # Example:
    # ACCEL_FS = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}   # g
    # ACCEL_BASE_SCALE = 9.80665 / 32768.0               # m/s^2/LSB, per g
    #
    # A part with a FIXED full-scale keeps the old form — one baked scale,
    # no scale_param: ACCEL = {'scale': 9.80665 / 2048.0}.

    def __init__(self):
        super().__init__()
        self._read_responses = {
            self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who in self.WHO_AM_I_VALUES

    def configure(self, config):
        # 1. Declare ALL configurable parameters with valid values,
        #    INCLUDING sample_rate. sample_rate is a runtime-patchable
        #    parameter just like full-scale or bandwidth — never bake
        #    it into the bytecode as a constant. The host must be able
        #    to change it via `nxs set sample_rate N` without
        #    re-uploading the driver.
        self.declare_param("sample_rate", values=[10, 50, 100, 250, 500, 1000],
                           default=250, unit="Hz")
        self.declare_param("range", values=[2, 4, 8, 16],
                           default=8, unit="g")
        self.declare_param("bandwidth", values=[100, 200, 400],
                           default=200, unit="Hz")
        # declare_param takes an optional kind (default "reload"): reload
        # re-runs configure() on set (a sensor re-init) — correct for the
        # config registers above. Use kind="live" only when a value is
        # applied without a VM restart by a live consumer (e.g. a driven
        # output rate); ordinary sensor config registers stay "reload".

        # 2. Read config with defaults
        sample_rate = config.get('sample_rate', 250)
        range_val   = config.get('range', 8)
        bw_val      = config.get('bandwidth', 200)

        # 3. Compute register values
        # Per the datasheet: rate_div = internal_rate / target - 1.
        rate_div  = max(0, min(255, 1000 // sample_rate - 1))

        # 4. Write registers — tag EVERY config-dependent write with param=,
        #    including the one that selects the sample rate. No software
        #    reset by default — whether this part needs one is a per-part
        #    decision keyed on the Click's RST routing and the datasheet;
        #    see "Reset handling" under Critical API Rules.
        self.write(0x21, rate_div,
                       param=("sample_rate", sample_rate))
        self.write(0x22, self.RANGE_TABLE[range_val],
                       param=("range", range_val))
        self.write(0x38, 0x01)             # enable DRDY

        # 5. Declare output fields. For a runtime-tunable range, the scale is
        #    the BASE constant and scale_param names the live range param, so
        #    the SI output tracks a `set range N` with no re-upload. A
        #    fixed-scale part drops scale_param and bakes the single scale.
        #    No 'unit' on SI-semantic fields — the canonical unit is
        #    inherited from the semantic (constants/field_semantics.yaml).
        self.set_output([
            {'name': 'accel_x', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'range'},
            {'name': 'accel_y', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'range'},
            {'name': 'accel_z', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'range'},
        ])
        self.set_sample_size(6)  # 3 × int16

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x27)       # STATUS_REG
        if not (status & 0x08):            # DRDY bit
            return None
        raw = self.read_burst(0x28, 6) # DATA_OUT
        return Sample(raw)
```

**Analog (AN) and PWM pins.** Beyond the digital buses, a Click can use
the mikroBUS analog or PWM pin:

- **`self.read_analog(ch)`** in `measure()` samples the **AN** pin (ADC)
  and returns the raw count as a **value**. Publish it through a named
  output field — `return Sample(voltage=raw)`, not a positional
  `Sample(raw)`: unlike `read_burst`, `read_analog` stages off the sample
  buffer, so a positional commit ships empty bytes. Pair it with a
  `uint16` field whose `scale` is `Vref/4096 · gain`.
- **`self.drive_pwm(freq=…, duty=…)`** in `configure()` drives the **PWM**
  pin via two `kind="live"` range params (`pwm_freq` Hz, `pwm_duty` %)
  that the host retunes at runtime, no reload.

### Command-Based I²C with CRC (command-response with per-word checksum)

**When to use**: the datasheet describes the protocol as "commands"
and "response packets with checksums" rather than a register map.
Telltale signs:

- There's no list of register addresses — instead a table of opcodes
  (1 or 2 bytes each).
- Each response comes back as `[data, data, CRC, data, data, CRC, …]`
  — 2 data bytes + 1 CRC byte per "word", repeated.
- The datasheet prescribes a measurement delay (e.g. "wait 8.3 ms
  after the measurement command before reading") — this is the STOP
  between the write and read transactions.

Examples: command-response humidity, gas, and IR-temperature parts.

```python
from nxs import I2cCommandDriver, Sample, I2cProfile
from nxs.framing import Crc, I2cFrame


class <SensorName>(I2cCommandDriver):
    """<Sensor> I²C command-response driver."""

    PINS = {}   # command-response parts of this family don't use mkbus_int

    # I²C address(es). NXS auto-detects by scanning + trying the
    # first command; no WHO_AM_I register exists on these parts.
    I2C_ADDRS = [0x__]
    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = []  # empty list = skip WHO_AM_I probe

    # ── Communication profile (optional) ────────────────
    # Many command-protocol parts cap below the bus default (e.g. 100 kHz).
    # Declare the datasheet max so NXS clamps
    # the shared bus down on load. See "Communication Profile layer".
    I2C_PROFILE = I2cProfile(max_hz=100_000)

    # CRC spec from the datasheet. E.g. poly 0x31, init 0xFF,
    # xor_out 0x00; an SMBus PEC part uses poly 0x07, init 0x00.
    FRAME = I2cFrame(
        data_bytes=2,
        crc_bytes=1,
        crc=Crc(width=8, poly=0x31, init=0xFF, xor_out=0x00),
    )

    # Measurement pattern constants — read from the datasheet.
    MEASURE_COMMAND = (0x__, 0x__)  # 1 or 2 bytes, opcode only
    MEASURE_DELAY_MS = __            # datasheet-prescribed
    MEASURE_NUM_WORDS = __           # how many (data, crc) chunks

    def probe(self):
        pass  # no WHO_AM_I; bus ACK on first command is enough

    def configure(self, config):
        self.declare_param(
            "sample_rate", values=[1, 2, 5, 10],
            default=1, unit="Hz")

        # Scales come straight from the datasheet's conversion
        # formula, landed in the canonical SI unit. Example:
        #   Temp [K]   = 273.15 + 100 * (raw / 65535)   # fold the Celsius zero into the offset
        #   RH [%]     = 0  + 100 * (raw / 65535)
        # 'unit' only on non-SI fields (e.g. humidity '%RH'); SI
        # semantics inherit the canonical unit.
        self.set_output([
            {'name': '<field1>', 'type': 'uint16', 'byte_order': 'big',
             'scale': <scale>, 'offset': <offset>},
            ...
        ])
        self.set_sample_size(<num_words> * 2)  # 2 data bytes per word
```

### Critical rules for `I2cCommandDriver`

- **CRC `xor_out` must be 0** and `feedback_style` must be `'standard'`
  (the default). The compiler's CRC verification uses the
  "`CRC(msg || emitted_crc) == 0`" property, which holds only for
  `xor_out=0`. If your datasheet specifies a different CRC, either
  rewrite the formula to match (often possible via init manipulation)
  or file a follow-up to generalise the compiler.
- **No `@measure_loop` decorator.** The measure loop is generated
  procedurally from `MEASURE_COMMAND` / `MEASURE_DELAY_MS` /
  `MEASURE_NUM_WORDS`. Don't define a `measure()` method.
- **Measurement delay is a hard lower bound.** `MEASURE_DELAY_MS`
  should be the datasheet's max measurement time plus a small margin
  (e.g. 8.3 ms max → `MEASURE_DELAY_MS = 10`). Under-specifying
  corrupts every sample; over-specifying just costs latency.
- **Response size caps at 128 bytes** (sample-buffer limit). For parts
  that return larger packets (e.g. particle-data sensors at 60 B), keep an
  eye on the header + data total.
- **CRC failure is non-fatal** at the driver level. The emitted loop
  silently skips the store and retries on the next cycle. You don't
  need to handle CRC errors in the driver.

### Command-Based I²C with On-Device Compensation (compensation barometer)

**When to use**: the part delivers raw ADC counts plus per-chip factory
coefficients and leaves the compensation polynomial to the host.
Telltale signs:

- A command protocol (reset / convert / read-ADC opcodes), not a
  register map — like the command-response parts above, but…
- …the datasheet's "PROM" or "calibration" section lists N factory
  coefficients read once at startup, and a fixed-point polynomial (with
  explicit `2^k` shifts) that combines them with the raw counts.
- The polynomial's intermediates exceed 32 bits — a coefficient × a raw
  count (`D1 * SENS ≈ 3.4e16`) is the giveaway.

Examples: compensation barometers — anything with a "first/
second-order temperature compensation" worked example in the datasheet.

The compensation runs on-device in the VM's 64-bit fixed-point ops, so
the wire carries compensated SI. The math is plain Python integer
arithmetic — the compiler infers 64-bit width from structure (any `*`
widens), with nothing to annotate. Full reference: the "Sensor Needing
On-Device Compensation" section of the author's guide.

```python
from nxs import I2cCommandDriver, Sample


class <SensorName>(I2cCommandDriver):
    """<Sensor> baro — on-device int64 compensation."""

    PINS = {}                       # polled; no DRDY line
    I2C_ADDRS = [0x76, 0x77]        # CSB strap selects the address LSB
    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = []            # empty → no WHO_AM_I probe

    CMD_RESET = 0x1E                # reloads the PROM into the chip
    PROM_C1   = 0xA2                # C1..CN at 0xA2, 0xA4, … (16-bit big-endian)

    def probe(self):
        self.send_command(self.CMD_RESET)
        self.sleep_ms(3)

    def configure(self, config):
        self.declare_param("sample_rate", values=[1, 2, 5, 10, 25],
                           default=10, unit="Hz")

        # Read the factory coefficients ONCE. Bound to self.<attr>, they
        # live in the work buffer and persist into the measure loop.
        self.c1 = self.read(self.PROM_C1 + 0, 2)
        self.c2 = self.read(self.PROM_C1 + 2, 2)
        # … through cN at PROM_C1 + 2*(N-1) …

        # The descriptor's scale/offset projects the polynomial output to
        # the inherited canonical SI units: pressure ×1.0 → pascal;
        # temperature centi-°C ×0.01 +273.15 → kelvin.
        self.set_output([
            {'name': 'pressure',    'type': 'int32', 'byte_order': 'big',
             'scale': 1.0},
            {'name': 'temperature', 'type': 'int32', 'byte_order': 'big',
             'scale': 0.01, 'offset': 273.15},
        ])
        self.set_sample_size(8)     # two int32 fields

    @I2cCommandDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        self.send_command(0x50)     # convert D2 (temperature), max oversampling
        self.sleep_ms(10)
        d2 = self.read(0x00, 3)     # 24-bit ADC read
        self.send_command(0x40)     # convert D1 (pressure), max oversampling
        self.sleep_ms(10)
        d1 = self.read(0x00, 3)

        # Plain Python; every `*` lowers to the int64 path automatically.
        dt   = d2 - self.c5 * 256
        temp = 2000 + dt * self.c6 // 2**23
        off  = self.c2 * 2**16 + self.c4 * dt // 2**7
        sens = self.c1 * 2**15 + self.c3 * dt // 2**8
        p    = (d1 * sens // 2**21 - off) // 2**15
        return Sample(pressure=p, temperature=temp)
```

### Critical rules for on-device compensation

- **Coefficients via `self.read(reg, width)` in `configure()`, bound to
  `self.<attr>`.** `width` is the byte count (PROM words are 2 B,
  big-endian). The `self.` binding is what makes them persist into
  `measure()`; a bare local would die at the end of the cycle.
- **Register addresses stay literals in `measure()`** (`0x50`, `0x00`),
  same as every driver. The *only* `self.*` references allowed in
  `measure()` arithmetic are the coefficients bound in `configure()`.
- **Write plain integer math.** A multiply always widens to 64-bit and
  propagates through the surrounding `+ - //`. `* 2**k` lowers to a
  shift, `// 2**k` to an arithmetic shift; `//` by a non-power-of-two is
  rejected. Using a wide value in a 32-bit-only context is a compile
  error, not a silent truncation.
- **Emit with `return Sample(field=value, …)`** — kwargs name the fields
  declared by `set_output()`. A kwarg with no matching field is a hard
  compile error.
- **Second-order terms are plain Python** — `if temp < 2000:` is ordinary
  control flow, `f * f` is a multiply. No baro-specific machinery.
- **Class conventions live in `references/barometer.md`** (read in
  Step 2) — the output set, the no-altitude-on-device rule, the
  `osr` canon, and the worked-example cross-check.
- **CRC is RISC, not a new opcode.** A PROM CRC-4/CRC-16 integrity check
  decomposes into LOAD/XOR/shift/AND/branch in the cold `configure()`
  path. Only the per-sample `OP_CRC8` hot path earns a dedicated op.

### Custom SPI Framing (Fixed-Width Word with CRC)

**When to use**: the SPI protocol packs `[rw, addr, optional flags,
data, crc]` into a single fixed-width word (typically 32 bits) with
an embedded CRC, rather than the simple `addr+data` model. Telltale
signs in the datasheet:

- The SPI transaction diagram shows ONE word per access (e.g., 32 bits).
- A bit-field table lists `rw`, `addr`, `data`, `crc` with explicit
  widths summing to the word size.
- A CRC specification: polynomial, init value, xor_out, "covers
  bits N..M" — sometimes with chip-specific feedback rules.
- A "two-frame read" rule: frame K issues the request, frame K+1
  carries the response on MISO (with or without an inter-frame
  settle delay).

Examples: CRC-framed SPI parts (32-bit word, CRC-8 with chip-specific
feedback, 1-frame pipeline). NOT for
"plain" SPI sensors that work like I²C with chip-select (those use
the regular `RegisterDriver` template above).

```python
from nxs import RegisterDriver, Sample, SpiProfile
from nxs.framing import Crc, SpiFrame


class <SensorName>(RegisterDriver):
    """<Sensor> custom-framed SPI driver."""

    # Restrict to SPI only — these parts have no I²C variant.
    # `BUSES` is the switch; `I2C_ADDRS` is unnecessary here (it
    # defaults to `[]` and is unused when I²C isn't in BUSES).
    BUSES = ('spi',)

    PINS = {
        'drdy': 'mkbus_int',         # Data ready (input, optional)
    }

    # Some parts have no traditional WHO_AM_I but expose a
    # FIXED_VALUE register that always reads back a constant. Use
    # that as the probe anchor; WHO_AM_I_VALUES carries the high
    # byte (NXS's probe machinery compares the MSB).
    WHO_AM_I_REG = 0x__              # FIXED_VALUE register address
    WHO_AM_I_VALUES = [0x__]         # high byte of the fixed value

    # ── Communication profile (clock + mode) ────────────
    # A FRAME driver composes its own address/data words, so the profile
    # here carries only the clock ceiling and SPI mode (CPOL/CPHA 0-3);
    # the framing switches (addr_bytes, rw_read_level, …) don't apply
    # under FRAME. Firmware applies min(max_hz, DTS) and the mode before
    # probe. See "Communication Profile layer" under Critical API Rules.
    SPI_PROFILE = SpiProfile(max_hz=8_000_000, mode=0)

    # Frame definition. `fields` is MSB-first within the word; widths
    # must sum to `width`. `crc.covers` lists which fields the CRC
    # covers (in declaration order). `feedback_style` is `'standard'`
    # for textbook CRCs; some parts use chip-specific variants
    # (e.g. `'input-lsb'`) — match the datasheet's worked examples.
    # `read_pipeline=N`: the response to request K arrives in frame
    # K+N (most parts: 0; two-frame-handshake parts: 1).
    # `inter_frame_sleep_us=N` (or `_ms`): response-staging settle
    # between consecutive FRAME reads — one internal update period of
    # the part's datapath, with 2x margin (8 kHz datapath -> 250 us).
    # `status_ok=(field, value)`: the per-frame return-status field and
    # its "success" code, straight from the protocol chapter's status
    # truth table.
    #
    # DECLARE EVERY INTEGRITY FIELD THE PROTOCOL DEFINES. A declared
    # `crc` / `status_ok` makes the compiler verify every harvested
    # measure response on-device (CRC recomputed and compared, status
    # mask-checked; a mismatch drops that tick, and sustained failure
    # escalates through the sample watchdog). Omitting a CRC or status
    # field the datasheet defines is a capabilities lie: corrupted or
    # unprepared responses would publish as plausible SI samples.
    # Self-check: the compiled measure section carries one OP_CRC8 per
    # harvested word (disassemble to confirm).
    FRAME = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x__, init=0x__, xor_out=0x__,
                covers=('rw', 'addr', 'rs', 'data'),
                feedback_style='standard'),
        read_pipeline=1,
        inter_frame_sleep_us=250,
        status_ok=('rs', 0b__),   # the status table's "success" code
    )

    def __init__(self):
        super().__init__()
        # Seed the host-side tracer so probe()'s assert succeeds at
        # compile time. Use the FULL 16-bit fixed value here.
        self._read_responses = {
            self.WHO_AM_I_REG: [0x____],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who == 0x____            # full 16-bit fixed value

    def configure(self, config):
        # `sample_rate` declaration depends on the part's rate plumbing
        # — see "Trigger handling": a rate register → declare
        # unconditionally and tag the divider write; no rate register →
        # declare only inside `if config.get('trigger') == 'poll':`.
        self.declare_param("sample_rate", values=[100, 250, 500],
                           default=250, unit="Hz")

        # Reset / unlock / register-write sequence per the datasheet.
        # `self.write(reg, val)` composes a full FRAME (rw=1, addr,
        # data, CRC) automatically — you supply only the address
        # and the 16-bit data value.
        self.write(0x__, 0x____)        # reset
        self.sleep_ms(__)

        # ── DRDY / ODR pin routing ─────────────────────────────
        # If the part has an ODR / DRDY output pin — including a
        # fixed-rate sync with no divider — ALWAYS configure the
        # routing in `configure()` per the datasheet's documented
        # procedure. DRDY mode pulses the pin at the chip's
        # internal sampling rate — that's a hardware-locked, jitter-
        # free clock that beats firmware polling on every metric.
        # `trigger="from_config"` below lets the user pick `drdy` (the
        # default) or `poll` at upload time, but the chip-side routing
        # only takes effect in DRDY mode; `poll` mode just ignores the
        # pulses, so configuring it both ways is harmless.
        # Some parts gate the routing behind an unlock + bank switch
        # sequence — follow the datasheet's procedure verbatim. Fixed
        # magic words are plain writes; a routing register whose
        # reserved bits are undocumented (or whose datasheet mandates
        # read-modify-write) is `write_modify`, which reads and merges
        # on-device:
        #     self.write(MODE_REG, UNLOCK_TCODE_1)
        #     self.write(MODE_REG, UNLOCK_TCODE_2)
        #     ... etc until banks are unlocked ...
        #     self.write(BANK_SELECT_REG, ROUTING_BANK)
        #     self.write_modify(ROUTE_REG, set_bits=ROUTE_BIT)
        #     ... routing writes ...
        #     self.write(BANK_SELECT_REG, 0x0000)   # back to bank 0

        # If the chip has additional configuration banks for runtime
        # params (e.g., full-scale-range, bandwidth), follow the same
        # bank-switch dance and tag config-dependent writes with
        # `param=("name", value)` so they're runtime-patchable:
        #     self.write(BANK_SELECT_REG, BANK_N)
        #     self.write(BANK_N_REG_X, value, param=("name", val))
        #     ...
        #     self.write(BANK_SELECT_REG, 0x0000)

        self.set_output([
            {'name': '<field1>', 'scale': <scale>, 'unit': '<unit>'},
            ...
        ])
        self.set_sample_size(<num_words> * 2)   # 2 data bytes per word

    @RegisterDriver.measure_loop(trigger="from_config", sample_rate=250)
    def measure(self):
        # FRAME-aware burst read: emits N+read_pipeline interleaved
        # MEMCPY_IMM/REG_XFER pairs sharing a rotating buffer slot,
        # with `inter_frame_sleep_ms` between consecutive frames,
        # and packs the data field of each response into the output
        # region in declared order.
        raw = self.read_words(<start_reg>, <num_words>)
        return Sample(raw)
```

### Critical rules for FRAME-based SPI drivers

- **Default to DRDY mode.** If the part has an ODR / DRDY output pin —
  including a fixed-rate sync with no divider — configure the routing
  in `configure()` per the datasheet's documented procedure and use
  `trigger="from_config"` on the measure_loop decorator (see "Trigger
  handling"). Hardcoding `trigger="poll"` on a part with a data-ready
  output is a capabilities lie; reserve poll for parts with genuinely
  no such pin.
- **`BUSES = ('spi',)`** locks the driver to SPI. Without it the
  compiler tries to also generate an I²C path which makes no sense.
  This is the only switch needed — `I2C_ADDRS` defaults to `[]` and
  is unused when I²C isn't in BUSES, so don't bother declaring it.
- **You don't call `FRAME.compose(...)` directly** — use `self.write
  (reg, val)` and `self.read(reg)` / `self.read_words(reg, N)` and
  the compiler generates the right wire bytes (with CRC) for you.
- **`inter_frame_sleep_us` / `inter_frame_sleep_ms`** is the
  response-staging settle between two consecutive FRAME reads of a
  **pipelined** part (`read_pipeline` ≥ 1): the part needs time to
  prepare request K's response before frame K+1 clocks it out. **Derive
  it from the part's internal update rate**: a sampled register stages
  its response on the next datapath tick, so the settle is one internal
  update period with 2× margin — an 8 kHz datapath (125 µs period)
  takes `inter_frame_sleep_us=250`. Never default to a whole
  millisecond just because the ms knob's minimum is 1: on a fast
  datapath that is ~8× the real floor and divides the achievable burst
  rate by the same factor. Setting the settle to 0 corrupts every
  sampled read (stale/idle MISO, values read as zero or 0xFFFF), and
  the trap is *clock-dependent*, so it hides at low SPI clocks: with no
  gap the staging window IS the frame duration (~51 µs at 625 kHz,
  ~3.2 µs at 10 MHz). Do **not** zero the settle because some datasheet
  table shows "back-to-back frames" — a self-test / BIST or command
  table describes a *different* transaction than the pipelined
  data-register read. A documented post-edge delay ("wait N µs after
  the sync edge") is not a frame gap — carry it as one
  `self.sleep_us(N)` at the top of the measure loop. The hardware
  checklist validates data integrity at the chosen value AND at the
  profile's top clock, since that is where an under-set gap surfaces.
- **`read_pipeline=N`** captures the two-frame-handshake pattern:
  frame K issues the request, frame K+N carries the response. Most
  parts: 0. IMUs that need a settle frame: 1. Check the datasheet's
  read transaction timing diagram.
- **CRC `feedback_style`**: `'standard'` for textbook CRCs (left-shift,
  XOR with poly if MSB set). Some parts use chip-specific feedback —
  if the standard CRC fails the datasheet's worked examples, the
  driver layer can declare a registered variant (e.g. `'input-lsb'`).
- **`read_words(reg, N)`** packs `N * 2` bytes (one 16-bit data field
  per response) into the output region, which ends at the rotating
  TX/RX slot — for a 32-bit FRAME that is 124 bytes, so 62 words max.
  Splitting into multiple bursts means multiple unlock/bank-switch
  dances if the data spans banks.
- **Multi-bank parts**: if the chip has separate banks selected via a
  `bank_select` register (e.g. bank 0 for data, banks 6/7 for
  full-scale-range, etc.), drivers must `write(bank_select, N)`
  before bank-N writes and switch back to bank 0 before measure
  reads data registers. Some parts gate bank writes behind an
  unlock sequence (e.g. a "tcode" walk) — follow the datasheet's
  procedure verbatim.
- **Reserved bits may carry non-zero factory state.** When the
  datasheet mandates read-modify-write for a register (or leaves its
  reserved bits undocumented), use
  `self.write_modify(reg, set_bits=…, clear_bits=…)`: it reads the
  register **on-device at load**, applies the mask, and writes back
  with the frame CRC recomputed at runtime, so no unit's factory bits
  are ever baked into the image. Hardcoding a full-register value
  there zeroes bits the datasheet never documented — a known way to
  wedge silicon. Fixed magic words (unlock tcodes) stay plain
  `write()`. A configurable field on such a register is still a runtime
  param: tag the `write_modify` with `param=(name, value)` and pass
  `clear_bits` as the whole field mask (constant across values) so the
  OR `set_bits` immediate is the single patch site `nxs set` rewrites.
  A field split across two registers tags each write with the same
  param (two sites, ≤ `MAX_PATCH_SITES`). Only a field whose write is
  genuinely value-independent stays a compile-time key.
- **`feedback_style='input-lsb'`** is registered but most chips use
  `'standard'`. If you're unsure, generate one read FRAME with both
  values, compare against the datasheet's worked example bytes, and
  pick the matching one.

### Literal SPI Words (computed command bits — parity-framed parts)

A `FRAME` schema composes words from fixed-layout fields plus an
auto-computed CRC. Some parts put *computed* bits elsewhere in the word
— a parity bit over the address+rw field is the classic case — and
their command words don't decompose into FRAME fields. Don't reach for
compiler features: every command word is a compile-time constant, so
compute it in plain Python at class definition time, store it as an
UPPER_CASE class constant, and clock it verbatim with
`self.xfer(word)`. Response checks (parity, error flags) are ordinary
shift/mask arithmetic in `measure()`.

```python
from nxs import RegisterDriver, Sample, SpiProfile


def _read_word(addr):
    """Read command word: parity(15) | rw=1(14) | addr(13:0), even
    parity over bits 14:0 — plain Python, evaluated at class
    definition, checkable against the datasheet's example words."""
    word = (1 << 14) | addr
    return word | ((bin(word).count("1") & 1) << 15)


class ParityFramedEncoder(RegisterDriver):
    BUSES = ('spi',)     # literal wire words are SPI-only
    PINS = {}            # no data-ready output → poll (Trigger handling)
    SPI_PROFILE = SpiProfile(max_hz=10_000_000, mode=1)

    # No identity register on this part: every response is parity- and
    # error-gated, so the first gated data read is the probe.
    WHO_AM_I_VALUES = []
    WHO_AM_I_SKIP_REASON = "no identity register; reads are parity-gated"

    CMD_POSITION = _read_word(0x3FFF)

    def configure(self, config):
        # Hardcoded-poll driver → declare unconditionally (the poll
        # branch gate applies to trigger="from_config" parts only).
        self.declare_param("sample_rate", values=[10, 50, 100, 250],
                           default=100, unit="Hz")
        self.set_output([
            # 14-bit angle: full turn = 2^14 counts, so the scale
            # divisor is the count space (16384), never the maximum
            # code (16383) — that off-by-one accumulates a full count
            # of error per revolution.
            {'name': 'angle', 'type': 'uint16',
             'scale': 2 * 3.141592653589793 / 16384},
        ])
        self.set_sample_size(2)

    @RegisterDriver.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.xfer(self.CMD_POSITION)      # prime: response arrives next word
        a = self.xfer(self.CMD_POSITION)  # response to the previous word
        if a & 0x4000:                     # error flag → drop the sample
            return None
        t = a >> 8                         # even-parity fold: xor the
        p = a ^ t                          # halves down until bit 0
        t = p >> 4                         # holds the word's parity
        p = p ^ t
        t = p >> 2
        p = p ^ t
        t = p >> 1
        p = p ^ t
        if p & 1:                          # parity violation → drop
            return None
        angle = a & 0x3FFF
        return Sample(angle=angle)


# Cross-check the computed words against the datasheet's worked
# examples — a wrong parity helper fails the compile, not the bench.
assert ParityFramedEncoder.CMD_POSITION == 0xFFFF
```

### Critical rules for literal-word (`xfer`) drivers

- **`BUSES = ('spi',)` is REQUIRED** — a literal word has no defined
  wire behaviour on I²C, and the compiler rejects `xfer` on any other
  `BUSES` declaration.
- **`x = self.xfer(word, width=2)`** stages the word's bytes MSB-first
  off the sample region, clocks them full-duplex with one CS assertion
  per word, and loads the response — unsigned, MSB-first, the full
  `width` (1, 2, or 4 bytes; default 2). In statement position the
  response is discarded — that's the pipeline-priming idiom.
- **Pipelined responses sequence naturally.** On a part whose response
  to word N arrives during word N+1, prime once in statement position,
  then chain assignments — each `xfer` returns the *previous* word's
  response: `self.xfer(CMD_A)`, then `a = self.xfer(CMD_B)` (A's
  response), then `b = self.xfer(CMD_B)` (B's response — repeating the
  final word re-requests it; the extra response is simply never
  clocked out).
- **Cross-check every computed word** against the datasheet's worked
  example words with a module-level `assert` — the parity helper is
  exactly the kind of arithmetic that reads right and is wrong.
- **Parity folds are two-statement steps.** `p = a ^ (a >> 8)` nests an
  expression in an operand and does not compile; write `t = a >> 8`
  then `p = a ^ t`, halving the shift down to 1, then gate on
  `if p & 1: return None`.
- **Gates are integrity, not quality**: a parity or error-flag
  violation drops the sample (`return None`) — corrupt words never
  publish, and the part's own diagnostics fields (gain, field
  magnitude) are ordinary outputs, not gates.
- **Inter-word gaps**: the per-word CS turnaround already exceeds
  short CS-high minimums; when the datasheet mandates a longer gap,
  insert `self.sleep_us(n)` between words.
- **Writes are two consecutive words** — command then data, each with
  its computed bits folded in at class definition — issued as two
  statement-position `xfer` calls.

Stream drivers split chip-touching work and host-side declarations
across three methods:

| Method | What it does | What it does NOT do |
|---|---|---|
| `probe()` | Fixed handshake: configures the bus baud, sends a known **fixed** command (no runtime params), **verifies a structured response** with a bounded timeout. Failure here → runner enters `PROBE_FAILED`, advances slots. | No runtime params. No `declare_param`, no `set_output`. |
| `configure(config)` | Declares params via `declare_param`, sets `set_output` + `set_sample_size`. **MAY** send param-driven chip-config commands using the `stage(...) → compute_checksum(...) → send_staged(...)` pattern so `set <param>` re-patches the bytes in place without re-uploading. | Must not repeat the probe handshake — by the time configure runs, the chip already responded once in probe. |
| `measure()` | Frames one logical record per `vm_sample` using `read_until` / `read_n` framing helpers. | No probing, no chip-config writes. |

The split puts the handshake-and-verify in `probe()` so a missing or
wrong-baud chip fails fast — the runner maps any negative VM return
from `probe()` through `on_probe_error()` → `PROBE_FAILED` → slot
advance — and keeps `configure()` to declarations plus the
param-driven stream-setup writes. Stream-setup belongs after probe
because there's no point sending baud / rate / mode commands to a
chip that hasn't acknowledged it's the right one.

```python
from nxs import StreamDriver, SensorDriver
from nxs.compiler import ChecksumFletcher


class <SensorName>(StreamDriver):
    DEFAULT_BAUD = 38400
    PINS = { 'pps': 'mkbus_int' }

    # ── Communication profile ───────────────────────────
    # A stream driver sets its baud with set_baud() in probe() (below).
    # UART framing profiles are not yet applied by firmware — declaring a
    # UART_PROFILE is a compile error, so use set_baud() for now.

    RATES = {1: 1000, 5: 200, 10: 100}

    # Fixed probe command — never changes with config.
    PROBE_FRAME = bytes([ ... 14 B with pre-baked checksum ... ])
    ACK_PREFIX  = bytes([ ... first N bytes of expected response ... ])

    # ── Probe: the chip's response to a known command IS the WHO_AM_I.
    def probe(self):
        self.set_baud(self.DEFAULT_BAUD)
        self.sleep_ms(100)
        self.write(self.PROBE_FRAME)
        ack = self.read_n(len(self.ACK_PREFIX) + 2,  # + CK_A/CK_B
                          timeout_ms=2000)
        ack.expect(self.ACK_PREFIX)

    # ── Configure: declarations + (optionally) param-driven chip
    #    writes via stage/checksum/send so `set` is in-place patchable.
    def configure(self, config):
        self.declare_param("baud", values=[9600, 38400, 115200],
                           default=38400, unit="baud")
        self.declare_param("rate", values=list(self.RATES.keys()),
                           default=1, unit="Hz")

        # UART sensors: driver controls trigger + outer cadence.
        config['sample_rate'] = 100
        config['trigger'] = 'poll'

        rate_hz = config.get('rate', 1)
        meas_rate_ms = self.RATES[rate_hz]

        # Build a CFG-RATE frame with the user-selected meas_rate_ms.
        # `stage()` records a patch site at frame-offset 6 (size=2)
        # so a later `set rate=N` rewrites just those two bytes.
        # `compute_checksum()` then recomputes the trailing Fletcher
        # bytes over the patched frame at runtime.
        frame = self._build_cfg_rate_frame(meas_rate_ms)
        # patch tuple: (name, requested_value, encoded_value,
        # frame_offset_of_first_patchable_byte, size_in_bytes)
        self.stage(frame,
                   patch=("rate", rate_hz, meas_rate_ms, 6, 2))
        self.compute_checksum(ChecksumFletcher(),
                              start_off=2, length=10, dst_off=12)
        self.send_staged(len(frame))
        self.sleep_ms(100)

        self.set_output([
            {'name': 'data', 'type': 'string', 'count': 96,
             'scale': 1.0, 'unit': ''},
        ])
        self.set_sample_size(96)

    # ── Measure: one logical record per vm_sample.
    @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.read_until(b'\n', max=96)
        self.store_sample_n()
```

If the chip needs no runtime-tunable config (just declarations),
omit the `stage(...) → compute_checksum(...) → send_staged(...)`
block from `configure()` entirely — `probe()` already left the
chip in a usable state.

### Stream-driver probe patterns

Stream-driver `probe()` sends a known command and verifies a
structured response. Three worked examples by wire format:

**UBX (a binary-config GNSS)** — send a CFG-VALSET (or any
config-class write), expect UBX-ACK-ACK at the top of the response:

```python
def probe(self):
    self.set_baud(self.DEFAULT_BAUD)
    self.sleep_ms(100)
    # 24-byte UBX-CFG-VALSET that sets CFG_RATE_MEAS = 1000 ms.
    self.write(self._build_cfg_valset(rate_hz=1))
    # A factory-default receiver broadcasts NMEA at power-on, so the
    # RX buffer may hold sentence bytes ahead of the ACK. Sync to the
    # UBX header first — 0xB5 never occurs in NMEA text — so the
    # expect can't land mid-sentence, then read the ACK-ACK body:
    #   05 01 02 00 06 8A CK_A CK_B
    self.read_until(b'\xb5\x62', timeout_ms=2000)
    ack = self.read_n(8, timeout_ms=2000)
    ack.expect(b'\x05\x01\x02\x00\x06\x8a')
    # CK_A, CK_B are chip-computed; not verified.
```

**SBF (a text-command GNSS)** — send a text-mode sync +
no-op command, expect `$R+<echo>` ACK:

```python
def probe(self):
    self.set_baud(self.DEFAULT_BAUD)
    self.sleep_ms(100)
    # 10 'S' chars puts the chip into command mode regardless of
    # whatever stream it was previously emitting.
    self.write(b'SSSSSSSSSS')
    self.sleep_ms(50)
    # `sdio,COM1,CMD,none\r\n` disables SBF output on COM1 — chip
    # echoes back `$R+sdio,COM1,CMD,none\r\n` (or `$R?...` on NACK).
    self.write(b'sdio,COM1,CMD,none\r\n')
    ack = self.read_n(4, timeout_ms=2000)
    ack.expect(b'$R+s')   # `$R+` = ACK prefix; `s` = first echo byte.
```

**AT-command (cellular / BLE / GNSS over modem AT layer)** — send
`AT\r\n`, expect `OK\r\n` (most modems also echo the command):

```python
def probe(self):
    self.set_baud(self.DEFAULT_BAUD)
    self.sleep_ms(100)
    self.write(b'AT\r\n')
    # With command echo on: AT\r\r\nOK\r\n (8 bytes).
    # With echo off:        OK\r\n          (4 bytes).
    # Pick one based on the modem's default — most default echo-on.
    ack = self.read_n(8, timeout_ms=1000)
    ack.expect(b'AT\r\r\nOK\r')
```

If `read_n` doesn't see `count` bytes within `timeout_ms`, it emits
`OP_ERROR ERR_CODE_TIMEOUT` → `VmErr::TIMEOUT` (-417). If `.expect()`
sees a mismatch on any byte, it emits `OP_ERROR ERR_CODE_MISMATCH`
→ `VmErr::MISMATCH` (-418). Both surface to the runner the same
way as register-driver WHO_AM_I mismatch: probe fails, slot advances.

### Stream-driver framing patterns

For text-line or length-prefixed binary protocols, use the framing
helpers in `measure()` so each `vm_sample` carries one complete
logical record (NMEA sentence, SBF frame, UBX message). The helpers
compile to bytecode loops over RISC primitives — no protocol-
specific firmware op is needed. Pick the helper that matches the
chip's wire format.

**Delimiter pattern** (NMEA `\n`, AT-commands `\r\n`,
line-oriented text):

```python
@SensorDriver.measure_loop(trigger="poll", sample_rate=100)
def measure(self):
    self.read_until(b'\n', max=96)   # one sentence into sample_buf
    self.store_sample_n()             # commit exactly len bytes
```

`read_until` reads UART bytes one-by-one until the delimiter is
encountered (delimiter included in the captured prefix) or `max`
bytes accumulated. The delimiter may be a single byte (`b'\n'`) or
a multi-byte sequence (`b'\xb5\x62'` for UBX sync, `b'$@'` for
SBF sync, `b'\r\n'` for line endings). The total length lands in
the `__cursor` runtime register; `store_sample_n()` reads it and
commits exactly that many bytes. Both `read_until` and `read_n`
accept `timeout_ms`, emitting the TIMEOUT error path on expiry —
mandatory in `probe()` (a dead chip must fail loudly), normally
omitted in `measure()` (the poll loop waits for data by design).

Self-similar prefix/suffix delimiters (which would require full
KMP backtracking) are rejected at compile time. `b'\xb5\x62'`,
`b'$@'`, `b'\r\n'`, `b'OK\r\n'` are all fine; `b'\xaa\xaa'` or
`b'abab'` are not.

**Framed binary record pattern** (any sync + fixed header + payload +
checksum wire format). `read_n` takes a compile-time count — frame
lengths are fixed per record type, so configure the module to emit ONE
record type and size the read to it. The record lands at `sample_buf[0]`,
so commit it with `store_sample()` (the compile-time `set_sample_size`),
NOT `store_sample_n()`. `store_sample_n()` publishes the `__cursor`
scratch register, which the intervening `match`/`verify_checksum` may
overwrite — it belongs only to the variable-length `read_until` path.
Gate the header with `match(...)` and the payload with
`verify_checksum(...)`; both return a mismatch count (0 = pass):

```python
@SensorDriver.measure_loop(trigger="poll", sample_rate=100)
def measure(self):
    self.read_until(b'\xa5\x5a')     # sync to the frame; record lands at 0
    self.read_n(30)                  # header(4) + payload(24) + ck(2)
    m = self.match(0x02, 0x11, 0x18, 0x00)   # class, id, len LE
    if m != 0:
        return None                  # foreign frame — resync next pass
    bad = self.verify_checksum(ChecksumFletcher(), 0, 28, 28)
    if bad != 0:
        return None                  # corrupt frame — drop
    self.store_sample()              # fixed-length record: compile-time size
```

Publish the payload as **typed fields at their record offsets** —
`set_output` with `'at': <offset>` per field (gaps over headers,
reserved bytes, and checksums carry no fields) — so a binary GNSS's
position lands on the geodetic SI semantics instead of an opaque
blob. The full geodetic field vector, the altitude datum rule, and
the binary-vs-NMEA protocol policy live in `references/gnss.md`
(read in Step 2).

**Protocol variants.** A module whose protocols need different framing
bytecode (binary records vs a text stream) keeps BOTH in one driver:
one measure loop per protocol tagged `when=("protocol", "<value>")`,
exactly one `default=True`, and `configure()` branching on
`config['protocol']` (stamped before it runs). The choice is a
compile-time config key (`--config protocol=...`), not a runtime
param — switching means uploading the other config, or storing both
in driver-store slots and cycling.

### Runtime-tunable parameters with `compute_checksum`

When a config parameter (like a GNSS's `rate`) controls bytes that
appear inside a frame WITH a CRC/checksum, the checksum bytes
must be computed at runtime — otherwise patching the rate bytes
leaves the checksum stale and the chip rejects the frame.

The recipe: stage the frame body via `self.write(b, param=(…))`
calls (so the rate bytes are patchable) but compute the checksum
using `self.compute_checksum(spec, start, length, dst)` so the
checksum recomputes on every reload.

```python
def configure(self, config):
    self.declare_param("rate", values=[1, 5, 10, 20], default=1, unit="Hz")
    rate_hz = config.get('rate', 1)
    meas_rate_ms = self.RATES[rate_hz]

    # Stage the UBX frame in sample_buf. Class + ID + length + payload.
    # Rate bytes (meas_rate_ms LSB/MSB) are param-tagged so nxs
    # `set rate` can patch them in place.
    self.write(...)  # … etc, building the frame at known offsets

    # Recompute the two Fletcher checksum bytes at the end. This loop
    # runs every time the bytecode re-executes — including after `set
    # rate` patches the rate bytes — so the checksum is always correct.
    self.compute_checksum(ChecksumFletcher(),
                          start_off=2, length=22, dst_off=24)
```

Checksum-spec choices today: `ChecksumFletcher` (UBX, TCP, NTP),
`ChecksumXorFold` (NMEA), `ChecksumPolynomial(poly, init, xor_out)`
(any polynomial CRC-8 — dispatches to the firmware's `OP_CRC8`
when width is 8). Adding a new spec subclass for a new family
(Modbus CRC-16, LRC, etc.) is a host-side addition that doesn't
touch firmware.

## Critical API Rules

### Companion I²C devices (multi-die packages)

Some packages put TWO independent I²C slaves on one bus — a primary
die plus a companion die at its own fixed address, often with register
maps that overlap by number. One package is still ONE driver: splitting
the dies across two drivers ships a fraction of the part's measurands
per upload, which is a capability reduction. Declare the companion and
assemble one fused sample:

```python
class TwoDieCombo(RegisterDriver):
    BUSES = ('i2c',)                  # REQUIRED: companions are I²C-only
    I2C_ADDRS = [0x30, 0x31]          # the PRIMARY die's strap scan
    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = [0x21]          # primary identity

    I2C_COMPANIONS = {
        'aux': {'addr': 0x0D,          # fixed address, never a strap
                'who_am_i_reg': 0x0F,
                'who_am_i_values': [0x33]},
    }
```

- **Address topology picks the primary — never identity strength.**
  The die whose address is strap-selectable is the PRIMARY: the
  `I2C_ADDRS` scan exists only for the primary, and a companion has
  exactly ONE fixed address, so inverting the roles hardcodes one
  strap and orphans every board strapped the other way. A die at a
  genuinely fixed address is the companion. The prologue
  identity-checks every die wherever it sits, so a stronger anchor
  on the fixed-address die is a reason to give the COMPANION check
  that anchor — not to make that die primary.
- **Reach the companion per access** with `dev=` on `read` / `write` /
  `read_burst` / `write_burst`, in `probe()`, `configure()`, and
  `measure()`: `self.write(0x1B, 0x82, dev='aux')`,
  `stat = self.read(0x18, dev='aux')`,
  `self.read_burst(0x10, 6, into=6, dev='aux')`. Each access is
  **bracketed** — the compiler retargets, performs the access, and
  restores the primary — so the bus always rests at the primary and no
  drop path or probe retry can start on the wrong die.
- **Every die carries its own identity anchor.** The companion's
  WHO_AM_I is checked in the probe prologue (error code 0xC1, distinct
  from the primary's 0xC0, so the bench can tell "wrong primary" from
  "companion missing"). A companion with no identity register declares
  `who_am_i_skip_reason` — the same audit contract as the primary's
  `WHO_AM_I_SKIP_REASON`.
- **Capability parity spans ALL dies**: every documented measurand and
  setting of every die is published/exposed or its exclusion stated —
  a fused sample, not a per-die driver family.
- **Registers are keyed per die.** Overlapping register numbers across
  dies are normal (the address disambiguates on the wire); a
  `param=`-tagged write on one die never collides with the same
  register number on another.
- **Mock responses for companion reads** key on the tuple:
  `self._read_responses = {0x00: [0x21], ('aux', 0x0F): [0x33]}`.
- Trace-time seeding, mag-style single-shot conversions, and stale-hold
  fusion (companion bytes persist in the sample buffer between updates)
  compose with the existing rules — nothing else changes.

### Communication Profile layer (optional)

A register driver's bus wire protocol is declared as one **profile per
bus** — a keyword-constructed descriptor that reads like the bus-interface
section of the datasheet, with no vendor names. The compiler bakes every
declared profile into the image; the firmware applies the one matching the
active `bus` param to the peripheral before `probe()` runs. Every input to a
profile is a datasheet fact or a fixed board fact — resolve them yourself and
generate a complete driver; never stop to ask the user which buses, clocks, or
switches to declare. The emitted `BUSES` / `BUS` / `*_PROFILE` stay as knobs
they can tweak afterward.

```python
from nxs import RegisterDriver, Sample, SpiProfile, I2cProfile

class MySensor(RegisterDriver):
    # Every number here is copied from the datasheet's interface-timing
    # table and cited — never recalled, rounded, or taken from the
    # feature summary (summaries round up; the timing table is the contract).
    SPI_PROFILE = SpiProfile(addr_bytes=2, rw_read_level=0,
                             max_hz=1_000_000)   # DS SPI timing table
    I2C_PROFILE = I2cProfile(max_hz=400_000)     # DS I2C timing table
    BUS = 'i2c'                 # compile-time default transport
```

**`SpiProfile`** switches (defaults describe the conventional bus):

| Switch | Default | Datasheet fact it captures |
|---|---|---|
| `max_hz` | DTS | SPI clock ceiling; firmware clamps to `min(this, DTS)` |
| `mode` | 0 | SPI mode 0–3 (CPOL/CPHA) |
| `bit_order` | `'msb'` | `'msb'` or `'lsb'` first |
| `addr_bytes` | 1 | address bytes on the wire (2 for parts that frame an 8-bit address across two serialized bytes) |
| `rw_read_level` | 1 | R/W bit (bit 7) level for a read; a write inverts it |
| `dummy_bytes` | 0 | dummy bytes after the address, before read data |
| `auto_inc` | `'implicit'` | `'implicit'` / `'msb'` / `'none'` |

**`I2cProfile`** switches: `max_hz` (I²C clock, snapped down to the
nearest Zephyr tier) and `auto_inc` (`'implicit'` / `'msb'` for parts that
auto-increment via the sub-address MSB). SMBus PEC (`pec='crc8'`) is not yet applied by firmware
and is rejected at compile.

**`BUS`** names the compile-time default transport. Both profiles ride in
the image, so switching a dual-bus part is `--config bus=spi` at upload or
`nxs set bus` at runtime — never a recompile. A SPI-only part sets
`BUSES = ('spi',)` and declares only `SPI_PROFILE`.

**Cover the whole chip.** `BUSES` lists every bus the *silicon* supports, not
just the pre-soldered Click bus. For a dual-bus part (I²C and SPI)
declare `BUSES = ('i2c', 'spi')` and set `BUS` to the Click's pre-soldered
default (product page / schematic). A silicon-single-bus part (SPI-only)
lists one bus.

**Register address width.** Register opcodes carry a uniform 16-bit
operand, but device-side framing of addresses wider than 8 bits is not yet
implemented — only 8-bit-addressed parts are supported today.

**How to fill each profile (deterministic — never ask).** For every bus in
`BUSES`, read the answer off the datasheet's bus-interface + timing tables and
these fixed board facts:

- **Protocol switches** — `mode`, `addr_bytes`, `rw_read_level`, `dummy_bytes`,
  `auto_inc`: declare each one the datasheet's bus-interface section shows
  differing from the conventional register bus; a conventional part declares
  none. These are mandatory — a wrong protocol clocks garbage, not a slow bus.
- **`max_hz`** — compare the datasheet's max clock for the bus to the board's
  effective default and declare it only when they differ:
  - **SPI** — the firmware baseline is **1 MHz** and the mikroBUS SPI node has
    no DTS ceiling, so a datasheet SPI max above 1 MHz applies directly.
    Declare `SPI_PROFILE(max_hz=<datasheet SPI max>)` to expose the part's real
    speed (e.g. `8_000_000`). Omit only if the SPI max is ≤ 1 MHz.
  - **I²C** — the board bus is **400 kHz** and `max_hz` clamps to
    `min(this, 400 kHz)`. Declare `I2C_PROFILE(max_hz=<datasheet I²C max>)` only
    when the part is *slower* than 400 kHz (some command-response parts at 100 kHz); a part rated
    ≥ 400 kHz needs nothing — the declaration would be a no-op.

Copy the numbers from the datasheet timing tables, never the feature summary
(summaries round up; the timing table is the contract). The datasheet clock is
the silicon's rating — declare it even if a bus is bench-unvalidated on this
Click; if bring-up shows signal-integrity trouble, the cap belongs in the
board configuration, not the driver. Full reference: the driver development
guide's bus configuration section.

### Reset handling — decide from the schematic and the datasheet

The platform pulses the mikroBUS RST line (polarity per
`RESET_ACTIVE`) before every probe, including param reloads. Whether
that pulse reaches the part is a per-Click fact — decide reset
handling from two things you already extracted: does the Click route
mikroBUS RST to the sensor's reset/enable pin (Step 3 schematic), and
does the sensor even have one (Step 4 pin description)?

**1. Click routes RST to the part** → the platform hard-resets it at
every bind. `configure()` issues NO reset of any kind; declare
`RESET_ACTIVE` per the datasheet and start writing config.

**2. RST not routed, or the part has no reset pin** → the platform
pulse does not reach this part; it lives on power-on reset. When the
part documents a software reset (a reset bit or reset command),
ISSUE it: the config-register rewrite that follows covers
configuration state, but only the reset returns the state the
sequence never touches — filter pipelines, internal state machines,
a wedged bus engine — to the documented power-on point, and that
post-reset state is what hardware validation anchors to. Skip the
reset only for a part that documents none. The shape is exactly
this, never mid-sequence:

```python
def configure(self, config):
    # declare_param() calls first, then:
    self.write(RESET_REG, RESET_VALUE)   # FIRST register write, standalone
    self.sleep_ms(<2x the datasheet boot/turn-on time, floor 10>)  # cite it
    # ... all configuration writes AFTER the delay ...
```

- The delay is 2x the datasheet's boot/turn-on maximum with a 10 ms
  floor, cited to the timing table. The maximum comes from the
  datasheet's power-up/start-up table — never from a vendor driver's
  delay constant (a vendor delay proves sufficiency on one bench,
  not the silicon's bound). Under-sleeping loses the writes that
  follow — silently on SPI (no acks), producing stable wrong data
  that is expensive to diagnose.
- If the datasheet or errata says the part resets immediately on the
  RST bit (acks stop mid-transaction), the write itself reports a bus
  error over I2C and the probe fails — that part cannot soft-reset
  from a register driver; drop the reset and rely on the full
  config-register rewrite.

**3. Shared-pin dual-interface parts** (Step 4 auto-detection bullet)
never soft-reset in `configure()` regardless of wiring — every reset
re-arms the interface-detection window mid-session.

Command-protocol parts (`I2cCommandDriver`) whose reset is part of
the protocol keep it, followed by the datasheet's boot delay.

### declare_param() — REQUIRED for every config parameter

Call `self.declare_param()` in `configure()` BEFORE using the parameter.
This registers the parameter in the capabilities descriptor so the host
can discover and modify it at runtime.

Declare ALL user-facing parameters — not just sensor modes but also
`sample_rate`, filter bandwidth, and any other configurable setting
that affects register values.

**Names are canonical across the fleet** — hosts script against them,
so the same concept never gets a synonym. Rate is `sample_rate`;
full-scale is `<quantity>_fs` (`accel_fs`, `gyro_fs`); filter
bandwidth is `<quantity>_bw` per-quantity, or `filter_hz` for a
single joint knob covering all quantities. The same canon applies to
compile-time config keys. Never coin `range`, `fs`, `bandwidth`, or
`<quantity>_range` for a concept that already has a canonical name.

```python
self.declare_param("accel_fs", values=[2, 4, 8, 16], default=8, unit="g")
self.declare_param("sample_rate", values=[10, 50, 100, 250, 500, 1000],
                   default=250, unit="Hz")
```

**Filter bandwidth is a runtime param whenever the part has one.**
Declare the values as physical cutoff frequencies in Hz (never register
codes), map each to its code in a dict, and tag the write. Exclude
codes that change the internal base rate the rate divider divides — a
"bandwidth" setting that silently re-times every declared `sample_rate`
value is a different clock tree, not a filter option. Each such param
must solely own its register byte (see "Two params on one register"
under Traps). Default to the widest in-path bandwidth (the least
filtering that keeps the divider's base rate), so the out-of-box
response matches the part's headline behavior.

**A setting that cannot be a runtime param becomes a compile-time
config key — never a silently fixed value.** Most documented settings
*can* be runtime params: a plain `write` takes `param=`, and a
`write_modify` takes `param=` too (clear the whole field so the OR is
the patch site — see the reserved-bits rule). The setting stays
compile-time only when its write is genuinely value-independent, or a
direct-`write` register is already owned by another param (RMW sites
may share). Expose a compile-time setting as a config key
(`config.get('key', default)`) with the dependent scale or behavior
baked per value, look the value up in a class-level table so an unknown
value fails the compile instead of falling back, and document it in the
module docstring's `Config keys` block. Hardcoding one value and
dropping the choice is a capability reduction, not a simplification.

**A per-instance setting with a uniform projection is still one
knob.** A setting the part instantiates per axis or per channel (a
filter cutoff per axis, a threshold per channel) is exposed as ONE
config key applied uniformly to every instance, offering the values
where the instances' tables coincide. "The encoding is per-instance"
or "the register mixes two quantities" is not an exclusion reason —
project the common value; only a setting with no meaningful uniform
projection may be excluded, with that stated.

### param= tag — REQUIRED on config-dependent register writes

When `self.write()` or `self.set_baud()` writes a value that depends on
the config (sample rate divider, full-scale register value, baud code,
etc.), tag it with `param=("name", value)`. This records the bytecode
offset for runtime patching so parameter changes don't need a re-upload.

```python
self.write(0x21, rate_div,      param=("sample_rate", sample_rate))
self.write(0x22, accel['reg'],  param=("accel_fs", accel_fs))
self.set_baud(baud=baud,        param=("baud", baud))
```

Do NOT tag fixed/constant writes (reset sequences, interrupt enable, etc.).

**`sample_rate` in particular** — three cases:

- **The chip has a rate register** (ODR divider): `declare_param("sample_rate",
  ...)` + `self.write(RATE_REG, rate_div, param=("sample_rate",
  sample_rate))`. The tag owns the rate; writing the divider untagged bakes it
  in permanently.
- **No rate register, fixed-rate data-ready sync**: `declare_param` +
  `drdy_base_hz=<sync Hz>` on the decorator — the compiler paces by
  dividing the sync (see "Trigger handling").
- **A poll-paced part with no rate register** (command-response baros,
  humidity parts): just `declare_param("sample_rate", ...)`. On a
  `trigger="poll"` driver the compiler patches the loop's `SLEEP_MS` interval
  from the declared rate, so `set sample_rate` retunes the poll cadence with no
  register write. Do NOT also tag a write — the poll interval and a rate
  register are two ways to set the same knob, so the compiler rejects a
  `sample_rate` that drives both.

### Trigger handling

**A data-ready output makes DRDY the default — a rule, not a
preference.** Any pin the part drives at its output data rate counts:
a DRDY/INT latch, an ODR sync, even one fixed at the part's internal
rate with no divider. A hardware edge is a drift-free clock; firmware
polling drifts with measure-loop overhead. The mikroBUS `INT` pin
(`mkbus_int`) is wired into the firmware as the DRDY input — every
NXS board can use DRDY out of the box. A fixed-rate sync the loop
can't service edge-for-edge still wins: the loop pace-locks to the
first edge after each pass, so the output cadence stays
hardware-derived instead of drifting with loop overhead.

- **Register-bus parts with a data-ready output**: declare the pin in
  `PINS`, use `@measure_loop(trigger="from_config")`, and enable the
  pin routing in `configure()` per the datasheet's documented
  procedure. User picks `trigger=drdy` or `trigger=poll` at upload;
  default is `drdy`. Hardcoding `trigger="poll"` on a part with a
  data-ready output is a capabilities lie — reserve poll for parts
  with genuinely no such pin.
- **The enable is an evidence question.** Establish from the datasheet
  whether the data-ready output pulses at power-on or must be enabled,
  and act on the citation, never an assumption: active at power-on
  (cite the register-default table or interface section) → no enable
  writes; needs enabling → the documented sequence goes in
  `configure()`, reserved-bit registers via `write_modify`; the
  deciding section exists but cannot be read → that is a gap — stop
  with the NOT EXPRESSIBLE card naming the section. A `PINS`/drdy
  declaration whose pin never pulses is the same capabilities lie as
  hardcoded poll, and it fails louder: a DRDY timeout on every load.
- **Direct reads vs FIFO — run the wire-time budget** (see "On-chip FIFO
  datapath" below) against the fastest declared `sample_rate`. Past the
  budget, the part rewrites its output registers while the burst is still on
  the wire — torn samples, often an all-`0xFF` tail — and one late DRDY
  service drops a sample; the FIFO freezes each sample's bytes and turns
  lateness into backlog.
- **UART sensors**: ALWAYS hardcode `@measure_loop(trigger="poll", sample_rate=1)`
  → the driver sets `config['sample_rate']` and `config['trigger']` internally
- **Register-bus parts with no data-ready output** (and command-response
  parts, where the bus paces each conversion): use
  `@measure_loop(trigger="poll", sample_rate=N)`

**`sample_rate` on a register-less part depends on the sync.** Three
cases:

- **A rate register exists**: declare `sample_rate` unconditionally —
  the tagged register write is a patch site that works under both
  triggers.
- **No rate register, but a fixed-rate data-ready sync** (the pin runs
  at the part's internal rate, no divider): declare `sample_rate`
  unconditionally AND pass the sync rate to the decorator —
  `@measure_loop(trigger="from_config", drdy_base_hz=<sync Hz>)`. The
  compiler paces the drdy loop by dividing the sync at the source
  (`OP_EVENT_DIV`, patched by `set sample_rate`), so delivered spacing
  is exact against the sensor's clock; the same values patch the poll
  loop's sleep. Every declared value must divide the base exactly
  (compile error otherwise) — pick the base's divisors, and cap the
  ladder at what the read burst sustains *end-to-end*: the wire-time
  budget's platform-overhead term (≈ 200 µs per transaction plus staging
  sleeps), not wire arithmetic alone.
- **No rate register, no fixed sync** (command-response parts):
  `sample_rate` exists solely as the poll loop's `SLEEP_MS` patch —
  gate the declaration on the requested trigger:

```python
def configure(self, config):
    if config.get('trigger') == 'poll':
        # Poll cadence only — no register and no divisible sync.
        self.declare_param("sample_rate", values=[10, 25, 50, 100],
                           default=100, unit="Hz")
```

### On-chip FIFO datapath

A FIFO decouples acquisition from the read: the part buffers each frozen
sample, so a read can't tear and a late DRDY service grows the queue instead
of dropping a sample.

**Direct reads vs FIFO is arithmetic, not judgment.** The declared
`sample_rate` values are a contract — every value must hold on every declared
bus — so budget one measure pass on the *slowest* declared bus (I²C for a
dual-bus part) at the *fastest* declared rate:

```
t_pass ≈ 9 × (N + 8) / f_i2c      # status gate + addressing + N-byte burst
t_pass ≈ 8 × (N + 4) / f_spi      # the same pass over SPI
```

`N` is the sample size in bytes. These are **wire terms only** — the
executed pass adds platform overhead: budget ≈ 200 µs per bus
*transaction* (VM dispatch plus bus-stack setup on the target MCU class)
and every staging sleep at face value. A single-burst loop adds one such
term and the wire arithmetic stands; a per-word framed protocol that
reads W words as W transactions is *dominated* by the overhead term —
`t_pass ≈ W × 200 µs + Σ staging sleeps + wire` — and the fastest honest
`sample_rate` is what that end-to-end pass sustains, not what the wire
math promises. Worked: 7 words with 6 × 250 µs staging gaps → ≈ 2.9 ms
per pass → ~340 Hz sustainable → against a fixed 4 kHz sync, declare the
exact divisors up to 250 Hz and stop.

Compare against the sample period with 3× headroom — DRDY wake latency,
other traffic on the shared bus, and the next update landing mid-burst
all eat into it:

- `t_pass ≤ period/3` at the fastest declared rate → status-gated direct
  burst. It stays the default below the budget: lower latency, no FIFO
  state to manage.
- `t_pass > period/3` and the part has a headerless FIFO → the FIFO path is
  REQUIRED — a direct burst past the budget reads the output registers
  while the part rewrites them.
- `t_pass > period/3` and no usable FIFO (absent, or headered) → cap the
  declared `sample_rate` values at the fastest rate that fits the budget. A
  declared rate the bus can't carry is a capabilities lie.

The FIFO relaxes the deadline, not the bus: its own pass must still fit the
full period on the slowest bus, or the queue only grows — cap the rates.
Poll-paced parts run the same check with the whole pass (conversion sleeps
included) against the poll period; with no tear risk, no 3× headroom.

**Phase discipline (I²C).** On parts whose serial engine shares timing with
the internal sample path, a transaction that lands on the sample-update
instant can be NACK'd — some engines also hold SDA for a few milliseconds
after. A DRDY-triggered pass starts just after an update, phase-locked away
from the hazard; poll pacing samples it at random, and even a DRDY loop
de-phases once the pass plus the platform's per-sample work outgrows the
period. The runner absorbs sporadic hits — bus recovery, then an in-place
retry of the faulted op while the FIFO carries the queue — so an isolated
NACK costs milliseconds and zero samples. A NACK rate that *tracks pacing
or rate* is a phase-budget violation: re-check the budget above before
blaming the part.

Worked numbers — the 14-byte packet from the recipe below, on I²C 400 kHz:
t_pass ≈ 9 × 22 / 400 kHz ≈ 500 µs. Declaring rates up to 1000 Hz gives a
333 µs budget → FIFO required (its pass, ~610 µs, fits the 1000 µs period).
Capping the values at 500 Hz gives 667 µs → direct reads hold.

**Hard constraint — headerless FIFOs only.** A `measure()` body has no loops and
reads fixed-length bursts, so it can only consume a FIFO whose frames are a
fixed byte layout — a *headerless* FIFO, where every frame is the enabled
channels in a documented order, same width every time. A part that tags each
frame with a per-frame header/ID byte (so the layout varies at runtime) needs
parsing the DSL can't express: use DRDY direct reads for those. Decide this from
the datasheet before choosing the FIFO path.

**Extract from the datasheet:**

- the **enable mask** register and which channel each bit queues — this fixes
  the packet's contents *and order* (the part packs enabled channels in a
  documented order; that order is your sample layout)
- the **count** register — bytes or samples? — and its **endianness**
- the **control** bits: FIFO enable, FIFO reset/flush, one-shot vs level reset
- **headered vs headerless** (the constraint above)
- **overflow** behaviour (drop-old vs stop) and the FIFO depth

**Recipe.** In `configure()`, after the sensor config, queue the channels then
flush-and-enable in one write:

The `measure()` body compiles to VM bytecode, which resolves only integer
**literals** — write the registers and masks as hex with the datasheet name in a
comment, exactly as below (illustrative FIFO registers):

```python
self.write(0x40, 0xF8)   # FIFO_EN: queue the channels, in packet order
self.write(0x41, 0x44)   # USER_CTRL: FIFO_RST | FIFO_EN — flush stale + enable
```

In `measure()` (`trigger="from_config"`, DRDY), read one packet through a
**tiered count gate** — wait when short, read through a small backlog, resync
only when far behind or torn:

```python
count = self.read(0x42, 2)          # FIFO_COUNTH:L — bytes buffered, big-endian
if count < 14:
    return None                     # frame not complete — wait for the next DRDY
if count > 56:
    self.write(0x41, 0x44)          # >4 frames behind — resync (FIFO_RST | FIFO_EN)
    return None
raw = self.read_burst(0x46, 14)     # FIFO_R_W — atomic packet -> sample buffer
after = self.read(0x42, 2)
left = count - 14
d = after - left
if d == 0:
    return Sample(raw)              # clean read
d = d - 14
if d == 0:
    return Sample(raw)              # one frame arrived mid-read
self.write(0x41, 0x44)             # torn read — drop + resync
return None
```

Why tiered, not flush-on-any-mismatch: a strict `count != 14 → flush` drops good
data on ordinary jitter — a two-frame backlog is two valid samples, not an
error. The gate reads through a bounded backlog (K ≈ 4 frames) and resets only
when the queue runs away or a read tears (the after-count doesn't land on a
frame boundary). Every line is a fixed-length read or an integer compare —
expressible today. The literals (`14` = packet bytes, `56` = K × 14) are the
part's, from the enable mask.

Put every published channel in the FIFO packet so one read is the whole sample
(e.g. keep temperature in the mask rather than a second direct read). If the
datasheet says the first post-enable frame is stale, discard it — an extra armed
read that returns `None`.

**Not yet expressible:** batched *N*-packets-per-wake reads (draining the queue
in one `measure()` call, the path to sustained 1 kHz-to-host) need a bounded
unrolled drain or a drain verb the DSL doesn't have. Track it; don't hand-roll a
loop.

### Output units — SI, canonical field names

**Publish every measurand the part streams.** A channel the part
produces that maps to a canonical SI semantic (an IMU's die
temperature, a combo part's secondary quantity) ships in the output
vector unless the wire-time budget excludes it — and any exclusion is
written down with its reason. A channel dropped because a fact about
it cannot be read (a missing transfer function, an unreadable table)
is a gap-card stop, not an omission.

Use the canonical field name for each quantity and scale the value to
its canonical SI unit. The name sets the semantic; the compiler inherits
the canonical unit from `constants/field_semantics.yaml` (omit `unit`
for these fields — declaring a conflicting one is a compile error), and
a Cyphal host projects the field onto a standard `uavcan.si.sample.*`
subject with no per-driver schema. Vector quantities take `_x/_y/_z`
suffixes.

| Field | Canonical unit | Field | Canonical unit |
|---|---|---|---|
| `accel_*` | `m/s^2` (NOT g) | `voltage` | `V` |
| `gyro_*` | `rad/s` (NOT deg/s) | `current` | `A` |
| `mag_*` | `tesla` | `distance` | `m` |
| `temp` | `kelvin` (NOT celsius — fold +273.15 into the offset) | `force` | `N` |
| `pressure` | `pascal` | `frequency` | `Hz` |
| `angle` | `rad` | `luminance` | `cd/m^2` |
| `mass` | `kg` | `torque` | `N*m` |
| `speed` | `m/s` | `flow` | `m^3/s` |

A name outside this set is *generic*: it still ships in `RawSample` with
its descriptor and its authored `unit`, but gets no typed SI subject.
Pick the closest canonical name and scale to SI; do the conversion in
the driver, not on the host.

### measure() — AST-compiled patterns

Only these Python patterns are supported in `measure()`:

| Pattern | SPI/I2C | UART |
|---|---|---|
| `var = self.read(reg)` | Read register → VM reg | — |
| `var = self.read(reg, width[, signed=, endian=])` | Read `width` bytes → VM reg; `signed=True` for int8..int32, `endian="little"` for LE parts (default unsigned BE) | — |
| `var = self.read_analog(0)` | Read AN pad (ADC) → VM reg (ch 0 only) | Read AN pad → VM reg |
| `raw = self.read_burst(reg, N[, into=off])` | Burst read → sample buf at `into` (default 0); a second bank needs an explicit `into=` | — |
| `self.write(reg, val)` | Write register | — |
| `self.send_command(byte)` | Raw command byte (command-response parts) | — |
| `x = self.xfer(word[, width])` | Clock a literal full-duplex SPI word (`BUSES = ('spi',)` only); response → VM reg, unsigned MSB-first. Statement position discards it (pipeline priming) | — |
| `self.sleep_ms(n)` | Conversion / settle delay | — |
| `self.sleep_us(n)` | Sub-millisecond settle / inter-word gap | — |
| `avail = self.available()` | — | RX bytes available → VM reg |
| `m = self.match(b0, b1, …)` | Mismatch count of `sample_buf[0..N)` vs constant bytes (0 = pass) | Same |
| `bad = self.verify_checksum(ChecksumFletcher(), start, len, ck_off)` | Recompute Fletcher over `sample_buf[start..start+len)`, count mismatches vs the received bytes at `ck_off` | Same |
| `raw = self.read(N)` | — | Read N bytes → sample buf |
| `y = a <op> b` | Integer arithmetic — `+ - * // << >> & \| ^` (see compensation) | — |
| `if not (var & MASK): return None` | Skip if bit not set | Skip if no data |
| `if var <op> const:` / `else:` / `elif` | Branch — `== != < > <= >=`; `if`/`elif`/`else` all supported | — |
| `return Sample(raw)` | Commit the raw sample buffer | Commit sample |
| `return Sample(field=value, …)` | Commit *computed* values to declared fields | — |
| `return None` | Skip iteration | Skip iteration |

**Value-reads vs buffer-placement.** `read`, `read(reg, width)` and
`read_analog` return a *value* into a VM register — they stage their bytes off
the sample buffer, so they never overwrite sample data. `read_burst` /
`read_words` / UART `read(N)` *place* the sample bytes at buffer offset 0. So a
positional `return Sample(raw)` — which commits the buffer as-is — is only valid
after a buffer-placing read; a value-read is published through a named field
(`return Sample(field=var)`).

**Constants in `measure()` are literals or UPPER_CASE class attributes.**
Register addresses and masks are written as literals (`0x28`, `0x08`);
computed wire words live as UPPER_CASE class-level integer constants
(`self.CMD_POSITION`) — those resolve at compile time, and anything
else on `self.*` is rejected. The one runtime exception is a
**coefficient bound in `configure()`** (`self.c1`, set via
`self.read(reg, width)`): those *are* referenced by name in `measure()`
arithmetic, because the `self.` binding is what persists them into the
loop. See the compensation section above.
**UART avail mask MUST be `0xFFFF`** (not `0xFF`).
**Max sample size is 128 bytes** (`set_sample_size(N)` and
`read_burst`/`uart.read` counts); a driver that also uses a value-read
(`read(reg, width)`, `read_analog`, `xfer`) caps at 124 — the last 4
bytes are the value-read staging slot. Stay ≤ 110 when the sample must
be readable over I²C: the register-map sample record spends 18 of the
window's 128 bytes on its header.

## Traps that compile but mislead

Datasheet transcriptions that read as correct but compile to the wrong thing.
Each is a `CompileError`; the fix follows.

**Runtime logic in `configure()`.** `configure()` is traced at compile time
against mock reads, so a value read there is a placeholder, not the live
register. A conditional or read-modify-write on it bakes the mock's branch:

```python
def configure(self, config):
    if self.read(CTRL) & 0x40:      # CompileError: the value is a mock
        self.write(CTRL, 0x01)
```

Runtime *branching* belongs in `measure()`, where reads execute on-device.
In `configure()` write the full register value directly — or, when the
value must preserve on-device bits, use `self.write_modify(reg,
set_bits=…, clear_bits=…)`, whose read happens on-device at load.

**Two params on one register.** Each param patches one register; two params
writing the same register overwrite each other at load, and each bakes the
other's compile-time bits:

```python
self.write(CTRL1, odr, param=("sample_rate", sr))   # CompileError:
self.write(CTRL1, fs,  param=("range", rng))         # two params, one register
```

Compose the register from one param — `self.write(CTRL1, odr | fs,
param=("sample_rate", sr))` — and leave the other field fixed, or give each
its own register.

**A param tagged at more than `MAX_PATCH_SITES` sites.** A param may patch up
to `MAX_PATCH_SITES` bytecode sites (a field split across two registers), no
more. A rate on a single register is one site — don't spread it across two
writes to the same register.

**A value-dependent register layout.** A patched param must emit the same set
of patch sites, at the same offsets, for every value; a conditional write
before the tagged write moves the sites and is rejected.

**`declare_param` after the tag.** Declare a param before any `param=("name",
…)` tag — the tag needs the declared value set — and the tagged value must be
one of the declared values.

**A live param nothing reads.** `kind="live"` takes effect only through the PWM
pair (`pwm_freq`/`pwm_duty`) or an output field's `scale_param`. Any other live
param is a no-op `set`; use `kind="reload"`.

**A `scale_param` holding register codes.** A `scale_param` value multiplies
the field's scale, so its values are physical magnitudes (`2, 4, 8, 16` g),
never register codes (`0, 1, 2, 3`) — a `0` dead-zeroes the field.

**A patch value too wide for its site.** A tagged `write` patches one byte,
`set_baud` four, `stage(patch=…, size=n)` exactly `n`; every value of the param
must fit that width.

**A FRAME part's identity word.** `WHO_AM_I_VALUES` on a FRAME driver is
compared against the frame's data field on-device (not just in `probe()`), so
it must be the full field-width value (a CRC-framed part's is `0x1234`, not
the high byte `0x12`).

## Step 6: Self-Check

Compile first — the installed `nxs` package is the compiler; no device is
needed. One invocation per declared bus:

```bash
python -c "from nxs.drivers.<name> import <Class>; \
           print(len(<Class>().compile({'bus': 'i2c'}).bytecode))"
python -c "from nxs.drivers.<name> import <Class>; \
           print(len(<Class>().compile({'bus': 'spi'}).bytecode))"
```

A `CompileError` names the offending construct — fix the driver and
re-compile until every declared bus passes. The compiler sweeps every
declared param value, so a passing compile covers the whole value set, not
just the defaults. Then check the semantics the compiler can't see:

- **Class**: subclasses `RegisterDriver` (I²C/SPI) or `StreamDriver`
  (UART). Never plain `SensorDriver`.
- **Naming**: file = the part number in lower_snake, class = the same
  in PascalCase (`xyz9000.py` / `Xyz9000`), consistent with the
  sibling drivers in the package; never the carrier board's marketing
  name.
- **Class checks**: every `references/<class>.md` read in Step 2 has
  its Self-checks section applied — a combo part applies every
  matching file's checks.
- **Companions**: a package with a second co-resident I²C slave
  declares it in `I2C_COMPANIONS` (own identity anchor or skip
  reason), reaches it only via `dev=`, and publishes a fused sample
  spanning every die — never a per-die driver split.
- **Probe anchors**: `WHO_AM_I_REG` and `WHO_AM_I_VALUES` are
  declared as class attributes for I²C/SPI register drivers. For
  drivers whose `BUSES` includes `'i2c'`, also declare `I2C_ADDRS`
  with every strapped address the part can land on — NXS scans
  these on load. SPI-only drivers (`BUSES = ('spi',)`) don't need
  `I2C_ADDRS`; the default of `[]` means no I²C scan happens, which
  is correct.
- **`__init__`**: seeds `self._read_responses` with WHO_AM_I (and any
  other register reads inside `probe()`) so the host-side tracer can
  clear the probe asserts.
- **`probe()`**: only reads registers + asserts. No writes, no loops.
- **`configure(config)`**: calls `declare_param(...)` for every key
  the user can tune — **including `sample_rate`** (register-less
  parts: only in the poll branch — see "Trigger handling") — before
  consuming it. Every config-dependent `self.write(...)` /
  `self.set_baud(...)` carries a matching `param=("name", value)`
  tuple. Fixed writes (reset, interrupt-enable) do not.
- **Trigger**: a part with any data-ready output — including a
  fixed-rate sync — declares it in `PINS`, uses
  `trigger="from_config"`, and enables the pin routing in
  `configure()`; hardcoded `poll` appears only on parts with no such
  pin or with bus-paced protocols. A drdy default carries either the
  enable sequence or a comment citing the datasheet fact that the
  output is active at power-on — one of the two must be in the driver.
- **UART framing** (`StreamDriver`): a fixed-length record read with
  `read_n` commits with `store_sample()`, which publishes the
  compile-time `set_sample_size`. Only a variable-length record
  captured by `read_until` uses `store_sample_n()`. `store_sample_n()`
  after `read_n` is a bug — it publishes the `__cursor` scratch
  register that an intervening `match`/`verify_checksum` may overwrite,
  so the device reports a wrong per-sample size.
- **Capability parity**: every user-relevant setting the datasheet
  documents (rate, ranges, filter bandwidth, operating modes) is a
  declared param, a documented config key, or a fixed write with a
  one-line stated reason; every streamed measurand with an SI semantic
  is an output field or its exclusion is stated; a per-bus clock
  ceiling above the firmware baseline is declared in a profile. A
  capability the datasheet documents that the driver silently lacks
  fails this check.
- **Value tables are consumed whole.** When a datasheet table backs a
  param or config key (full-scale codes, bandwidth codes), the driver
  exposes every row, or names the rows it dropped and why. Count the
  rows you extracted against the table's row count — a table that
  lost rows between the datasheet and the driver is a silent
  reduction, and a table you could only partially read is a gap-card
  stop, not a shorter table.
- **Hygiene**: the docstring's pin map names mikroBUS signals, never
  MCU peripheral instances; class constants and tables are consumed by
  probe/configure/measure — no dead declarations.
- **Docstring accuracy**: the docstring's `Config keys` block matches
  the declared params and config keys **verbatim** — names, value
  sets, defaults. Re-read it against the final `declare_param` calls
  and `config.get` defaults before finishing; a block describing an
  earlier draft's values is a lie to the user.
- **Computed wire words** (`xfer` drivers): every command word is an
  UPPER_CASE class constant cross-checked by a module-level `assert`
  against the datasheet's worked example words.
- **`set_output([...])`** declares each output field with a numeric
  `scale` that lands the value in the semantic's canonical SI unit
  (`m/s^2`, `rad/s`, `tesla`, `kelvin`, `pascal`); the unit itself is
  inherited — write `unit` only for non-SI fields (`%RH`, NMEA,
  generic). No `g`, `dps`, `celsius`, `Fahrenheit`.
- **`set_sample_size(N)`** matches the total bytes returned by the
  measure loop's `read_burst` / `read`. N ≤ 128 (≤ 124 when the loop
  also uses a value-read — see the staging-slot rule in the table
  section; ≤ 110 to stay readable through the I²C sample record).
- **`@measure_loop(...)`**: only the patterns listed in the table
  above. Register addresses are literals (not `self.WHO_AM_I_REG`);
  computed wire words are UPPER_CASE class constants (`self.CMD_X`).
  UART availability check uses `& 0xFFFF`.
- **Data path**: the wire-time budget ("On-chip FIFO datapath") holds
  for the fastest declared `sample_rate` on the slowest declared bus —
  direct reads within budget, FIFO past it, rates capped when neither
  fits.
- **Communication profile**: if the chip's bus protocol deviates from
  the conventional register bus (non-standard SPI address phase / R/W
  polarity / mode, a slower-than-bus clock, sub-address-MSB auto-increment, SMBus
  PEC), declare `SPI_PROFILE` / `I2C_PROFILE` with only the deviating
  switches. A conventional part declares none. See "Communication
  Profile layer" above.
- **Reset polarity**: if the datasheet's RST pin is active-high, declare
  `RESET_ACTIVE = 'high'`. Omit it (active-low default) otherwise.
- **No imports beyond** the documented surface: `from nxs import …`
  the base your driver needs (`RegisterDriver` / `StreamDriver` /
  `I2cCommandDriver` / `SensorDriver` / `Sample`) plus any communication
  profile you declare (`SpiProfile` / `I2cProfile` / `UartProfile`); and,
  only for framed transports, `from nxs.framing import Crc, I2cFrame,
  SpiFrame` and `from nxs.compiler import ChecksumFletcher,
  ChecksumPolynomial, ChecksumXorFold`. Nothing else (no `math`, no
  `struct`).

## Step 7: Report

When the driver is written and self-checked, output **exactly the card below**
— filled from the driver you created — and nothing else. No preamble, no
narrative, no at-rest values, no troubleshooting, no commentary. The structure
is identical on every run.

```
nxs driver: <name>

  file      <driver path>
  bus       <default> (default)[ · <other>]
  trigger   <trigger>[ · <rate> Hz]
  outputs   <field> (<unit>)[ · <field> (<unit>) …]
  params    <name>=<default>[ · <name>=<default> …]

  nxs upload <name>[ --config bus=<other>]
  nxs stream --hz <rate>
  nxs set <param> <value>
  nxs store save 0
```

Fill from the driver: `<driver path>` is the Step 5 directory + `<name>.py`;
group axis fields (`accel_x/y/z`) under one unit; `<rate>` is the integer
sample rate. A single-bus part shows only `bus  <the bus>` and drops the
`--config bus=` hint. Default transport is I²C; append `-t cyphal-serial -p
<port>` (or `-t cyphal-can -p <iface>`) to the commands only if the user asked
for it. Add no lines beyond the card.

**Multi-sensor (suite).** For a host that manages several NXS units
declaratively, also show the suite.yaml form — the new driver becomes a
`sensors:` entry under its unit, and `nxs suite switch` deploys it:

```yaml
units:
  - name: <unit-role>
    module: nxs
    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x30}]
    sensors:
      - driver: <sensor_name_lowercase>
        config: {sample_rate: 100, <other_key>: <value>}
```

The switch converges every declared unit and is idempotent:

```bash
nxs suite switch
```

After live-tuning params on a declared unit (`nxs --unit <name> set ...`),
`nxs suite freeze --unit <name>` adopts the tuned values back into the
manifest. The manifest schema, switch semantics, and the tune-then-freeze
flow are in the Integration & Operation Manual §5
(`docs/specs/nxs-integration-manual.md`).

When the part is not expressible (the stop rule above), output exactly
this card instead — one `missing` block per gap — write no driver file,
and end the run:

```
nxs driver: <name> — NOT EXPRESSIBLE

  part      <chip> — <bus/protocol in one line>
  needs     <the wire behavior the datasheet requires>
  missing   <the DSL construct that does not exist; quote the exact
             CompileError when one was raised>
  closest   <the nearest documented construct and why it falls short>
  unblock   <the smallest DSL extension that would cover this part>
```
