---
name: nxs-generate-sensor-personality
description: >
  Generate a new NXS sensor personality from a datasheet or mikroE Click board.
  Use when a user plugs in a new sensor, provides a datasheet PDF/URL,
  or names a sensor IC. Creates a complete Python SensorDriver subclass
  with probe(), configure(), and @measure_loop methods.
allowed-tools: Read WebFetch WebSearch Write Bash Glob Grep
argument-hint: [sensor-name-or-click-board-url]
---

# Generate NXS Sensor Personality

You are creating a sensor personality for the NXS sensor VM. The personality is
a datasheet in code: a Python class that describes every register,
mode, and scale factor of the sensor IC. The YAML config then selects
which mode to use at deployment time.

**Plug-and-play — resolve, don't ask.** Every choice is a datasheet fact
(registers, modes, scales, WHO_AM_I, bus protocol, clock ceilings) or a fixed
NXS-board fact (mikroBUS I²C Fast-Mode tier, 330 kHz on the wire; SPI 1 MHz baseline, no DTS ceiling;
RST/INT/AN/PWM on the mikroBUS socket). Resolve them and emit a complete
personality: declare every bus the silicon supports plus the datasheet clocks,
default `BUS` to the pre-soldered transport, and leave each a knob (`BUSES`,
`BUS`, profiles, params; the params land in the sibling `<name>.yaml`
descriptor, and every generated personality is a `.py` + `.yaml` pair). Ask
only when a *required* fact is absent from the datasheet or Click
materials, never to pick between a default and its alternative.

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
from your copy, name the section in the card and stop — a personality built
on an assumed default ships a lie that only surfaces on hardware. A
part whose datasheet cannot be retrieved **at all** is not generatable:
the Click sources are wire-protocol-normative, never a capability
source, so stop with the card instead of shipping a personality with
silently reduced capabilities (dropped fields, guessed clock ceilings,
omitted parameters).

## Step 1: Read the Reference Materials

**ALWAYS start here.** Read the personality authoring reference to understand
the exact APIs, patterns, and constraints — whichever of these exists in
your checkout:

```
docs/released/specs/nxs-personality-authoring.md    # inside the firmware repository
docs/nxs-personality-authoring.md          # standalone SDK checkout / release bundle
```

That guide is the single source of truth for what a personality can do
and how `RegisterDriver` / `StreamDriver` behave. The templates and
rules in this skill summarise it, but when the two disagree, the
guide wins.

**You will not see the compiler source** — that's deliberate. Write
against the documented surface, not against any imagined internal
behaviour. You do run the compiler as a black box: Step 6 compiles
the personality per declared bus, and a `CompileError` is the feedback
loop.

## Step 2: Identify the Sensor and Check Compatibility

From the user's input ($ARGUMENTS), identify:
- The sensor IC (part number from the datasheet or Click silkscreen)
- Communication protocol: **I2C, SPI, or UART**
- If a mikroE Click board URL is given, fetch it to identify the IC

**3.3V only.** NXS provides 3.3V on the mikroBUS power rail.

**A camera is a camera personality.** A CSI-2 image sensor (it streams
video over MIPI behind a serializer; it is not a mikroBUS part) takes
the camera skill: run `/nxs-generate-camera-personality` and stop here.
The rest of this skill is for mikroBUS parts only.

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
still enter the personality only from the documents in front of you; a
part that contradicts its family prior wins, stated in a comment.

## Step 3: Get the mikroE C Driver and Pin Mapping

Search for the Click board driver on GitHub:

```
https://github.com/MikroElektronika/mikrosdk_click_v2/tree/master/clicks/<click-name>
```

Read: `lib_<name>/include/<name>.h` (register map, pin defs) and
`lib_<name>/src/<name>.c` (init sequence, read functions).

The Click driver is normative for the **wire protocol** (framing, CRC
parameters, register addresses, strap options), never for
**capability**: vendor examples skip reset, rate programming, filters,
FIFO, and interrupts, so the feature bar comes from the datasheet
filtered through this skill's rules (reset handling, trigger handling,
datapath budget, runtime params). Vendor **delays** are an upper bound,
not a spec: when the datasheet documents the transaction timing (a
post-edge delay in the trigger section, a response-preparation time in
the read-timing diagram), the datasheet's numbers win. A table
elsewhere showing frames back-to-back (self-test, BIST, a command
sequence) does not license dropping a pipelined read's staging settle;
that is a different transaction (see `inter_frame_sleep_ms`).

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
  reset exit, no personality attribute is needed — the platform floats
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
- **Data path**: one burst from the data registers per sample. The
  wire-time budget under "Data path" decides which `sample_rate` values
  you may declare, from the sample size and the slowest declared bus
  clock. Extract its one datasheet input now: whether the part holds a
  **coherent output set** while a burst is on the wire (shadowed or
  latched output registers, block data update, "a burst read returns one
  sampling instant")
- **For UART sensors**: default baud rate, command protocol

**Timing declarations from the datasheet.** If the part is a stream receiver whose message carries a solution computed earlier (GNSS), call `self.stamp_frame()` at the measure loop's frame-sync point — it stamps the raw first-byte arrival. Event-sampled parts need nothing — the delivered DRDY edge stamps them automatically.

## Step 5: Write the Personality Pair

A personality is a pair in a directory named after the sensor, written where the
user works — never into the installed package (a customer's site-packages
is wiped by the next reinstall):

```
./<name>/<name>.py     codegen and physics (this step)
./<name>/<name>.yaml   facts — the params table (Step 5b)
```

`<name>` is the snake_case sensor name, also the YAML `meta.driver` value.
`nxs upload ./<name>/<name>.py` compiles the pair from there; `nxs personality
install ./<name>` copies it into the personality store so `personality: <name>` in
suite.yaml resolves to it. Create the directory, then write `<name>.py`.

The family bases live in the installed package (`nxs.drivers`); find its
path from the package rather than hardcoding it:

```bash
python -c "import nxs.drivers, os; print(os.path.dirname(nxs.drivers.__file__))"
```

**Check for a family base first.** An underscore-prefixed module in the
same directory (`_ubx_nav_pvt.py`) is a personality-family base: the shared
epoch message, measure loop, output table, and framing helpers for every
part that speaks that protocol. List them before writing anything:

```bash
python -c "import nxs.drivers, os; d=os.path.dirname(nxs.drivers.__file__); print([f for f in os.listdir(d) if f.startswith('_') and not f.startswith('__')])"
```

Read the base's docstring: it names the family and states what a
subclass still owns. When the part belongs to that family, subclass it
and implement only the part-specific surface — probe expectations,
configuration keys, rate tables, protocol variants — instead of
re-emitting the shared machinery. A standalone personality for a part the
base already covers silently detaches from every later fix to the base,
which is the failure this check exists to prevent.

When a part is the **second** member of a family whose first member is
still standalone, extract the shared machinery into a new `_family.py`
base and make both personalities subclass it, rather than copying the first
personality. Two concrete implementations are the threshold for extracting a
base — one is not.

**Name the personality after the part, never the carrier board.** The file
and class carry the part number of the silicon or module whose
datasheet defines the wire protocol — the name on the customer's BOM —
not the Click board's marketing name (a carrier is one of many homes
for the same part; a module product like a GNSS receiver uses the
module part number, which is its datasheet-bearing part). File: the
part number in lower_snake (`xyz9000.py`); class: the same in
PascalCase with digits kept (`class Xyz9000`), matching the sibling
personalities already in the package.

### I2C/SPI (Register-Based) Sensor Template

```python
"""
<SENSOR> personality for NXS VM.

<Vendor> <PART> <measurands>; datasheet <document id, revision>.
Bus: I2C 0x__/0x__ (<strap pin>), default; SPI mode <n>, <f> MHz max.
Config: sample_rate <v>|<v>|<v> Hz (<default>); range <v>|<v>|<v> g (<default>);
    trigger drdy|poll (drdy); bus i2c|spi (i2c).
Outputs: accel_x/y/z (m/s^2), temp (K).
Pins: INT -> <signal> (data ready); RST -> <signal> (active low).
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
    # Every address the <strap pin> strap selects; scanned on load, the
    # first WHO_AM_I match is latched.
    I2C_ADDRS = [0x__, 0x__]

    # ── Reset polarity (optional) ───────────────────────
    # Only when the datasheet's RST pin description says reset asserts
    # HIGH (DS <pin table>); omit for the active-low default.
    RESET_ACTIVE = 'high'

    # ── Communication profile (bus interface, optional) ──
    # One profile per bus, only where the wire protocol deviates from the
    # conventional register bus; clocks copied from the DS timing table.
    #   SPI_PROFILE = SpiProfile(addr_bytes=2, rw_read_level=0,
    #                            max_hz=1_000_000)   # DS SPI timing table
    #   I2C_PROFILE = I2cProfile(max_hz=400_000)     # DS I2C timing table
    #   BUS = 'i2c'                                  # compile-time default

    # ── Configuration lookup tables ─────────────────────
    # <REG> <field>: every documented range (DS Table __). The base scale
    # is per unit of range; scale_param multiplies it by the live range.
    ACCEL_FS = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}   # g
    ACCEL_BASE_SCALE = 9.80665 / 32768.0               # m/s^2 per LSB, per g

    def __init__(self):
        super().__init__()
        self._read_responses = {
            self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who in self.WHO_AM_I_VALUES

    def configure(self, config):
        # 1. Declare every runtime param from the sibling <name>.yaml.
        self.declare_params_from_descriptor()

        # 2. Read config with defaults.
        sample_rate = config.get('sample_rate', 250)
        range_val   = config.get('range', 8)
        bw_val      = config.get('bandwidth', 200)

        # 3. <RATE_REG> divides the <base> Hz base rate: div = base/rate - 1.
        rate_div  = max(0, min(255, 1000 // sample_rate - 1))

        # 4. Register writes; every config-dependent write carries param=.
        self.write(0x21, rate_div,                        # <RATE_REG>
                       param=("sample_rate", sample_rate))
        self.write(0x22, self.RANGE_TABLE[range_val],     # <RANGE_REG>
                       param=("range", range_val))
        self.write(0x38, 0x01)             # <INT_REG>: enable DRDY

        # 5. Outputs: base scale x scale_param tracks the live range; a
        #    fixed range bakes one scale. SI fields inherit their unit.
        self.set_output([
            {'name': 'accel_x', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'range'},
            {'name': 'accel_y', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'range'},
            {'name': 'accel_z', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'range'},
        ])
        self.set_sample_size(6)  # 3 x int16

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x27)       # STATUS_REG
        if not (status & 0x08):            # DRDY bit
            return None
        raw = self.read_burst(0x28, 6) # DATA_OUT
        return Sample(raw)
```

### Docstring and comments

The module docstring is the title line, a blank line, and at most eight
lines, each a fact the user needs: the part and its datasheet revision;
the bus with its address or chip select; every config key and param
with its values, unit, and default; every output with its unit; the
mikroBUS pins used. Nothing else. No data-path, integrity, or
capability-parity paragraphs. A dropped table row or an unpublished
measurand is stated in one line at the table or the output vector, with
its reason.

Inline comments are one or two lines. A register write cites the
datasheet register or section. Comment nothing else unless it states a
limit or an ordering rule a reader would otherwise get wrong. A timing
value cites the datasheet section that gives it, or stands alone. No
issue numbers, dates, bench names, first person, history, or repository
paths; no dashes for punch, no rhetorical contrasts.

Template notes:

- `I2C_ADDRS` models one die's strap alternatives. A part with more than
  8 address options omits the list and declares `i2c_addr` as a param
  (the param path skips the scan). A second co-resident die at its own
  address goes in `I2C_COMPANIONS` (see "Companion I²C devices").
- Import the profile types you declare: `from nxs import ..., SpiProfile,
  I2cProfile`. Clock ceilings are copied from the datasheet's timing
  table, never recalled or rounded (see "Communication Profile layer").
- Every documented mode goes in a lookup table. A runtime full-scale
  keeps the register code in the table and a base scale per unit of
  range on the field; a fixed full-scale bakes one scale and no
  `scale_param` (`ACCEL = {'scale': 9.80665 / 2048.0}`).
- SI at the source: a field with an SI semantic publishes its canonical
  unit (see "Output units") and declares no `unit`; temperature folds
  the Celsius zero into the offset (+273.15). Only non-SI fields
  (humidity `%RH`, NMEA, generic) carry an authored `unit`.
- `sample_rate` is always declared; it is a runtime param like any
  other, never a bytecode constant. A software reset, when the part
  needs one, is the first register write (see "Reset handling").

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

### Command-response I²C (opcodes instead of a register map)

**When to use**: the datasheet describes the protocol as "commands"
rather than a register map: a table of opcodes (1 or 2 bytes each), a
prescribed conversion delay between the command and the read, and a
read opcode that returns the result (a barometer's CONVERT / ADC READ
pair). The measure loop is written, as on a register driver: send the
command, sleep the conversion time, read the result behind its read
opcode.

```python
from nxs import I2cCommandDriver, Sample, I2cProfile


class <SensorName>(I2cCommandDriver):
    """<Sensor> I²C command-response personality."""

    PINS = {}   # command-paced parts of this family don't use mkbus_int

    # I²C address(es). NXS auto-detects by scanning + trying the
    # first command; no WHO_AM_I register exists on these parts.
    I2C_ADDRS = [0x__]
    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = []  # empty list = skip WHO_AM_I probe

    # ── Communication profile (optional) ────────────────
    # Declared only when the part caps below the Fast-Mode bus; the bus is
    # clamped down on load (DS I2C timing table).
    I2C_PROFILE = I2cProfile(max_hz=100_000)

    # Command set, from the datasheet's command table.
    CMD_CONVERT = 0x__   # start a conversion
    CMD_READ = 0x__      # the read opcode the result answers behind

    def probe(self):
        pass  # no WHO_AM_I; bus ACK on first command is enough

    def configure(self, config):
        self.declare_params_from_descriptor()  # sample_rate 1|2|5|10 Hz

        # Scales from the DS conversion formulas, in canonical SI.
        self.set_output([
            {'name': '<field1>', 'type': 'uint16', 'byte_order': 'big',
             'scale': <scale>, 'offset': <offset>},
            ...
        ])
        self.set_sample_size(<bytes>)

    @I2cCommandDriver.measure_loop(trigger="poll", sample_rate=10)
    def measure(self):
        self.send_command(self.CMD_CONVERT)
        self.sleep_ms(<conversion_ms>)     # the datasheet's max conversion time
        raw = self.read(self.CMD_READ, <bytes>)
        return Sample(<field1>=raw)
```

### Critical rules for `I2cCommandDriver`

- **The conversion sleep is a hard lower bound.** Sleep the datasheet's
  max conversion time plus a small margin (8.3 ms max → `sleep_ms(10)`).
  Under-specifying corrupts every sample; over-specifying just costs
  latency. A part whose precision mode changes the conversion time
  declares one measure variant per mode (`when=`).
- **The read goes behind a read opcode.** `read(opcode, n)` sends the
  opcode byte and reads `n` bytes; a part that answers a bare read
  transaction with no opcode, or frames each word with a CRC byte
  (a word-plus-CRC framing, `[data, data, crc]`), is a gap card: this family
  neither reads without a command byte nor verifies a CRC.
- **Response size caps at 128 bytes** (sample-buffer limit).

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
    """<Sensor> baro, on-device int64 compensation."""

    PINS = {}                       # polled; no DRDY line
    I2C_ADDRS = [0x76, 0x77]        # CSB strap selects the address LSB
    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = []            # empty: no WHO_AM_I probe
    WHO_AM_I_SKIP_REASON = ("no identity register; the reset's PROM reload "
                            "and the coefficient reads in configure() are the presence test")

    CMD_RESET = 0x1E                # reloads the PROM into the chip
    PROM_C1   = 0xA2                # C1..CN at 0xA2, 0xA4, ... (16-bit big-endian)

    def probe(self):
        self.send_command(self.CMD_RESET)
        self.sleep_ms(3)

    def configure(self, config):
        self.declare_param("sample_rate", values=[1, 2, 5, 10, 25],
                           default=10, unit="Hz")

        # Factory coefficients, read once and bound to self so they
        # persist into measure().
        self.c1 = self.read(self.PROM_C1 + 0, 2)
        self.c2 = self.read(self.PROM_C1 + 2, 2)
        # ... through cN at PROM_C1 + 2*(N-1) ...

        # Polynomial output: pressure in Pa (scale 1.0); temperature in
        # centi-degC (scale 0.01, offset 273.15 to kelvin).
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
  same as every personality. The *only* `self.*` references allowed in
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

    # SPI-only silicon (no I2C variant), so no I2C_ADDRS.
    BUSES = ('spi',)

    PINS = {
        'drdy': 'mkbus_int',         # Data ready (input, optional)
    }

    # A part with no WHO_AM_I anchors on a FIXED_VALUE register that
    # always reads one constant (DS <section>); the full field is compared.
    WHO_AM_I_REG = 0x__              # FIXED_VALUE register address
    WHO_AM_I_VALUES = [0x____]       # the fixed value

    # ── Communication profile (clock + mode) ────────────
    # The FRAME composes the words, so the profile carries only the SCLK
    # ceiling and mode (DS SPI timing table).
    SPI_PROFILE = SpiProfile(max_hz=8_000_000, mode=0)

    # ── SPI frame (DS <section>, <figure>) ─────────────
    # <word layout>. CRC-8 poly 0x__, init 0x__, over <fields>; worked
    # examples <word> -> <crc>. Response to request K clocks out in frame K+1.
    FRAME = SpiFrame(
        width=32,
        fields=[('rw', 1), ('addr', 5), ('rs', 2), ('data', 16), ('crc', 8)],
        crc=Crc(width=8, poly=0x__, init=0x__, xor_out=0x__,
                covers=('rw', 'addr', 'rs', 'data'),
                feedback_style='standard'),
        read_pipeline=1,
        inter_frame_sleep_us=250,     # one <f> kHz internal update with 2x margin
        status_ok=('rs', 0b__),       # DS <status table>: <code> = success
    )

    def __init__(self):
        super().__init__()
        # Trace-time seed so probe()'s assert clears at compile time.
        self._read_responses = {
            self.WHO_AM_I_REG: [0x____],
        }

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who == 0x____            # full 16-bit fixed value

    def configure(self, config):
        # Declared unconditionally with a rate register (tagged divider
        # write); with none, only in the poll branch (see "Trigger handling").
        self.declare_param("sample_rate", values=[100, 250, 500],
                           default=250, unit="Hz")

        # Reset per the datasheet; write() composes the full FRAME.
        self.write(0x__, 0x____)        # <RESET_REG>: reset
        self.sleep_ms(__)               # 2x DS <start-up time>

        # ── DRDY / ODR pin routing (DS <section>) ───────────────
        # The documented procedure, verbatim: fixed unlock words are plain
        # writes; a register with reserved bits is write_modify.
        #     self.write(MODE_REG, UNLOCK_TCODE_1)
        #     self.write(MODE_REG, UNLOCK_TCODE_2)
        #     self.write(BANK_SELECT_REG, ROUTING_BANK)
        #     self.write_modify(ROUTE_REG, set_bits=ROUTE_BIT)
        #     self.write(BANK_SELECT_REG, 0x0000)   # back to bank 0

        # Runtime params in other banks: the same bank walk, each
        # config-dependent write tagged with param=.
        #     self.write(BANK_SELECT_REG, BANK_N)
        #     self.write(BANK_N_REG_X, value, param=("name", val))
        #     self.write(BANK_SELECT_REG, 0x0000)

        self.set_output([
            {'name': '<field1>', 'scale': <scale>, 'unit': '<unit>'},
            ...
        ])
        self.set_sample_size(<num_words> * 2)   # 2 data bytes per word

    @RegisterDriver.measure_loop(trigger="from_config", sample_rate=250)
    def measure(self):
        raw = self.read_words(<start_reg>, <num_words>)   # <first reg>..<last reg>
        return Sample(raw)
```

### Critical rules for FRAME-based SPI drivers

- **Default to DRDY mode.** If the part has an ODR / DRDY output pin,
  including a fixed-rate sync with no divider, configure the routing
  in `configure()` per the datasheet's documented procedure and use
  `trigger="from_config"` on the measure_loop decorator (see "Trigger
  handling"). The chip-side routing only matters in DRDY mode; `poll`
  ignores the pulses, so configuring it under both is harmless.
  Hardcoding `trigger="poll"` on a part with a data-ready output is a
  capabilities lie; reserve poll for parts with genuinely no such pin.
- **Declare every integrity field the protocol defines.** `fields` is
  MSB-first within the word and the widths sum to `width`; `crc.covers`
  lists the covered fields in declaration order; `status_ok=(field,
  value)` is the return-status field and its success code from the
  protocol chapter's status truth table. A declared `crc` / `status_ok`
  makes the compiler verify every harvested measure response on-device
  (CRC recomputed and compared, status mask-checked; a mismatch drops
  that tick, sustained failure escalates through the sample watchdog).
  Omitting a CRC or status field the datasheet defines publishes
  corrupted or unprepared responses as plausible SI samples. Self-check:
  the compiled measure section carries one OP_CRC8 per harvested word.
- **`WHO_AM_I_VALUES` carries the full field-width value** of the
  FIXED_VALUE register; the FRAME prologue compares the whole data field
  on-device (see "A FRAME part's identity word" under Traps).
- **`read_words(reg, N)`** emits N + `read_pipeline` interleaved frames
  with `inter_frame_sleep` between them and packs each response's data
  field into the output region in declared order.
- **`configure()` is traced once** into one program that runs on whichever
  bus the `bus` param selects (auto-declared from `BUSES`, the first entry
  the default). There is no bus test at trace time: a Python branch on the
  bus compiles to nothing, so every write in `configure()` must hold on
  every declared bus. A part whose bring-up differs per bus declares one bus.
- **`BUSES = ('spi',)`** locks the personality to SPI. Without it the
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
  personality layer can declare a registered variant (e.g. `'input-lsb'`).
- **`read_words(reg, N)`** packs `N * 2` bytes (one 16-bit data field
  per response) into the output region, which ends at the rotating
  TX/RX slot — for a 32-bit FRAME that is 124 bytes, so 62 words max.
  Splitting into multiple bursts means multiple unlock/bank-switch
  dances if the data spans banks.
- **Multi-bank parts**: if the chip has separate banks selected via a
  `bank_select` register (e.g. bank 0 for data, banks 6/7 for
  full-scale-range, etc.), personalities must `write(bank_select, N)`
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
  OR `set_bits` immediate is the single rewrite site `nxs set` uses.
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
    """Read command word (DS command frame): even parity at bit 15 over
    bits 14:0, rw=1 at bit 14, 14-bit address."""
    word = (1 << 14) | addr
    return word | ((bin(word).count("1") & 1) << 15)


class ParityFramedEncoder(RegisterDriver):
    BUSES = ('spi',)     # literal wire words are SPI-only
    PINS = {}            # no data-ready output: poll
    SPI_PROFILE = SpiProfile(max_hz=10_000_000, mode=1)   # DS SPI timing table

    # No identity register; every response is parity and error-flag gated.
    WHO_AM_I_VALUES = []
    WHO_AM_I_SKIP_REASON = "no identity register; reads are parity-gated"

    CMD_POSITION = _read_word(0x3FFF)   # <REGISTER> (0x3FFF)

    def configure(self, config):
        # No rate register: sample_rate patches the poll interval only.
        self.declare_param("sample_rate", values=[10, 50, 100, 250],
                           default=100, unit="Hz")
        self.set_output([
            # Full turn = 2^14 counts: the divisor is the count space
            # 16384, not the maximum code 16383.
            {'name': 'angle', 'type': 'uint16',
             'scale': 2 * 3.141592653589793 / 16384},
        ])
        self.set_sample_size(2)

    @RegisterDriver.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.xfer(self.CMD_POSITION)      # prime: response arrives next word
        a = self.xfer(self.CMD_POSITION)  # response to the previous word
        if a & 0x4000:                     # error flag: drop the sample
            return None
        t = a >> 8                         # even-parity fold: xor the halves
        p = a ^ t                          # down until bit 0 holds the parity
        t = p >> 4
        p = p ^ t
        t = p >> 2
        p = p ^ t
        t = p >> 1
        p = p ^ t
        if p & 1:                          # parity violation: drop the sample
            return None
        angle = a & 0x3FFF
        return Sample(angle=angle)


# DS worked example: the <REGISTER> read command is 0xFFFF.
assert ParityFramedEncoder.CMD_POSITION == 0xFFFF
```

### Critical rules for literal-word (`xfer`) personalities

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

Stream personalities split chip-touching work and host-side declarations
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

    # A stream personality sets its baud with set_baud() in probe(); a
    # UART_PROFILE is a compile error (not applied by firmware).

    RATES = {1: 1000, 5: 200, 10: 100}   # Hz -> measurement period [ms]

    # Fixed probe command; never changes with config.
    PROBE_FRAME = bytes([ ... 14 B with pre-baked checksum ... ])
    ACK_PREFIX  = bytes([ ... first N bytes of expected response ... ])

    # Probe: the response to a known command is the identity check.
    def probe(self):
        self.set_baud(self.DEFAULT_BAUD)
        self.sleep_ms(100)
        self.write(self.PROBE_FRAME)
        ack = self.read_n(len(self.ACK_PREFIX) + 2,  # + CK_A/CK_B
                          timeout_ms=2000)
        ack.expect(self.ACK_PREFIX)

    # Configure: declarations, then param-driven chip writes via
    # stage/checksum/send so `set` rewrites them in place.
    def configure(self, config):
        self.declare_param("baud", values=[9600, 38400, 115200],
                           default=38400, unit="baud")
        self.declare_param("rate", values=list(self.RATES.keys()),
                           default=1, unit="Hz")

        # UART part: the personality pins the VM poll cadence.
        config['sample_rate'] = 100
        config['trigger'] = 'poll'

        rate_hz = config.get('rate', 1)
        meas_rate_ms = self.RATES[rate_hz]

        # stage() records the rewrite site (name, requested, encoded, frame
        # offset, size); compute_checksum() refreshes the checksum at runtime.
        frame = self._build_cfg_rate_frame(meas_rate_ms)
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

    # Measure: one logical record per sample.
    @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.read_until(b'\n', max=96)
        self.store_sample_n()
```

If the chip needs no runtime-tunable config (just declarations),
omit the `stage(...) → compute_checksum(...) → send_staged(...)`
block from `configure()` entirely — `probe()` already left the
chip in a usable state.

### Stream-personality probe patterns

Stream-personality `probe()` sends a known command and verifies a
structured response. Three worked examples by wire format:

**UBX (a binary-config GNSS)** — send a CFG-VALSET (or any
config-class write), expect UBX-ACK-ACK at the top of the response:

```python
def probe(self):
    self.set_baud(self.DEFAULT_BAUD)
    self.sleep_ms(100)
    # 24-byte UBX-CFG-VALSET that sets CFG_RATE_MEAS = 1000 ms.
    self.write(self._build_cfg_valset(rate_hz=1))
    # A factory-default receiver streams NMEA at power-on: sync to the UBX
    # header (0xB5 never occurs in NMEA text), then read the ACK-ACK body
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
    # `sdio,COM1,CMD,none\r\n` disables SBF output on COM1; the chip
    # echoes `$R+sdio,COM1,CMD,none\r\n` (or `$R?...` on NACK).
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
    # Pick the modem's default (most default to echo on).
    ack = self.read_n(8, timeout_ms=1000)
    ack.expect(b'AT\r\r\nOK\r')
```

If `read_n` doesn't see `count` bytes within `timeout_ms`, it emits
`OP_ERROR OpErrorCode.TIMEOUT`, which the VM reports as
`VmErr::TIMEOUT` (-417). If `.expect()` sees a mismatch on any byte,
it emits `OP_ERROR OpErrorCode.MISMATCH`, reported as
`VmErr::MISMATCH` (-418). Both surface to the runner the same way as
a register-personality WHO_AM_I mismatch: probe fails, slot advances.

### Stream-personality framing patterns

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
        return None                  # foreign frame: resync next pass
    bad = self.verify_checksum(ChecksumFletcher(), 0, 28, 28)
    if bad != 0:
        return None                  # corrupt frame: drop
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
bytecode (binary records vs a text stream) keeps BOTH in one personality:
one measure loop per protocol tagged `when=("protocol", "<value>")`,
exactly one `default=True`, and `configure()` branching on
`config['protocol']` (stamped before it runs). The choice is a
compile-time config key, not a runtime param: `nxs upload --config`
takes declared params, `sample_rate`, `bus` and `trigger` only and refuses
any other key, so a compile-time key is set from the Python API
(`<Driver>().compile({'protocol': ...})`), and switching means uploading
the other compile, or storing both in personality-store slots and cycling.

### Runtime-tunable parameters with `compute_checksum`

When a config parameter (like a GNSS's `rate`) controls bytes that
appear inside a frame WITH a CRC/checksum, the checksum bytes
must be computed at runtime — otherwise patching the rate bytes
leaves the checksum stale and the chip rejects the frame.

The recipe: stage the frame body via `self.write(b, param=(…))`
calls (so the rate bytes are rewritable) but compute the checksum
using `self.compute_checksum(spec, start, length, dst)` so the
checksum recomputes on every reload.

```python
def configure(self, config):
    self.declare_param("rate", values=[1, 5, 10, 20], default=1, unit="Hz")
    rate_hz = config.get('rate', 1)
    meas_rate_ms = self.RATES[rate_hz]

    # Stage the UBX frame (class, ID, length, payload) with the rate
    # bytes param-tagged so `set rate` patches them in place.
    self.write(...)  # ... building the frame at known offsets

    # The Fletcher bytes are recomputed on every reload, including after
    # `set rate` patches the rate bytes.
    self.compute_checksum(ChecksumFletcher(),
                          start_off=2, length=22, dst_off=24)
```

The one checksum spec is `ChecksumFletcher` (UBX, TCP, NTP). A frame
under another checksum (NMEA's XOR fold, a CRC-8, Modbus CRC-16) is a
gap card: the spec subclass is a host-side addition that doesn't
touch firmware, and it lands with the first personality that needs it.

## Critical API Rules

### Companion I²C devices (multi-die packages)

Some packages put TWO independent I²C slaves on one bus — a primary
die plus a companion die at its own fixed address, often with register
maps that overlap by number. One package is still ONE personality: splitting
the dies across two personalities ships a fraction of the part's measurands
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
  `read_burst`, in `probe()`, `configure()`, and
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
  a fused sample, not a per-die personality family.
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

A register personality's bus wire protocol is declared as one **profile per
bus** — a keyword-constructed descriptor that reads like the bus-interface
section of the datasheet, with no vendor names. The compiler bakes every
declared profile into the image; the firmware applies the one matching the
active `bus` param to the peripheral before `probe()` runs. Every input to a
profile is a datasheet fact or a fixed board fact — resolve them yourself and
generate a complete personality; never stop to ask the user which buses, clocks, or
switches to declare. The emitted `BUSES` / `BUS` / `*_PROFILE` stay as knobs
they can tweak afterward.

```python
from nxs import RegisterDriver, Sample, SpiProfile, I2cProfile

class MySensor(RegisterDriver):
    # Every number is copied from the datasheet's interface-timing table
    # and cited; never recalled, rounded, or taken from the feature summary.
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
auto-increment via the sub-address MSB). SMBus PEC (`pec='crc8'`) is not applied by firmware
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
  - **I²C** — the board bus is the **Fast-Mode tier**, 330 kHz on the
    wire (the socket's timing preset lengthens the edges), and
    `max_hz` snaps to a tier. Declare `I2C_PROFILE(max_hz=<datasheet I²C max>)`
    only when the part is *slower* than 400 kHz (some command-response parts
    at 100 kHz), which snaps the bus to Standard-mode; a part rated
    ≥ 400 kHz needs nothing — the declaration would be a no-op.

Copy the numbers from the datasheet timing tables, never the feature summary
(summaries round up; the timing table is the contract). The datasheet clock is
the silicon's rating — declare it even if a bus is bench-unvalidated on this
Click; if bring-up shows signal-integrity trouble, the cap belongs in the
board configuration, not the personality. Full reference: the personality development
guide's bus configuration section.

### Reset handling — decide from the schematic and the datasheet

The platform pulses the mikroBUS RST line (polarity per
`RESET_ACTIVE`) before every probe, including param reloads. Whether
that pulse reaches the part is a per-Click fact — decide reset
handling from two things you already extracted: does the Click route
mikroBUS RST to the sensor's reset/enable pin (Step 3 schematic), and
does the sensor even have one (Step 4 pin description)?

Every settle after a power-mode transition (a reset, a wake from sleep, a
mode change) is twice the datasheet's maximum start-up time for that
transition, floor 10 ms, with the table cited in the comment.

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
reset only for a part that documents none. For a register part the reset
is the first write of `configure()` and `probe()` stays read-only; a
command-protocol part with no identity register resets in `probe()`, the
datasheet-mandated reload being its presence test, and the coefficient
reads that follow live in `configure()`. The shape is exactly this, never
mid-sequence:

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
  from a register personality; drop the reset and rely on the full
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

Values are integers: the descriptor wire carries int32 and the compiler
refuses a fraction. Encode a fractional physical value in a smaller
integer unit and name that unit in `unit`.

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
the rewrite site — see the reserved-bits rule). The setting stays
compile-time only when its write is genuinely value-independent, or a
direct-`write` register is already owned by another param (RMW sites
may share). Expose a compile-time setting as a config key
(`config.get('key', default)`) with the dependent scale or behavior
baked per value, look the value up in a class-level table so an unknown
value fails the compile instead of falling back, and document it in the
module docstring's `Config keys` block; the key is reachable from the
Python API only (`<Driver>().compile({...})`), since `nxs upload
--config` refuses it as unknown. Hardcoding one value and
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
  `trigger="poll"` personality the compiler patches the loop's `SLEEP_MS` interval
  from the declared rate, so `set sample_rate` retunes the poll cadence with no
  register write. Do NOT also tag a write — the poll interval and a rate
  register are two ways to set the same knob, so the compiler rejects a
  `sample_rate` that drives both.

### Trigger handling

**A data-ready output makes DRDY the default.** Any pin the part
drives at its output data rate counts: a DRDY/INT latch, an ODR sync,
even one fixed at the part's internal rate with no divider. The
mikroBUS `INT` pin (`mkbus_int`) is the firmware's DRDY input. A
fixed-rate sync the loop can't service edge-for-edge still wins: the
loop pace-locks to the first edge after each pass, so the cadence stays
hardware-derived.

The unit arms `mkbus_int` on the edge to its active level, and the board
declares the line active-high, so a sample is taken on the rising edge. A
part whose data-ready output idles high (an active-low INT, the common
default) is set to active-high push-pull in `configure()` through its
interrupt-polarity bit, cited from the datasheet; an open-drain output
relies on the Click's pull-up.

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
- **Run the wire-time budget** (see "Data path" below) against the
  fastest declared `sample_rate`. Past the budget a part without a
  coherent output set rewrites its registers while the burst is still on
  the wire — torn samples, often an all-`0xFF` tail. The answer is a
  shorter rate list, never the on-chip FIFO.
- **UART sensors**: ALWAYS hardcode `@measure_loop(trigger="poll", sample_rate=1)`
  → the personality sets `config['sample_rate']` and `config['trigger']` internally
- **Register-bus parts with no data-ready output** (and command-response
  parts, where the bus paces each conversion): use
  `@measure_loop(trigger="poll", sample_rate=N)`

**`sample_rate` on a register-less part depends on the sync.** Three
cases:

- **A rate register exists**: declare `sample_rate` unconditionally —
  the tagged register write is a rewrite site that works under both
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
        # Poll cadence only: no rate register and no divisible sync.
        self.declare_param("sample_rate", values=[10, 25, 50, 100],
                           default=100, unit="Hz")
```

### Data path

**One burst per sample.** `measure()` reads the sample with one burst from
the data registers: at the data-ready edge in a DRDY loop, behind the status
gate in a poll loop. One transaction keeps three properties at once: the
stamp a sample carries is the edge of the data it holds, a pass that runs
late loses the samples it overslept and nothing after them, and the pass is
as short as the bus allows.

```python
def measure(self):
    status = self.read(0x3A)             # the status register, cleared on read
    if not (status & 0x01):              # the data-ready bit
        return None
    raw = self.read_burst(0x3B, 14)      # OUT_X_H .. OUT_Z_L: the whole sample, one burst
    return Sample(raw)
```

**The budget is arithmetic, not judgment.** The declared `sample_rate`
values are a contract — every value must hold on every declared bus — so
budget one measure pass on the *slowest* declared bus (I²C for a dual-bus
part) at the *fastest* declared rate:

```
t_pass ≈ 9 × (N + 8) / f_i2c      # status gate + addressing + N-byte burst
t_pass ≈ 8 × (N + 4) / f_spi      # the same pass over SPI
```

`N` is the sample size in bytes. These are **wire terms only** — the
executed pass adds platform overhead: budget ≈ 200 µs per bus
*transaction* (VM dispatch plus bus-stack setup on the target MCU class)
and every staging sleep at face value. A DRDY loop adds one such term for
its burst, a poll loop two (the status read and the burst), and the wire
arithmetic stands; a per-word framed protocol that
reads W words as W transactions is *dominated* by the overhead term —
`t_pass ≈ W × 200 µs + Σ staging sleeps + wire` — and the fastest honest
`sample_rate` is what that end-to-end pass sustains, not what the wire
math promises. Worked: 7 words with 6 × 250 µs staging gaps → ≈ 2.9 ms
per pass → ~340 Hz sustainable → against a fixed 4 kHz sync, declare the
exact divisors up to 250 Hz and stop.

A measurand outside the burst window (a die temperature in another
register block) costs a second transaction. Publish it when the pass with
both transactions still holds the bound below; otherwise state its
exclusion.

Compare the pass against the sample period:

- The part holds a **coherent output set** during a burst (the datasheet
  input extracted above) → a burst reads one sampling instant at any
  phase, so the pass has the whole period: `t_pass ≤ period`.
- The part rewrites its outputs under the burst → keep 3× headroom for
  DRDY wake latency and the next update landing mid-burst:
  `t_pass ≤ period/3`.
- A declared rate that misses its bound on a declared bus is removed from
  the `sample_rate` values. A declared rate the bus can't carry is a
  capabilities lie.

Poll-paced parts run the same check with the whole pass (conversion sleeps
included) against the poll period; with no tear risk, no 3× headroom.

Worked numbers — a 14-byte sample on the board's I²C, 330 kHz on the
wire, DRDY-paced: t_pass ≈ 200 µs + 9 × 22 / 330 kHz ≈ 800 µs, and a poll
loop adds the status read's 200 µs. On a part with a coherent
output set that holds 1000 Hz. Without one the bound is 333 µs at 1000 Hz
and 667 µs at 500 Hz, so the list stops at 250 Hz.

**The on-chip FIFO is not a data path.** Leave it disabled. A `measure()`
body is one fixed burst per wake: it cannot drain a backlog and it cannot
filter a batch, which are the two things a FIFO is for. Read one frame per
wake, a FIFO only adds transactions (a count read before the frame, often
one after), and any frame it queues behind a late pass is published under
a later edge's stamp. A part whose data is reachable only through its FIFO
gets the NOT EXPRESSIBLE card naming the datasheet section.

**A refused transaction.** The runner checks the bus, backs off and
re-executes the faulted op, so measure-section ops must be safe to repeat:
reads are, and a measure loop writes nothing. The samples inside the
back-off are lost and counted. A refusal rate that *tracks pacing or rate*
is a budget violation: re-check the arithmetic above before blaming the
part.

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
the canonical unit from the table below (omit `unit` for these fields;
declaring a conflicting one is a compile error), and
a Cyphal host projects the field onto a standard `uavcan.si.sample.*`
subject with no per-personality schema. Vector quantities take `_x/_y/_z`
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
the personality, not on the host.

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
`read_burst`/`uart.read` counts); a personality that also uses a value-read
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
of rewrite sites, at the same offsets, for every value; a conditional write
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

**A FRAME part's identity word.** `WHO_AM_I_VALUES` on a FRAME personality is
compared against the frame's data field on-device (not just in `probe()`), so
it must be the full field-width value (a CRC-framed part's is `0x1234`, not
the high byte `0x12`).

## Step 5b: Write the Descriptor

The facts beside the code: write `./<name>/<name>.yaml` explicitly — the
personality does not exist until both files do. Its shape (the params table the
`configure()` template above reads through `declare_params_from_descriptor()`):

```yaml
# yaml-language-server: $schema=https://aliensense.github.io/nxs-docs/schemas/unit-driver.schema.json
# <one-line brief>
# Parameter capabilities; order fixes the wire's param
# indices, so entries never reorder.
meta:
  driver: <name>
  kind: unit
params:
  - {name: sample_rate, values: [10, 50, 100, 250, 500, 1000], default: 250, unit: Hz}
  - {name: range, values: [2, 4, 8, 16], default: 8, unit: g}
```

`meta.driver` equals the directory and file name. `nxs personality check ./<name>`
refuses a mismatch, a default outside its values, and a duplicate name.
Entries take an optional `type: range` (values exactly `[min, max]`; a range
is always live) and `kind: live` (applied without a VM restart); the defaults
are enum and reload. A computed table (values derived at configure time) calls
`self.declare_param()` explicitly; the YAML carries plain facts. A value that
never passes through `declare_param()` is invisible to `nxs caps`, `status`
and `tune`.

## Step 6: Self-Check

Compile first — the installed `nxs` tool is the compiler; no device is
needed. One invocation per declared bus, from the pair the user will use:

```bash
nxs upload ./<name>/<name>.py -o <name>.nxs
nxs upload ./<name>/<name>.py --config bus=spi -o <name>-spi.nxs
nxs personality check ./<name>
```

A `CompileError` names the offending construct — fix the personality and
re-compile until every declared bus passes. The compiler sweeps every
declared param value, so a passing compile covers the whole value set, not
just the defaults. Then check the semantics the compiler can't see:

- **Class**: subclasses `RegisterDriver` (I²C/SPI) or `StreamDriver`
  (UART). Never plain `SensorDriver`.
- **Naming**: file = the part number in lower_snake, class = the same
  in PascalCase (`xyz9000.py` / `Xyz9000`), consistent with the
  sibling personalities in the package; never the carrier board's marketing
  name.
- **Class checks**: every `references/<class>.md` read in Step 2 has
  its Self-checks section applied — a combo part applies every
  matching file's checks.
- **Companions**: a package with a second co-resident I²C slave
  declares it in `I2C_COMPANIONS` (own identity anchor or skip
  reason), reaches it only via `dev=`, and publishes a fused sample
  spanning every die — never a per-die personality split.
- **Probe anchors**: `WHO_AM_I_REG` and `WHO_AM_I_VALUES` are
  declared as class attributes for I²C/SPI register personalities. For
  personalities whose `BUSES` includes `'i2c'`, also declare `I2C_ADDRS`
  with every strapped address the part can land on — NXS scans
  these on load. SPI-only personalities (`BUSES = ('spi',)`) don't need
  `I2C_ADDRS`; the default of `[]` means no I²C scan happens, which
  is correct.
- **`__init__`**: seeds `self._read_responses` with WHO_AM_I (and any
  other register reads inside `probe()`) so the host-side tracer can
  clear the probe asserts.
- **`probe()`**: only reads registers + asserts, no loops. The one write
  allowed is the reset a command-protocol part without an identity
  register sends as its presence test.
- **`set_output`**: every field states `type` and `byte_order` (the
  defaults are `int16` and `big`) and its SI scale.
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
  output is active at power-on — one of the two must be in the personality.
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
  capability the datasheet documents that the personality silently lacks
  fails this check.
- **Value tables are consumed whole.** When a datasheet table backs a
  param or config key (full-scale codes, bandwidth codes), the personality
  exposes every row, or names the rows it dropped and why. Count the
  rows you extracted against the table's row count — a table that
  lost rows between the datasheet and the personality is a silent
  reduction, and a table you could only partially read is a gap-card
  stop, not a shorter table.
- **Hygiene**: the docstring's pins line names mikroBUS signals, never
  MCU peripheral instances; class constants and tables are consumed by
  probe/configure/measure, no dead declarations.
- **Docstring accuracy**: the docstring's `Config:` line matches the
  declared params and config keys verbatim (names, value sets,
  defaults). Re-read it against the final `declare_param` calls and
  `config.get` defaults before finishing.
- **Prose**: the docstring fits the shape under "Docstring and
  comments" (at most eight lines after the title); no comment exceeds
  two lines; every register write cites its register or section; no
  issue numbers, dates, bench names, first person, history, or
  repository paths anywhere in the file.
- **Computed wire words** (`xfer` personalities): every command word is an
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
- **Data path**: one burst per sample, the on-chip FIFO disabled, and
  the wire-time budget ("Data path") holds for the fastest declared
  `sample_rate` on the slowest declared bus — rates capped where it
  does not.
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
  profile you declare (`SpiProfile` / `I2cProfile`); and, only for
  framed transports, `from nxs.framing import Crc, SpiFrame` and
  `from nxs.compiler import ChecksumFletcher`. Nothing else (no
  `math`, no `struct`).

## Step 7: Report

When the personality is written and self-checked, output **exactly the card below**
— filled from the personality you created — and nothing else. No preamble, no
narrative, no at-rest values, no troubleshooting, no commentary. The structure
is identical on every run.

```
nxs personality: <name>

  files     ./<name>/<name>.py · ./<name>/<name>.yaml
  bus       <default> (default)[ · <other>]
  trigger   <trigger>[ · <rate> Hz]
  outputs   <field> (<unit>)[ · <field> (<unit>) …]
  params    <name>=<default>[ · <name>=<default> …]

  nxs upload ./<name>/<name>.py[ --config bus=<other>]
  nxs stream --hz <rate>
  nxs set <param> <value>
  nxs store save 0
  nxs personality install ./<name>
```

Fill from the personality: the two paths are the pair Step 5 wrote in the
working directory; group axis fields (`accel_x/y/z`) under one unit;
`<rate>` is the integer sample rate. A single-bus part shows only `bus
<the bus>` and drops the `--config bus=` hint. Default transport is I²C; append `-t cyphal-serial -p
<port>` (or `-t cyphal-can -p <iface>`) to the commands only if the user asked
for it. Add no lines beyond the card.

**Multi-sensor (suite).** For a host that manages several NXS units
declaratively, also show the suite.yaml form — the new personality becomes a
`sensors:` entry under its unit, and `nxs switch` deploys it:

```yaml
units:
  - name: <unit-role>
    module: nxs
    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x30}]
    sensors:
      - personality: <sensor_name_lowercase>
        config: {sample_rate: 100, <other_key>: <value>}
```

The switch converges every declared unit and is idempotent:

```bash
nxs switch
```

After live-tuning params on a declared unit (`nxs --unit <name> set ...`),
`nxs tune --freeze --unit <name>` adopts the tuned values back into the
manifest. The manifest schema, switch semantics, and the tune-then-freeze
flow are in the Integration & Operation Manual §5
(https://aliensense.github.io/nxs-docs/reference/nxs-integration-manual/).

When the part is not expressible (the stop rule above), output exactly
this card instead — one `missing` block per gap — write no personality file,
and end the run:

```
nxs personality: <name> — NOT EXPRESSIBLE

  part      <chip> — <bus/protocol in one line>
  needs     <the wire behavior the datasheet requires>
  missing   <the DSL construct that does not exist; quote the exact
             CompileError when one was raised>
  closest   <the nearest documented construct and why it falls short>
  unblock   <the smallest DSL extension that would cover this part>
```
