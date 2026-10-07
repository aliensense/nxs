# NXS — Click Personality Reference

Applies to: NXS v1.1 · image format 2.0 · `nxs` tool 1.1.x

| Document set | |
|---|---|
| [Device Reference](nxs-device-reference.md) | interfaces, performance and limits, shipped personalities, versioning |
| [Interface Description](nxs-host-interface.md) | transports, register map, commands, procedures |
| [Integration & Operation Manual](nxs-integration-manual.md) | design-in, host setup, workflows |
| **Click Personality Reference** (this document) | authoring personalities for unsupported sensors |
| [Cam Personality Reference](nxs-cam-personalities.md) | describing camera chains for `nxs cam` |
| [MCP Tool Reference](nxs-mcp.md) | operating and configuring through an AI agent |
| [FAQ](https://aliensense.github.io/nxs-docs/hardware/faq/) | frequently asked questions |
| [Glossary](nxs-glossary.md) | one word for each thing |
| [Technical Specifications](nxs-specifications.md) | capability summary tables |

## 1. The authoring model

An NXS personality is a Python class that *describes* a sensor: its identification, its configuration registers, its parameters, its sampling loop and its output fields. The `nxs` tool traces the class and compiles it to a bytecode image, which the virtual machine on the unit executes. The personality never runs as Python on the unit. Authoring is therefore datasheet transcription. Register addresses, conversion formulas and timing move from the sensor datasheet into a declarative class.

The input is a datasheet and the output is a constrained, verifiable artifact, so the primary authoring flow is **AI-assisted**. A coding agent receives the sensor datasheet, and this guide is the specification the agent follows. The specification is the class shape, the DSL contract and the validation gates below. The `nxs-generate-click-personality` skill shipped with the product is the reference implementation of that flow. A personality written by hand against the same contract is supported in the same way. The compiler and the validation contract do not distinguish the two.

Whichever path produced the personality, **the validation contract (§3) is the acceptance gate**. A personality is trusted because it passed verification, not because of how it was written. The shipped personalities are generated artifacts, and they are regenerated rather than edited. An author's own personality may be written either way.

The click personalities regenerated from their datasheets by the shipped skill compare with the hand-written ones the wheel carries. The IAM-20680 differs in a wake settle and a longer rate list. The FXOS8700 differs in its identity list, its status gate and the temperature channel, and the MS5611 in where its oversampling key lives. Each difference is a datasheet reading the validation contract settles.

## 2. A complete personality

```python
from nxs import RegisterClickPersonality, Sample

class MySensor(RegisterClickPersonality):
    WHO_AM_I_REG = 0x0F
    WHO_AM_I_VALUES = [0x6A]
    I2C_ADDRS = [0x6B, 0x6A]

    RATE_DIV = {50: 0x0F, 100: 0x07, 250: 0x03, 500: 0x01}  # Hz -> divider bits
    ACCEL_FS = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}        # g -> register value
    ACCEL_BASE_SCALE = 9.80665 / 32768.0                    # m/s^2 per LSB, per g

    def __init__(self):
        super().__init__()
        # Trace-time responses: compilation runs this class on the host,
        # so reads that gate control flow need a seeded answer.
        self._read_responses = {self.WHO_AM_I_REG: [self.WHO_AM_I_VALUES[0]]}

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who in self.WHO_AM_I_VALUES

    def configure(self, config):
        self.declare_param("sample_rate", values=[50, 100, 250, 500],
                           default=250, unit="Hz")
        self.declare_param("accel_fs", values=[2, 4, 8, 16],
                           default=8, unit="g")

        sample_rate = config.get('sample_rate', 250)
        accel_fs = config.get('accel_fs', 8)

        self.write(0x10, self.RATE_DIV[sample_rate],
                   param=("sample_rate", sample_rate))
        self.write(0x11, self.ACCEL_FS[accel_fs],
                   param=("accel_fs", accel_fs))

        self.set_output([
            {'name': 'accel_x', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'accel_fs'},
            {'name': 'accel_y', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'accel_fs'},
            {'name': 'accel_z', 'scale': self.ACCEL_BASE_SCALE,
             'scale_param': 'accel_fs'},
        ])
        self.set_sample_size(6)                              # 3 x int16

    @RegisterClickPersonality.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x1E)
        if not (status & 0x01):
            return None                                      # not ready
        raw = self.read_burst(0x28, 6)
        return Sample(raw)
```

Compile and run it exactly like a shipped personality:

```sh
nxs upload ./my_sensor.py --param sample_rate=250
nxs stream --count 5
```

## 3. The validation contract

Every personality, generated or hand-written, is verified against the device before it is trusted:

1. **Identity.** Declare `WHO_AM_I_REG` + `WHO_AM_I_VALUES` from the datasheet. The compiler auto-emits an identity prologue into the bytecode. The wrong sensor then halts the VM with a mismatch error, which `nxs status` shows, instead of streaming garbage. The assert in `probe()` runs at compile (trace) time against the seeded responses, not on the device. A sensor with no identity register must declare `WHO_AM_I_VALUES = []` with a `WHO_AM_I_SKIP_REASON`.

   Values the datasheet lists are normal evidence. A set derived from a fixed-bits claim ("bits n:m vary, the rest read 0") is weak. Silicon ships with "fixed" bits set. Corroborate such a set against a vendor or kernel reference driver. Never include `0x00` or `0xFF` without justification, because they also match empty reads and false-accept a wrong die. On a multi-die package whose primary's product code is factory-variable, put the hard identity on the die with a fixed WHO_AM_I (the companion check).
2. **Parameters against the datasheet.** Every `declare_param` value set matches a datasheet table (full-scale ranges, ODR dividers, oversampling ratios). Every config-dependent register write carries its `param=` tag, so runtime `set` works without re-upload.
3. **Units are canonical and enforced.** Fields whose names infer an SI semantic inherit their canonical unit at compile time. Such names are `accel_*`, `gyro_*`, `mag_*`, `temp`, `pressure`, `angle`, `voltage` and more. The units are `m/s^2`, `rad/s`, `tesla`, `kelvin` and `pascal`, and a conflicting declared unit is a compile error.

   The full recognized set, with each name's SI unit and standard subject, is the canonical-units table in §4.3, single-sourced from `constants/field_semantics.yaml`. Temperature is kelvin: fold the datasheet's Celsius zero point into the field's `offset` (+273.15). Fields with no SI semantic must declare a unit (`''` for unitless).
4. **Independent decode.** Stream from the device. Recompute at least one field by hand from the datasheet formula, raw counts × datasheet sensitivity. Compare the result against the device-served decode. The two derivations must agree. This check catches transposed scales that all other gates miss.
5. **Physical plausibility.** A stationary accelerometer reads ≈ 9.81 m/s² on one axis. Ambient temperature is ≈ 293–300 K, ambient pressure ≈ 101 kPa, and Earth's magnetic field ≈ 25–65 µT.
6. **Parameter round-trip.** `set` a full-scale range live. Check that the streamed magnitude stays the same for the same physical stimulus, because the scale retunes with the parameter.

## 4. DSL reference

### 4.1 Personality classes

| Base class | Sensor type | I/O verbs |
|---|---|---|
| `RegisterClickPersonality` | register-mapped I²C/SPI (IMUs, magnetometers) | `read`, `write`, `write_modify`, `read_burst`, `xfer` |
| `I2cCommandClickPersonality` | command-response I²C (barometers) | `send_command`, then `sleep_ms` and `read` as on a `RegisterClickPersonality` |
| `StreamClickPersonality` | UART streams (GNSS/NMEA) | `write`, `read`, `read_until`, `set_baud` |
| `CamPersonality` | CSI image sensors on a camera pod (§4.14) | `write`, `write_table`, `poll`, `select`, `sleep_ms` |

### 4.2 The facts file (`<personality>.yaml`)

A click personality ships as two files: the Python module (measurement logic) and a sibling facts file carrying its parameter table. The facts file is the machine-readable capability sheet — `nxs status` and `nxs tune` read it directly, with no personality import and no compilation.

```yaml
meta:
  personality: imu_archetype
  brief: 6-axis IMU, register bus
params:
  - {name: sample_rate, type: enum, values: [50, 100, 250, 500], default: 250, unit: Hz}
  - {name: accel_fs, type: enum, values: [2, 4, 8, 16], default: 8, unit: g, kind: reload}
```

The loader that `nxs status` and `nxs tune` share judges the table before any compile, and names the file. Parameter names are unique, an enum default is one of its values, and a range is `[min, max]` in that order with its default inside it.

`configure()` registers the whole table with one call:

```python
self.declare_params_from_descriptor()
```

Explicit `declare_param` calls remain valid alongside it for computed cases: a value set derived at configure time, or a parameter gated on the requested trigger. The split changes nothing on the wire. The compiled image is byte-identical to one built from explicit declarations, and the SDK pins that equivalence with image fixtures.

### 4.3 `declare_param(name, values, default, param_type="enum", unit="", kind="reload")`

Declares a runtime-settable parameter (name ≤ 16 bytes, unit ≤ 8 bytes). `param_type="enum"`: `values` is the allowed set, ≤ 16 entries. `param_type="range"`: `values` is `[min, max]`, validated inclusively on each set. A range parameter must be `kind="live"`, because a range has no bytecode patch site to re-apply on reload. The firmware rejects a non-live range at load.

Both shapes are validated at compile time, the declared default and any config override included, and are runtime-settable via `nxs set`. `kind="reload"` re-runs `configure()` on set, which is correct for sensor config registers. `kind="live"` applies without a restart, for values a running consumer reads continuously. Every parameter is visible to hosts with its allowed set or bounds and default (`nxs caps`, `GetParamInfo`).

The compiler enforces the parameter contract, so a `set` is never a silent no-op. A param is declared before its `param=` tag, and the tagged value is in the declared set. A param patches at most `MAX_PATCH_SITES` bytecode sites, and a setting written to two registers owns two. A direct `write` still binds one register to one param, but read-modify-write sites do not clobber and may share.

The set of patch sites must not change between values. A `reload` enum that patches no bytecode is rejected. So is a `live` param that no runtime consumer reads, and the runtime consumers are only the PWM pair and an output `scale_param`. Patch width follows the write: one byte for `write`, four for `set_baud` and a `write_modify` field, `n` for `stage(size=n)`. Every value must fit it.

Filter bandwidth is a runtime parameter whenever the part has one. Declare the values as physical cutoff frequencies in Hz (never register codes), map each to its code in a lookup table, and tag the write. Codes that change the internal base rate the rate divider divides are excluded. Such a code re-times every declared `sample_rate` value rather than selecting a filter.

### 4.4 `set_output(fields)` and canonical units

Each field: `{'name', 'scale', 'offset'?, 'unit'?, 'type'?, 'byte_order'?, 'scale_param'?}`. The field's *name* selects its semantic. The semantic selects the canonical unit and, over Cyphal, the standard `uavcan.si.sample.*` subject:

| Field name(s) | Canonical unit | Field name(s) | Canonical unit |
|---|---|---|---|
| `accel_x/y/z` | `m/s^2` | `voltage` | `V` |
| `gyro_x/y/z` | `rad/s` | `current` | `A` |
| `mag_x/y/z` | `tesla` | `distance` | `m` |
| `temp` | `kelvin` | `force` | `N` |
| `pressure` | `pascal` | `frequency` | `Hz` |
| `angle` | `rad` | `luminance` | `cd/m^2` |
| `mass` | `kg` | `torque` | `N*m` |
| `speed` | `m/s` | `flow` | `m^3/s` |

Omit `unit` for these fields: the compiler inherits the canonical unit and rejects a conflicting declaration. Matching is by exact name, case-insensitive, after alias folding. The full semantic names (`temperature`, `frequency`, `mass`, `distance`, `flow`) infer the same semantics as their table entries. So do the aliases `temp`, `freq`, `weight`, `range` and `flow_rate`. Names outside that set are *generic*: they stream with full self-description (name, type, scale/offset, declared unit) but claim no standard subject. `scale_param` names a declared parameter whose live value multiplies the base `scale` on-device, the idiom for runtime-tunable full-scale ranges.

The same semantics select per-unit calibration. Fields carrying the `accel_*`, `gyro_*`, and `mag_*` semantics form the three calibration vector buckets. The `angle` semantic carries the encoder zero-offset ([Interface Description §6.9](nxs-host-interface.md)). Declaring the semantic is the entire opt-in. The correction applies in the SI tier, so the personality's sample layout, scales, and raw stream are untouched.

Fields are packed sequentially by default. `'at': N` places a field at an explicit byte position within the sample. That is the binary-record idiom, where fields map onto scattered offsets inside a captured frame and the gaps (headers, reserved bytes, checksums) carry no fields. Explicit placement is all-or-none per personality. Overlapping fields, a field past the sample buffer, or more fields than the descriptor table holds are compile errors. Every descriptor carries its resolved byte position on the wire, so hosts decode each field at its declared offset rather than accumulating widths.

### 4.5 `@measure_loop(trigger=...)`

`trigger="drdy"` waits on the sensor's data-ready line. `trigger="poll", sample_rate=N` paces at a fixed rate. `trigger="from_config"` defers the choice to the configuration. The config's `trigger` key selects `drdy` or `poll` at compile time, and the `sample_rate` parameter paces the polling case. That is the idiom for sensors that support both modes. A part with any data-ready output, a fixed-rate sync with no divider included, uses `trigger="from_config"` with `drdy` as the default.

Hardcoded `poll` is for parts with no such pin and for bus-paced protocols (UART streams, command-response conversions). On a poll-paced personality a declared `sample_rate` with no rate register patches the loop's sleep interval, so `set sample_rate` retunes the cadence with no register write.

A **fixed-sync part** (data-ready at a fixed hardware rate, no divider register) additionally declares `drdy_base_hz=<sync Hz>` on the decorator. The compiled drdy loop then paces by dividing the sync at the source. The unit counts hardware edges in the interrupt and wakes the loop every `base/rate`-th edge. The delivered spacing is therefore exact against the sensor's own clock, even while a read burst spans many edges. Every declared `sample_rate` value must divide the base exactly (compile error otherwise). The same declaration paces poll mode via the sleep interval: one parameter, both modes.

The body reads raw bytes and returns them, or `Sample(field=value)` for computed values, where on-device compensation runs in 64-bit fixed point. Returning `None` skips the tick.

The body executes on-device. `configure()`, by contrast, is traced at compile time against mock reads, so a value read in `configure()` is a placeholder. A runtime conditional on it is rejected and belongs here in `measure()`. A register read-modify-write is `write_modify` (§4.7), whose read happens on-device at load.

The body compiles to VM bytecode and resolves integer **literals** (write registers/masks as hex with a naming comment) and **UPPER_CASE class-level integer constants**. `self.CMD_X` is the home for computed wire words. Any other `self.*` value is rejected, except `configure()`-bound coefficients.

Reads come in two shapes. `read_burst(reg, count[, into=off])` *places* bytes in the sample buffer at `into` (default 0). A second bank takes a distinct `into=`, and an overlap is a compile error. `read(reg, width[, signed=, endian=])` returns a *value* into a register: unsigned big-endian by default, `signed=True` for signed fields and `endian="little"` for little-endian parts. Value-reads stage off the sample buffer, so they never corrupt placed data. `sleep_ms(n)` and `sleep_us(n)` pace conversions and inter-word gaps.

Control flow supports `if`/`elif`/`else` and the integer comparisons `== != < > <= >=`. Loops and function calls other than the `self.*` verbs are rejected at compile time.

A personality whose protocols need *different framing bytecode*, which a runtime parameter cannot patch, declares one measure loop per protocol. Each loop is tagged `when=("<config key>", <value>)`, with exactly one marked `default=True`. `compile(config)` picks the variant the config names. An unknown value, mixed tagged/untagged loops, or a missing default are compile errors. `configure()` branches on the same key in plain Python, and the key is a compile-time configuration, not a runtime parameter. Switching protocols means uploading the other configuration, or storing both in personality-store slots and cycling.

**Acquisition timestamps.** Samples are stamped with their acquisition instant automatically. A drdy loop stamps the delivered data-ready edge, and a pass nothing armed stamps its commit instant. One verb arms the bound for data that predates its delivery. `stamp_frame()`, placed at a stream loop's frame-sync point, stamps the RX backlog's first-byte arrival. That arrival equals the frame's own first byte only while the loop drains and stamps every pass.

### 4.6 Data path: one burst per sample

A measure pass reads the sample with one burst from the data registers. The burst comes at the data-ready edge in a DRDY loop, and behind the status gate in a poll loop. The stamp a sample carries is then the edge of the data it holds. A pass that runs late loses the samples it overslept and nothing after them. The declared `sample_rate` values are a contract, and every value must hold on every declared bus. So budget one measure pass on the slowest declared bus at the fastest declared rate:

```
t_pass ≈ 9 × (N + 8) / f_i2c      # status gate + addressing + N-byte burst
t_pass ≈ 8 × (N + 4) / f_spi      # the same pass over SPI
```

`N` is the sample size in bytes. Add about 200 µs per bus transaction for the unit's own overhead, and every staging sleep at face value. Compare the pass with the sample period.

Some parts hold one coherent output set while a burst is on the wire (shadowed or latched output registers, block data update). Such a part reads a whole sample at any phase, and the pass has the full period: `t_pass ≤ period`. A part that rewrites its outputs under the burst needs 3× headroom: `t_pass ≤ period/3`. A declared rate that misses its bound on a declared bus is removed from the `sample_rate` values. Poll-paced parts run the same check with conversion sleeps included and no 3× headroom.

Worked: a 14-byte sample on the socket's I²C bus, 330 kHz on the wire, takes about 0.8 ms. A part with a coherent output set holds 1000 Hz. A part without one stops at 250 Hz.

The on-chip FIFO stays disabled. A measure body is one fixed burst per wake, so it can neither drain a backlog nor filter a batch. Read one frame at a time, a FIFO adds bus transactions to every pass. A frame it queues behind a late pass is published under a later edge's stamp.

The runner re-executes a measure op whose bus transaction was refused, after a bus check and a short back-off. Measure-section ops must therefore be safe to repeat. The samples inside the back-off are lost, and the unit counts them (`io_err_count`, `drdy_coalesced_count`, Interface Description §12.1).

**Binary-record streams.** A UART part that emits framed binary records publishes *typed fields at their record offsets* instead of a string blob. Such a record is sync bytes, a fixed header, little-endian payload fields and a trailing Fletcher checksum. The fields then carry SI semantics and project onto the standard subjects. The shape: sync with `read_until`, which consumes the sync bytes so the record lands at offset 0. Capture the fixed-length record with `read_n`. Gate the header with `match(...)`, the mismatch count of the leading bytes against constants.

Verify the checksum with `verify_checksum(ChecksumFletcher(), start_off, length, ck_off)`, the RX counterpart of `compute_checksum`. It compares the recomputed bytes against the received ones at `ck_off`. Then commit the fixed-length record with `store_sample()`, which publishes the declared `set_sample_size`. A variable-length record read with `read_until` commits with `store_sample_n()` instead. Both gates return a count usable in `if`, and a nonzero means drop the record and resync on the next pass.

```python
from nxs import StreamClickPersonality, ClickPersonality
from nxs.compiler import ChecksumFletcher


class BinaryRecordSensor(StreamClickPersonality):
    """Fictional framed-binary protocol: sync A5 5A, 4-byte header
    (class 0x02, id 0x11, len 24 LE), 24-byte payload, 2-byte
    Fletcher checksum — 30 bytes buffered per record."""

    def configure(self, config):
        self.set_baud(115200)
        self.set_output([
            {'name': 'distance', 'type': 'int32',
             'byte_order': 'little', 'at': 4, 'scale': 1e-3},
            {'name': 'speed', 'type': 'int32',
             'byte_order': 'little', 'at': 12, 'scale': 1e-3},
        ])
        self.set_sample_size(30)

    @ClickPersonality.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.read_until(b'\xa5\x5a')     # sync to the frame; record lands at 0
        self.read_n(30)                  # header + payload + checksum
        m = self.match(0x02, 0x11, 0x18, 0x00)
        if m != 0:
            return None                  # foreign frame — resync next pass
        bad = self.verify_checksum(ChecksumFletcher(), 0, 28, 28)
        if bad != 0:
            return None                  # corrupt frame — drop
        self.store_sample()              # fixed-length record: compile-time size
```

### 4.7 Literal SPI words: `xfer(word, width=2)`

Some SPI parts place *computed* bits inside the command word itself, and a parity bit over rw+address is the classic case. Their words therefore do not decompose into `FRAME` fields. Every such word is a compile-time constant. Compute it with plain Python at class definition time, store it as an UPPER_CASE class constant, and clock it verbatim with `x = self.xfer(word)`. The word's bytes go on the wire MSB-first with one chip-select assertion per word. The response loads unsigned MSB-first at the full `width` (1, 2, or 4 bytes, default 2).

In statement position the response is discarded. That is the pipeline-priming idiom for parts whose response to word N arrives during word N+1. `xfer` requires `BUSES = ('spi',)`. Response checks are ordinary `measure()` arithmetic: parity folds as two-statement shift/xor steps, error flags as masked gates, each violation dropping the sample with `return None`.

```python
from nxs import RegisterClickPersonality, Sample, SpiProfile


def _read_word(addr):
    """Read command word: parity(15) | rw=1(14) | addr(13:0), even
    parity over bits 14:0 — evaluated at class definition."""
    word = (1 << 14) | addr
    return word | ((bin(word).count("1") & 1) << 15)


class ParityFramedEncoder(RegisterClickPersonality):
    BUSES = ('spi',)
    PINS = {}
    SPI_PROFILE = SpiProfile(max_hz=10_000_000, mode=1)
    WHO_AM_I_VALUES = []
    WHO_AM_I_SKIP_REASON = "no identity register; reads are parity-gated"

    CMD_POSITION = _read_word(0x3FFF)

    def configure(self, config):
        self.declare_param("sample_rate", values=[10, 50, 100, 250],
                           default=100, unit="Hz")
        self.set_output([
            # 14-bit angle: the scale divisor is the count space
            # (2^14 = 16384), never the maximum code (16383).
            {'name': 'angle', 'type': 'uint16',
             'scale': 2 * 3.141592653589793 / 16384},
        ])
        self.set_sample_size(2)

    @RegisterClickPersonality.measure_loop(trigger="poll", sample_rate=100)
    def measure(self):
        self.xfer(self.CMD_POSITION)      # prime: response arrives next word
        a = self.xfer(self.CMD_POSITION)  # response to the previous word
        if a & 0x4000:                     # error flag → drop
            return None
        t = a >> 8                         # even-parity fold of the response
        p = a ^ t
        t = p >> 4
        p = p ^ t
        t = p >> 2
        p = p ^ t
        t = p >> 1
        p = p ^ t
        if p & 1:                          # parity violation → drop
            return None
        angle = a & 0x3FFF
        return Sample(angle=angle)


# A wrong parity helper fails the compile, not the bench.
assert ParityFramedEncoder.CMD_POSITION == 0xFFFF
```

Cross-check every computed word against the datasheet's worked examples with a module-level `assert`. Register writes on such parts are two consecutive statement-position `xfer` words (command, then data).

### 4.8 On-device read-modify-write: `write_modify(reg, set_bits=…, clear_bits=…, param=…)`

Some registers carry undocumented factory state in their reserved bits, and their datasheets mandate "read the whole register first, change the desired bits only". A traced `configure()` read cannot satisfy that, because it is a compile-time mock. Hardcoding a full-register value zeroes bits the datasheet never documented. `write_modify` performs the read-modify-write **on-device at load**. It reads `reg` through the personality's `FRAME`, clears `clear_bits`, sets `set_bits`, and writes the result back with the frame CRC recomputed at runtime. The modified value never exists in the compiled image, so no unit's factory bits are baked into another's configuration.

```python
self.write(0x19, 0x0055)                     # fixed unlock word — plain write
self.write_modify(0x17, set_bits=0x1000)     # RMW: reserved bits survive
self.write_modify(0x14, set_bits=code, clear_bits=0b111,   # runtime-tunable field
                  param=("accel_fs", accel_fs))
```

Available in `probe()`/`configure()` on personalities with a `FRAME` whose trailing 8-bit CRC covers all preceding fields (both `standard` and `input-lsb` feedback). A call with neither `set_bits` nor `clear_bits` is rejected.

A `param=` tag makes the RMW field runtime-tunable: the OR `set_bits` immediate becomes a bytecode rewrite site. `nxs set` rewrites it and reloads, re-running the RMW with the reserved bits read fresh. Pass `clear_bits` as the **whole field mask**, the same for every value, so only the OR immediate varies (a single stable site). The compiler rejects `set_bits` outside `clear_bits`. A field split across two registers tags each `write_modify` with the same param and owns two sites. Without `param=`, `set_bits` and `clear_bits` must be disjoint.

### 4.9 Companion I²C devices: `I2C_COMPANIONS` + `dev=`

A multi-die package exposes two independent I²C targets on one bus: a primary die (strap-scanned via `I2C_ADDRS`) plus a companion die at a fixed address. Their register maps often overlap by number. Address topology assigns the roles. The strap-selectable die is the primary, and only the primary is scanned. The fixed-address die is the companion, and a companion has exactly one fixed address. That holds regardless of which die carries the stronger identity register.

One package is one personality. The companion is declared class-level and reached per access with the `dev=` keyword on `read`/`write`/`read_burst`, in `probe()`, `configure()`, and `measure()`. Each access is bracketed: the compiled program retargets to the companion, performs the access, and restores the primary. The bus therefore always rests at the primary. Every companion carries its own identity anchor, checked at probe time with a distinct error code (`0xC1`, the primary's is `0xC0`), or a documented `who_am_i_skip_reason`.

Requires `BUSES = ('i2c',)`. A `param=`-tagged write is keyed per die, so register numbers may overlap across dies.

```python
from nxs import RegisterClickPersonality, Sample


class TwoDieCombo(RegisterClickPersonality):
    BUSES = ('i2c',)
    I2C_ADDRS = [0x30, 0x31]
    WHO_AM_I_REG = 0x00
    WHO_AM_I_VALUES = [0x21]
    I2C_COMPANIONS = {
        'aux': {'addr': 0x0D, 'who_am_i_reg': 0x0F,
                'who_am_i_values': [0x33]},
    }

    def __init__(self):
        super().__init__()
        self._read_responses = {0x00: [0x21], ('aux', 0x0F): [0x33]}

    def probe(self):
        who = self.read(self.WHO_AM_I_REG)
        assert who == 0x21

    def configure(self, config):
        self.write(0x1B, 0x82, dev='aux')     # companion init, bracketed
        self.set_output([
            {'name': 'accel_x', 'scale': 1.0},
            {'name': 'mag_x', 'scale': 1.0},
        ])
        self.set_sample_size(4)

    @RegisterClickPersonality.measure_loop(trigger="poll", sample_rate=50)
    def measure(self):
        raw = self.read_burst(0x0D, 2)
        self.read_burst(0x10, 2, into=2, dev='aux')
        return Sample(raw)
```

### 4.10 `read_analog(ch)` and `drive_pwm(freq, duty)`

`read_analog(ch)` samples the mikroBUS AN pad and returns the raw ADC count (u16), usable in expressions. Publish it through a named output field, `return Sample(voltage=raw)`, not a positional `Sample(raw)`. Unlike `read_burst`, `read_analog` stages its bytes off the sample buffer, so a positional commit ships empty bytes. `drive_pwm(freq=..., duty=...)`, called in `configure()`, drives the mikroBUS PWM pad and declares two live range parameters, `pwm_freq` at 500–25000 Hz and `pwm_duty` at 0–100 %. They are retunable at runtime via `nxs set` with no reload. The arguments are the initial drive applied when the personality loads.

### 4.11 Communication profiles and reset

A personality may declare up to 3 communication profiles, for example an I²C profile and an SPI profile with its framing. The unit picks the profile matching the wired bus at upload: one personality, either bus, no recompile. `RESET_ACTIVE = "high" | "low"` declares the sensor's reset polarity, and the unit pulses the mikroBUS RST line accordingly at bind.

A `FRAME` may declare the protocol's integrity fields: an in-frame CRC (`crc=`, both `standard` and `input-lsb` feedback styles) and/or a return-status field (`status_ok=(field, ok_value)`). Every harvested measure response is then verified on-device. The CRC is recomputed and compared against the received byte, and the status field is mask-compared against the ok value. A mismatching frame (corruption, or an unprepared response) drops that tick instead of publishing. Sustained failure produces no samples and surfaces through the sample watchdog as a VM error.

Declare every integrity field the datasheet defines. The covered window, the CRC field, and the status field must be byte-aligned, and the compiler rejects a frame it cannot verify.

### 4.12 Hard limits

The compiler enforces these limits with named errors:

- bytecode ≤ 4096 bytes
- serialized image ≤ 6144 bytes (header + metadata + bytecode, aggregate)
- descriptor trailer ≤ 2048 bytes
- ≤ 16 output fields
- sample ≤ 128 bytes (≤ 124 when the loop also uses a value-read, which stages in the last 4 bytes)
- names ≤ 16 bytes
- units ≤ 8 bytes
- ≤ 16 values per parameter
- ≤ 2 patch sites per parameter
- ≤ 3 profiles
- companion devices are I²C-only, each with a 7-bit fixed address

One interface constraint sits outside the compiler. A sample above 110 bytes is valid but exceeds the I²C sample record's data span, so it streams over Cyphal only ([Interface Description §6.3](nxs-host-interface.md)).

### 4.13 Sensor-class conventions

Parameter names are canonical across personalities, so hosts script one vocabulary. Rate is `sample_rate`. On a UART stream personality, whose poll cadence is personality-pinned, the receiver's own epoch rate is `rate`. Full-scale is `accel_fs` / `gyro_fs` / `mag_fs`. Filter bandwidth is `accel_bw` / `gyro_bw`, or `filter_hz` for a single joint knob. A magnetometer's independent rate and resolution are `mag_odr` / `mag_res`.

A barometer's oversampling is `osr`, and an environmental part's repeatability mode is the `precision` config key. Parameter values are physical magnitudes (g, dps, Hz), never register codes.

A GNSS receiver with a documented binary protocol publishes typed geodetic fields at their record offsets as the default protocol. The fields are `latitude`, `longitude`, `altitude`, `vel_north` / `vel_east` / `vel_down`, `pos_h_acc`, `pos_v_acc` and `vel_s_acc`, under the per-subject rules of [Interface Description §8.2.8](nxs-host-interface.md). Fix type and satellite count are generic integer fields consumers gate on. Ground speed is `speed` in m/s, and heading is `heading` in rad. NMEA, when the part speaks it, is a secondary variant behind the `protocol` config key, delivered as a single string field.

The canonical `altitude` is height above **mean sea level (MSL)**, matching the geodetic subject's DSDL definition. A receiver's ellipsoidal (WGS84) height is published as the generic field `alt_ellipsoid`.

A barometer publishes `pressure` in pascal and its compensation `temperature` in kelvin. Altitude is not computed on-device, because the sea-level reference that turns pressure into metres is the consumer's mission state.

### 4.14 Cam personalities

A cam personality compiles the same way, from its `.py`, its `.yaml` and its register tables, into an image of kind CAMERA ([Interface Description §10.1](nxs-host-interface.md)). The unit runs it once per `CAM_RUN` on the pod-side bus and halts, while the click personality keeps running ([Interface Description §6.11](nxs-host-interface.md)). The YAML is the digitized datasheet. It holds the identity and I²C address, the register and value widths, the register vocabulary, and the modes with their geometry and timing. It also holds the law family (`sony_imx`, or `generic` for a table-only part) with its parameters, and the capture facts the host's device tree needs. The Python file is the behaviour, on a fictional part:

```python
from nxs import CamPersonality, I2cProfile

INIT = [(0x0100, 0x0000), (0x0101, 0x0010), (0x0102, 0x0003)]
MODE_720P = [(0x0200, 0x0500), (0x0201, 0x02D0), (0x0202, 0x0640)]
MODE_1080P = [(0x0200, 0x0780), (0x0201, 0x0438), (0x0202, 0x08CA)]


class Wide16(CamPersonality):
    I2C_ADDRS = [0x36]
    I2C_PROFILE = I2cProfile(addr_bytes=2, data_width=2, byte_order="big")
    TICKS_PER_US = 37

    def probe(self):
        self.poll(0x0000, 0xFFFF, 0x1616, timeout_ms=200, poll_ms=10)

    def configure(self, config):
        self.declare_param("mode", values=[0, 1], default=0)      # 720p, 1080p
        self.declare_param("trigger", values=[0, 1], default=0)   # freerun, fast
        self.declare_param("line_time", values=[9000, 40000], default=9805,
                           param_type="range", unit="ns", kind="live")
        self.write_table(INIT)
        self.select("mode", {
            0: lambda: self.write_table(MODE_720P),
            1: lambda: self.write_table(MODE_1080P),
        })
        self.select("trigger", {
            0: lambda: self.write(0x0300, 0x0001),
            1: lambda: self.write(0x0300, 0x0003),
        })
        ticks = self.param("line_time") * self.TICKS_PER_US // 1000
        self.write_wide(0x0340, ticks, 2)
        self.store_param("line_time", ticks * 1000 // self.TICKS_PER_US)
```

`probe()` is the alive or identity check. `configure()` is the shared init, then one block per value of each enum parameter a `select()` names. The compiler emits a load of the value staged for the parameter and a compare-and-jump chain into the blocks. A run with a staged `mode` value therefore switches modes without a recompile. `select_grouped()` takes groups of values (`{frozenset({a, b}): block, …}`) and emits each block once, every value of a group jumping into it. That is the shape of an action that shares a tail with the whole program.

It is also the shape of a dispatch whose registers are free again inside its blocks, so nested dispatches cost the blocks nothing. A value the program computes from a staged parameter is a run value. `param()` reads it, `write_wide()` writes a result to a register and `store_param()` records what the run achieved. Parameter values are integers on the wire. The descriptor trailer the compiler writes from the YAML maps each mode's name to its value, and the tool stages by name from it ([Interface Description §6.11](nxs-host-interface.md)). There is no measure loop, and configure ends in `HALT`.

- `poll(reg, mask, value, timeout_ms, poll_ms=20, ne=False, soft=False, dev=None)`: read until the masked value matches (with `ne`, until it differs) or the deadline passes. A NAK counts as not yet. A timeout faults the run, or with `soft` sets the run's soft-miss flag and continues. `dev` polls a declared companion.
- `check(reg, value, mask=0xFF, dev=None)`: one read of a byte register, faulting the run with `MISMATCH` unless the masked value matches. It is a write's read-back.
- `retry(times, delay_ms, block, until, timeout_ms, poll_ms=20)`: runs `block` (a callable emitting writes) and waits up to `timeout_ms` for `until`, a `(reg, mask, value)` condition on a byte register. When the wait runs out it waits `delay_ms` and runs the block again, `times` runs in all. The last wait faults with the register's name. That is the shape of a one-shot that has to re-lock before the next write.
- `write_table(rows)`: writes `(reg, value)` rows in order, and consecutive registers become one burst. `load_table(path)` reads such rows from a table file beside the source at compile time. The file is a YAML list of `[reg, value]` pairs, a `reg: value` mapping, or a CSV, and it is where a vendor setting file goes.
- `param(name)`: the value staged for a declared parameter in this run, as a run value. An enum parameter reads as its integer. `+ - * //` with another run value or an integer build an expression in unsigned 32-bit arithmetic. The compiler carries the interval the declared ranges allow through every intermediate. It refuses an operation that can leave 0..2³²−1 or divide by zero.
- `write_wide(reg, value, width, byte_order="big")`: writes the low `width` bytes (1 to 4) of a run value to the consecutive registers from `reg`. A value that can exceed the width is refused.
- `store_param(name, value)`: records a run value as the parameter's value, and the host reads it back once the run ends ([Interface Description §6.11](nxs-host-interface.md)). It may sit outside the staged range by the program's rounding.
- `I2cProfile(addr_bytes=2)` lifts the 8-bit register cap. `data_width` (1, 2, or 4) and `byte_order` frame every value the program moves.

The class carries behaviour only. Exposure, frame rate, and bandwidth laws run on the host from the family the YAML names. A deployed robot holds the compiled image and not the source, so a host-side hook on a camera class is a compile error. The compiler prints the budget with the image: bytes per block, the dispatch overhead per `select()`, and the descriptor trailer against its cap. The `nxs-generate-cam-personality` skill and the [Cam Personality Reference](nxs-cam-personalities.md) carry the YAML schema and the worked example.

## 5. Compile, upload, persist

```sh
nxs upload ./my_sensor/my_sensor.py
nxs store save 0
nxs upload ./my_sensor/my_sensor.py -o my_sensor.nxs
nxs personality install ./my_sensor
```

`upload` compiles, uploads, and runs. `store save 0` persists the personality to auto-load at boot, and `-o` compiles to an artifact for fleet provisioning. A cam personality (§4.14) takes the camera form, `nxs <port> <link> upload <name>`. The upload saves the image into a store slot on the link's pod, and the camera verbs run it from there. The sensor runner never runs it, so no `store save` follows.

`personality install` copies the click personality into the personality store, `/opt/aliensense/personalities/<name>/`. There `personality: my_sensor` in suite.yaml and `nxs upload my_sensor` resolve it before the shipped set. The store is root-owned, so the copy goes through `sudo`, which asks for the password at a terminal. `nxs personality check` judges the click personality first. A bare `nxs status` counts the shipped and the installed personalities on its `personalities:` line.

A compiled `.nxs` artifact is bound to the image format it was built for. After a product-MINOR firmware update, rebuild artifacts with the matching tool. The tool refuses a stale artifact and prints the rebuild command.

## 6. Stability

The DSL surface in §4 is a public API from product version 1.0. Additions (new verbs, new canonical semantics) arrive in MINOR or PATCH releases and never invalidate an existing personality source. A change that would break compiled images is a product-MINOR event, an image format event. A change that would break personality *sources* is a product-MAJOR event. Versioning model: [Device Reference §5](nxs-device-reference.md).
