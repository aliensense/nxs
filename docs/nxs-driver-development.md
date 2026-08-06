# NXS — Driver Development Guide

Applies to: NXS v1.0 · image format 1.0 · `nxs` tool 1.0.x

| Document set | |
|---|---|
| [Datasheet](nxs-datasheet.md) | electrical, pinout, performance, supported sensors |
| [Interface Description](nxs-host-interface.md) | transports, register map, commands, procedures |
| [Integration & Operation Manual](nxs-integration-manual.md) | design-in, host setup, workflows |
| **Driver Development Guide** (this document) | authoring drivers for unsupported sensors |

## 1. The authoring model

An NXS driver is a Python class that *describes* a sensor — its identification, configuration registers, parameters, sampling loop, and output fields. The `nxs` tool traces the class and compiles it to a bytecode image the on-device virtual machine executes; the driver never runs as Python on the module. Authoring is therefore datasheet transcription: register addresses, conversion formulas, and timing move from the sensor datasheet into a declarative class.

Because the input is a datasheet and the output is a constrained, verifiable artifact, the primary authoring flow is **AI-assisted**: provide the sensor datasheet to a coding agent, and this guide is the specification the agent follows — the class shape, the DSL contract, and the validation gates below. The `nxs-generate-sensor-driver` skill shipped with the product is the reference implementation of that flow. Hand-writing a driver against the same contract is equally supported; the compiler and the validation contract do not distinguish the two.

Whichever path produced the driver, **the validation contract (§3) is the acceptance gate** — a driver is trusted because it passed verification, not because of how it was written.

## 2. A complete driver

```python
from nxs import RegisterDriver, Sample

class MySensor(RegisterDriver):
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

    @RegisterDriver.measure_loop(trigger="from_config")
    def measure(self):
        status = self.read(0x1E)
        if not (status & 0x01):
            return None                                      # not ready
        raw = self.read_burst(0x28, 6)
        return Sample(raw)
```

Compile and run it exactly like a shipped driver:

```sh
nxs upload ./my_sensor.py --param sample_rate=250
nxs stream --count 5
```

## 3. The validation contract

Verify every driver — generated or hand-written — against the device before trusting it:

1. **Identity.** Declare `WHO_AM_I_REG` + `WHO_AM_I_VALUES` from the datasheet — the compiler auto-emits an identity prologue into the bytecode, and the wrong sensor halts the VM with a mismatch error surfaced by `nxs status` instead of streaming garbage. `probe()`'s assert runs at compile (trace) time against the seeded responses, not on the device; a sensor with no identity register must declare `WHO_AM_I_VALUES = []` with a `WHO_AM_I_SKIP_REASON`. Mind the set's provenance: values the datasheet lists are normal evidence, but a set derived from a fixed-bits claim ("bits n:m vary, the rest read 0") is weak — silicon ships with "fixed" bits set — so corroborate it against a vendor or kernel reference driver, and never include `0x00`/`0xFF` without justification (they also match empty reads, false-accepting a wrong die). On a multi-die package, put the hard identity on the die with a fixed WHO_AM_I (the companion check) when the primary's product code is factory-variable.
2. **Parameters against the datasheet.** Every `declare_param` value set matches a datasheet table (full-scale ranges, ODR dividers, oversampling ratios), and every config-dependent register write carries its `param=` tag so runtime `set` works without re-upload.
3. **Units are canonical and enforced.** Fields whose names infer an SI semantic (`accel_*`, `gyro_*`, `mag_*`, `temp`, `pressure`, `angle`, `voltage`, …) inherit their canonical unit at compile time — `m/s^2`, `rad/s`, `tesla`, `kelvin`, `pascal` — and a conflicting declared unit is a compile error. The full recognized set, with each name's SI unit and standard subject, is the canonical-units table in §4.3 (single-sourced from `constants/field_semantics.yaml`). Temperature is kelvin: fold the datasheet's Celsius zero point into the field's `offset` (+273.15). Fields with no SI semantic must declare a unit (`''` for unitless).
4. **Independent decode.** Stream from the device and recompute at least one field by hand from the datasheet formula — raw counts × datasheet sensitivity — and compare against the device-served decode. The two derivations must agree; this catches transposed scales that all other gates miss.
5. **Physical plausibility.** A stationary accelerometer reads ≈ 9.81 m/s² on one axis; ambient temperature ≈ 293–300 K; ambient pressure ≈ 101 kPa; Earth's magnetic field ≈ 25–65 µT.
6. **Parameter round-trip.** `set` a full-scale range live and confirm the streamed magnitude is unchanged for the same physical stimulus (the scale retunes with the parameter).

## 4. DSL reference

### 4.1 Driver classes

| Base class | Sensor type | I/O verbs |
|---|---|---|
| `RegisterDriver` | register-mapped I²C/SPI (IMUs, magnetometers) | `read`, `write`, `write_modify`, `read_burst`, `xfer` |
| `I2cCommandDriver` | command-response I²C (baros, hygrometers) | `send_command` + a declarative command/delay/CRC frame pattern |
| `StreamDriver` | UART streams (GNSS/NMEA) | `write`, `read`, `available`, `read_until`, `set_baud` |

### 4.2 `declare_param(name, values, default, param_type="enum", unit="", kind="reload")`

Declares a runtime-settable parameter (name ≤ 16 bytes, unit ≤ 8 bytes). `param_type="enum"`: `values` is the allowed set, ≤ 16 entries. `param_type="range"`: `values` is `[min, max]`, validated inclusively on each set; a range parameter must be `kind="live"` — a range has no bytecode patch site to re-apply on reload, and the firmware rejects a non-live range at load. Both shapes are validated at compile time — the declared default and any config override included — and runtime-settable via `nxs set`. `kind="reload"` re-runs `configure()` on set — correct for sensor config registers. `kind="live"` applies without a restart, for values a running consumer reads continuously. Every parameter is visible to hosts with its allowed set or bounds and default (`nxs caps`, `GetParamInfo`).

The compiler enforces the patch contract so a `set` is never a silent no-op: a param is declared before its `param=` tag and the tagged value is in the declared set; a param patches at most `MAX_PATCH_SITES` bytecode sites (a setting written to two registers owns two — a direct `write` still binds one register to one param, but read-modify-write sites don't clobber and may share); the set of patch sites must not change between values; and a `reload` enum that patches no bytecode, or a `live` param that no runtime consumer reads (only the PWM pair or an output `scale_param`), is rejected. Patch width follows the write — one byte for `write`, four for `set_baud` and a `write_modify` field, `n` for `stage(size=n)` — and every value must fit it.

Filter bandwidth is a runtime parameter whenever the part has one: declare the values as physical cutoff frequencies in Hz (never register codes), map each to its code in a lookup table, and tag the write. Codes that change the internal base rate the rate divider divides are excluded — such a code re-times every declared `sample_rate` value rather than selecting a filter.

### 4.3 `set_output(fields)` and canonical units

Each field: `{'name', 'scale', 'offset'?, 'unit'?, 'type'?, 'byte_order'?, 'scale_param'?}`. The field's *name* selects its semantic; the semantic selects the canonical unit and, over Cyphal, the standard `uavcan.si.sample.*` subject:

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

Omit `unit` for these fields — the compiler inherits the canonical unit and rejects a conflicting declaration. Matching is by exact name, case-insensitive, after alias folding: the full semantic names (`temperature`, `frequency`, `mass`, `distance`, `flow`) and the aliases `temp`, `freq`, `weight`, `range`, and `flow_rate` all infer the same semantics as their table entries. Names outside that set are *generic*: they stream with full self-description (name, type, scale/offset, declared unit) but claim no standard subject. `scale_param` names a declared parameter whose live value multiplies the base `scale` on-device — the idiom for runtime-tunable full-scale ranges.

Fields pack sequentially by default; `'at': N` places a field at an explicit byte position within the sample — the binary-record idiom, where fields map onto scattered offsets inside a captured frame and the gaps (headers, reserved bytes, checksums) carry no fields. Explicit placement is all-or-none per driver; overlapping fields, a field past the sample buffer, or more fields than the descriptor table holds are compile errors. Every descriptor carries its resolved byte position on the wire, so hosts decode each field at its declared offset rather than accumulating widths.

### 4.4 `@measure_loop(trigger=...)`

`trigger="drdy"` waits on the sensor's data-ready line; `trigger="poll", sample_rate=N` paces at a fixed rate. `trigger="from_config"` defers the choice to the configuration: the config's `trigger` key selects `drdy` or `poll` at compile time, and the `sample_rate` parameter paces the polling case — the idiom for sensors that support both modes. A part with any data-ready output — including a fixed-rate sync with no divider — uses `trigger="from_config"` with `drdy` as the default; hardcoded `poll` is for parts with no such pin and for bus-paced protocols (UART streams, command-response conversions). On a poll-paced driver a declared `sample_rate` with no rate register patches the loop's sleep interval, so `set sample_rate` retunes the cadence with no register write. A **fixed-sync part** (data-ready at a fixed hardware rate, no divider register) additionally declares `drdy_base_hz=<sync Hz>` on the decorator: the compiled drdy loop then paces by dividing the sync at the source — the module counts hardware edges in the interrupt and wakes the loop every `base/rate`-th edge, so the delivered spacing is exact against the sensor's own clock even while a read burst spans many edges. Every declared `sample_rate` value must divide the base exactly (compile error otherwise), and the same declaration paces poll mode via the sleep interval — one parameter, both modes. The body reads raw bytes and returns them (or `Sample(field=value)` for computed values — on-device compensation runs in 64-bit fixed point). Returning `None` skips the tick.

The body executes on-device; `configure()`, by contrast, is traced at compile time against mock reads, so a value read in `configure()` is a placeholder — a runtime conditional on it is rejected and belongs here in `measure()`, and a register read-modify-write is `write_modify` (§4.7), whose read happens on-device at load. The body compiles to VM bytecode and resolves integer **literals** (write registers/masks as hex with a naming comment) and **UPPER_CASE class-level integer constants** (`self.CMD_X` — the home for computed wire words); any other `self.*` value is rejected, except `configure()`-bound coefficients. Reads come in two shapes: `read_burst(reg, count[, into=off])` *places* bytes in the sample buffer at `into` (default 0; a second bank takes a distinct `into=`, and an overlap is a compile error), while `read(reg, width[, signed=, endian=])` returns a *value* into a register — unsigned big-endian by default, `signed=True` for signed fields and `endian="little"` for little-endian parts. Value-reads stage off the sample buffer, so they never corrupt placed data. `sleep_ms(n)` and `sleep_us(n)` pace conversions and inter-word gaps. Control flow supports `if`/`elif`/`else` and the integer comparisons `== != < > <= >=`; loops and function calls other than the `self.*` verbs are rejected at compile time.

A driver whose protocols need *different framing bytecode* — which a runtime parameter cannot patch — declares one measure loop per protocol, each tagged `when=("<config key>", <value>)`, with exactly one marked `default=True`; `compile(config)` picks the variant the config names (an unknown value, mixed tagged/untagged loops, or a missing default are compile errors). `configure()` branches on the same key in plain Python. The key is a compile-time configuration, not a runtime parameter: switching protocols means uploading the other configuration (or storing both in driver-store slots and cycling).

### 4.5 Data path: direct reads vs the on-chip FIFO

Direct register reads and the on-chip FIFO use the same verbs; a wire-time budget chooses between them. The declared `sample_rate` values are a contract — every value must hold on every declared bus — so budget one measure pass on the slowest declared bus at the fastest declared rate:

```
t_pass ≈ 9 × (N + 8) / f_i2c      # status gate + addressing + N-byte burst
t_pass ≈ 8 × (N + 4) / f_spi      # the same pass over SPI
```

`N` is the sample size in bytes; compare against the sample period with 3× headroom (DRDY wake latency, other traffic on the shared bus, the next update landing mid-burst). `t_pass ≤ period/3` → status-gated direct burst. `t_pass > period/3` and the part has a *headerless* FIFO (fixed packet layout; a per-frame-header FIFO is not consumable by the VM) → read the FIFO: it freezes each sample's bytes, so a burst cannot tear, and a late DRDY service grows the queue instead of dropping a sample. `t_pass > period/3` with no usable FIFO → cap the declared `sample_rate` values at the fastest rate that fits the budget. The FIFO relaxes the deadline, not the bus: its own pass must still fit the full period. Poll-paced parts run the same check with conversion sleeps included and no 3× headroom.

In `configure()`, queue the channels and flush-and-enable; in `measure()`, read one packet per pass through a tiered count gate (illustrative FIFO registers):

```python
def configure(self, config):
    self.write(0x40, 0xF8)              # FIFO_EN: queue the channels, in packet order
    self.write(0x41, 0x44)              # USER_CTRL: FIFO_RST | FIFO_EN — flush + enable
    self.set_sample_size(14)            # one packet

@RegisterDriver.measure_loop(trigger="from_config")
def measure(self):
    count = self.read(0x42, 2)          # FIFO_COUNTH:L — bytes buffered
    if count < 14:
        return None                     # frame incomplete — wait for the next DRDY
    if count > 56:
        self.write(0x41, 0x44)          # >4 frames behind — resync
        return None
    raw = self.read_burst(0x46, 14)     # FIFO_R_W — one frozen packet
    after = self.read(0x42, 2)
    left = count - 14
    d = after - left
    if d == 0:
        return Sample(raw)              # clean read
    d = d - 14
    if d == 0:
        return Sample(raw)              # one frame arrived mid-read
    self.write(0x41, 0x44)              # torn read — drop + resync
    return None
```

Two properties of the FIFO path follow from the same physics. **Phase**: on parts whose serial engine shares timing with the internal sample path, a transaction that lands on the sample-update instant can be NACK'd, sometimes with SDA held for a few milliseconds after. A DRDY-triggered pass is phase-locked just after an update; poll pacing samples the hazard at random, and even a DRDY loop de-phases once the pass plus the system's per-sample work outgrows the period — the wire-time budget above is a correctness bound, not a preference. The runner absorbs sporadic hits (bus recovery, then an in-place retry of the faulted op while the FIFO holds the queue), so an isolated NACK costs milliseconds and no samples; a NACK rate that tracks pacing means the budget is violated. **Resync loss**: the mid-stream resync above drops the buffered frames — the correct trade at the gate's depth. A driver that must not lose them can instead drain `count % frame_size` bytes to realign and consume every whole frame.

The gate reads through a bounded backlog — a two-frame backlog is two valid samples, not an error — and resets only when the queue runs away or the after-count shows the read missed a frame boundary. The literals (`14` packet bytes, `56` = 4 × 14) follow from the enable mask.

**Binary-record streams.** A UART part that emits framed binary records (sync bytes, a fixed header, little-endian payload fields, a trailing Fletcher checksum) publishes *typed fields at their record offsets* instead of a string blob — the fields then carry SI semantics and project onto the standard subjects. The shape: sync with `read_until` (the sync bytes are consumed, the record lands at offset 0), capture the fixed-length record with `read_n`, gate the header with `match(...)` (mismatch count of the leading bytes against constants), verify the checksum with `verify_checksum(ChecksumFletcher(), start_off, length, ck_off)` — the RX counterpart of `compute_checksum`, comparing the recomputed bytes against the received ones at `ck_off` — then commit the fixed-length record with `store_sample()`, which publishes the declared `set_sample_size`. A variable-length record read with `read_until` commits with `store_sample_n()` instead. Both gates return a count usable in `if`, and a nonzero means drop the record and resync on the next pass.

```python
from nxs import StreamDriver, SensorDriver
from nxs.compiler import ChecksumFletcher


class BinaryRecordSensor(StreamDriver):
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

    @SensorDriver.measure_loop(trigger="poll", sample_rate=100)
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

### 4.6 Literal SPI words: `xfer(word, width=2)`

Some SPI parts place *computed* bits inside the command word itself — a parity bit over rw+address is the classic case — so their words do not decompose into `FRAME` fields. Every such word is a compile-time constant: compute it with plain Python at class definition time, store it as an UPPER_CASE class constant, and clock it verbatim with `x = self.xfer(word)`. The word's bytes go on the wire MSB-first with one chip-select assertion per word, and the response loads unsigned MSB-first at the full `width` (1, 2, or 4 bytes; default 2); in statement position the response is discarded — the pipeline-priming idiom for parts whose response to word N arrives during word N+1. `xfer` requires `BUSES = ('spi',)`. Response checks are ordinary `measure()` arithmetic: parity folds as two-statement shift/xor steps, error flags as masked gates, each violation dropping the sample with `return None`.

```python
from nxs import RegisterDriver, Sample, SpiProfile


def _read_word(addr):
    """Read command word: parity(15) | rw=1(14) | addr(13:0), even
    parity over bits 14:0 — evaluated at class definition."""
    word = (1 << 14) | addr
    return word | ((bin(word).count("1") & 1) << 15)


class ParityFramedEncoder(RegisterDriver):
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

    @RegisterDriver.measure_loop(trigger="poll", sample_rate=100)
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

### 4.7 On-device read-modify-write: `write_modify(reg, set_bits=…, clear_bits=…, param=…)`

Some registers carry undocumented factory state in their reserved bits, and their datasheets mandate "read the whole register first, change the desired bits only". A traced `configure()` read cannot satisfy that — it is a compile-time mock — and hardcoding a full-register value zeroes bits the datasheet never documented. `write_modify` performs the read-modify-write **on-device at load**: it reads `reg` through the driver's `FRAME`, clears `clear_bits`, sets `set_bits`, and writes the result back with the frame CRC recomputed at runtime. The modified value never exists in the compiled image, so no unit's factory bits are baked into another's configuration.

```python
self.write(0x19, 0x0055)                     # fixed unlock word — plain write
self.write_modify(0x17, set_bits=0x1000)     # RMW: reserved bits survive
self.write_modify(0x14, set_bits=code, clear_bits=0b111,   # runtime-tunable field
                  param=("accel_fs", accel_fs))
```

Available in `probe()`/`configure()` on drivers with a `FRAME` whose trailing 8-bit CRC covers all preceding fields (both `standard` and `input-lsb` feedback). A call with neither `set_bits` nor `clear_bits` is rejected.

A `param=` tag makes the RMW field runtime-tunable: the OR `set_bits` immediate becomes a bytecode patch site (`nxs set` rewrites it and reloads, re-running the RMW with the reserved bits read fresh). Pass `clear_bits` as the **whole field mask** — the same for every value — so only the OR immediate varies (a single stable site); the compiler rejects `set_bits` outside `clear_bits`. A field split across two registers tags each `write_modify` with the same param and owns two sites. Without `param=`, `set_bits` and `clear_bits` must be disjoint.

### 4.8 Companion I²C devices: `I2C_COMPANIONS` + `dev=`

A multi-die package exposes two independent I²C slaves on one bus — a primary die (strap-scanned via `I2C_ADDRS`) plus a companion die at a fixed address, often with register maps that overlap by number. Address topology assigns the roles: the strap-selectable die is the primary (only the primary is scanned; a companion has exactly one fixed address), and the fixed-address die is the companion, regardless of which die carries the stronger identity register. One package is one driver: the companion is declared class-level and reached per access with the `dev=` keyword on `read`/`write`/`read_burst`/`write_burst`, in `probe()`, `configure()`, and `measure()`. Each access is bracketed — the compiled program retargets to the companion, performs the access, and restores the primary — so the bus always rests at the primary. Every companion carries its own identity anchor, checked at probe time with a distinct error code (`0xC1`; the primary's is `0xC0`), or a documented `who_am_i_skip_reason`. Requires `BUSES = ('i2c',)`; a `param=`-tagged write is keyed per die, so register numbers may overlap across dies.

```python
from nxs import RegisterDriver, Sample


class TwoDieCombo(RegisterDriver):
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

    @RegisterDriver.measure_loop(trigger="poll", sample_rate=50)
    def measure(self):
        raw = self.read_burst(0x0D, 2)
        self.read_burst(0x10, 2, into=2, dev='aux')
        return Sample(raw)
```

### 4.9 `read_analog(ch)` and `drive_pwm(freq, duty)`

`read_analog(ch)` samples the mikroBUS AN pad and returns the raw ADC count (u16), usable in expressions. Publish it through a named output field — `return Sample(voltage=raw)`, not a positional `Sample(raw)`: unlike `read_burst`, `read_analog` stages its bytes off the sample buffer, so a positional commit ships empty bytes. `drive_pwm(freq=..., duty=...)` — call in `configure()` — drives the mikroBUS PWM pad and declares two live range parameters, `pwm_freq` (500–25000 Hz) and `pwm_duty` (0–100 %), retunable at runtime via `nxs set` with no reload; the arguments are the initial drive applied when the driver loads.

### 4.10 Communication profiles and reset

A driver may declare up to 3 communication profiles (e.g. an I²C profile and an SPI profile with its framing); the module picks the profile matching the wired bus at upload — one driver, either bus, no recompile. `RESET_ACTIVE = "high" | "low"` declares the sensor's reset polarity; the module pulses the mikroBUS RST line accordingly at bind.

A `FRAME` that declares the protocol's integrity fields — an in-frame CRC (`crc=`, both `standard` and `input-lsb` feedback styles) and/or a return-status field (`status_ok=(field, ok_value)`) — gets every harvested measure response verified on-device: the CRC is recomputed and compared against the received byte, the status field mask-compared against the ok value. A mismatching frame (corruption, or an unprepared response) drops that tick instead of publishing; sustained failure produces no samples and surfaces through the sample watchdog as a VM error. Declare every integrity field the datasheet defines — the covered window, the CRC field, and the status field must be byte-aligned, and the compiler rejects a frame it cannot verify.

### 4.11 Hard limits

Bytecode ≤ 4096 bytes · serialized image ≤ 6144 bytes (header + metadata + bytecode, aggregate) · ≤ 16 output fields · sample ≤ 128 bytes (≤ 124 when the loop also uses a value-read, which stages in the last 4 bytes) · names ≤ 16 bytes · units ≤ 8 bytes · ≤ 16 values per parameter · ≤ 2 patch sites per parameter · ≤ 3 profiles · companion devices are I²C-only, each with a 7-bit fixed address. The compiler enforces all of them with named errors. One interface constraint sits outside the compiler: a sample above 110 bytes is valid but exceeds the I²C sample record's data span, so it streams over Cyphal only ([Interface Description §6.3](nxs-host-interface.md)).

### 4.12 Sensor-class conventions

Parameter names are canonical across drivers, so hosts script one vocabulary. Rate is `sample_rate` (on a UART stream driver, whose poll cadence is driver-pinned, the receiver's own epoch rate is `rate`); full-scale is `accel_fs` / `gyro_fs` / `mag_fs`; filter bandwidth is `accel_bw` / `gyro_bw`, or `filter_hz` for a single joint knob; a magnetometer's independent rate and resolution are `mag_odr` / `mag_res`; a barometer's oversampling is `osr`; an environmental part's repeatability mode is the `precision` config key. Parameter values are physical magnitudes (g, dps, Hz), never register codes.

A GNSS receiver with a documented binary protocol publishes typed geodetic fields at their record offsets as the default protocol — `latitude`, `longitude`, `altitude`, `vel_north` / `vel_east` / `vel_down`, `pos_h_acc`, `pos_v_acc`, `vel_s_acc` (the per-subject rules: [Interface Description §8.2.8](nxs-host-interface.md)) — with fix type and satellite count as generic integer fields consumers gate on, ground speed as `speed` (m/s), and heading as `heading` (rad). NMEA, when the part speaks it, is a secondary variant behind the `protocol` config key, delivered as a single string field. The canonical `altitude` is height above **mean sea level (MSL)**, matching the geodetic subject's DSDL definition; a receiver's ellipsoidal (WGS84) height is published as the generic field `alt_ellipsoid`.

A barometer publishes `pressure` (pascal) and its compensation `temperature` (kelvin); altitude is not computed on-device — the sea-level reference that turns pressure into metres is the consumer's mission state.

## 5. Compile, upload, persist

```sh
nxs upload ./my_sensor.py
nxs store save 0
nxs upload ./my_sensor.py -o my_sensor.nxs
```

`upload` compiles, uploads, and runs; `store save 0` persists the driver to auto-load at boot; `-o` compiles to an artifact for fleet provisioning.

A compiled `.nxs` artifact is bound to the image format it was built for; after a product-MINOR firmware update, rebuild artifacts with the matching tool (the tool refuses a stale artifact and prints the rebuild command).

## 6. Stability

The DSL surface in §4 is a public API from product version 1.0. Additions (new verbs, new canonical semantics) arrive in MINOR or PATCH releases and never invalidate an existing driver source; a change that would break compiled images is a product-MINOR (image format) event, and one that would break driver *sources* is a product-MAJOR event. Versioning model: [Datasheet §9](nxs-datasheet.md).
