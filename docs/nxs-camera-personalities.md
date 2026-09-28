# NXS — Camera Personality Reference

Applies to: NXS SDK 1.1.x · `nxs cam` · descriptor pack API 1 · NXS image format 2.0

| Document set | |
|---|---|
| [Technical Specifications](nxs-specifications.md) | capability summary tables |
| [Device Reference](nxs-device-reference.md) | interfaces, performance and limits, shipped personalities, versioning |
| [Interface Description](nxs-host-interface.md) | transports, register map, commands, procedures |
| [Integration & Operation Manual](nxs-integration-manual.md) | design-in, host setup, workflows |
| [Personality Authoring Reference](nxs-personality-authoring.md) | authoring personalities for unsupported sensors |
| **Camera Personality Reference** (this document) | describing camera chains for `nxs cam` |
| [MCP Tool Reference](nxs-mcp.md) | operating and configuring through an AI agent |

## 1. The authoring model

A camera chain is an image sensor behind a serializer behind a deserializer link, with an NXS unit on the sensor's pod. Three programs share the chain and none of them knows a sensor by name. The kernel drivers know what the device tree tells them: the sensor's address, its register and value widths, the modes, the control table. The unit holds the **camera personality**: the sensor's program, which it runs on the pod's bus when the host hands it the bus token, and the descriptors it serves back. The `nxs` tool holds the compiler, the deserializer and serializer choreography, the orchestration verbs, and the overlay generator that turns the unit's descriptors into the host's device tree. Every fact about a sensor comes from its personality: the I²C address, the register and value widths, the alive check, the modes, the controls, the laws, the capture facts.

Two things are authored against this document. A **descriptor pack** describes a carrier: the deserializer and serializer chips, the flows that assemble a stream, and the topology. A **sensor pair** describes an image sensor: `<name>.yaml`, the digitized datasheet, and `<name>.py`, the behaviour the unit runs, compiled by the same compiler as a unit driver into an NXS image of kind CAMERA ([Interface Description §10.1](nxs-host-interface.md)). A pack carries its sensors' pairs while they are authored; what ships to a robot is the compiled image, uploaded into the unit's store, and the pack that ships beside the tool carries the serdes chips only.

Everything in this document uses an invented hardware family (vendor `acme`). No value in any example is a real device's.

### 1.1 Relation to Linux conventions

A sensor descriptor and a Linux devicetree binding do the same job in the same schema language. Each states what a node of one `compatible` may carry, and each is json-schema: a binding in the kernel's YAML subset ([devicetree binding schemas](https://docs.kernel.org/devicetree/bindings/writing-schema.html)), a descriptor under `nxs/schemas/cam-descriptor.schema.json`. Where a descriptor key states a fact the kernel already names, the two map as follows:

| Descriptor | Linux |
|---|---|
| `meta.compatible` | the node's `compatible` |
| a mode's `geometry.lanes`, and `csi_lanes` on the port | the count of an endpoint's `data-lanes` ([video-interfaces.yaml](https://www.kernel.org/doc/Documentation/devicetree/bindings/media/video-interfaces.yaml)) |
| `capture.clock_noncontinuous` | the endpoint's `clock-noncontinuous`, same file |
| the frame law: line length and frame length against the clock | the frame interval from the blanking controls and the pixel rate ([camera sensor drivers](https://docs.kernel.org/userspace-api/media/drivers/camera-sensor.html)) |

A binding constrains one node. The tool also judges feasibility across nodes, such as a pair on one line or the lanes against the hub's output, and it answers a refusal with the lawful alternatives. The kernel still gets a standard description: the tool generates the devicetree overlay for the capture stack from the same facts (§3.1).

## 2. A complete pack

A pack is a directory. `nxs cam` searches, in order: every path in `$NXS_CAM_DESCRIPTORS` (colon-separated; read only under `nxs --experimental`, and every path must exist), the personality stores (a `personality install` writes an extension there), then the pack inside the wheel. The first pack whose `flows_for` list covers the topology's deserializer compatible wins; no pack found is an error naming the searched paths.

```
acmerig/
  pack.yaml            # the manifest
  topology.yaml        # the carrier's ports and links
  flows.py             # stream assembly (python)
  cam1/                # one directory per chip
    cam1.yaml          #   the datasheet: registers, modes, laws, capture facts
    cam1.py            #   the behaviour the unit runs: probe(), configure()
    cam1_init.yaml     #   register tables the behaviour loads
    cam1_mode_full.yaml
  ser1/  des1/         # serializer / deserializer: <chip>.yaml + <chip>.py (knobs)
```

`pack.yaml` declares the contents — unknown keys are rejected:

```yaml
pack: acmerig
api: 1                        # descriptor pack API this pack targets
chips: [cam1, ser1, des1]     # chip directories
flows: flows.py
flows_for: ["acme,des1"]      # deserializer compatibles these flows drive
topology: topology.yaml       # fallback port set; suite.yaml ports win
# golden:                     # optional byte-exact reference sequences
#   flow: dual                # flow the references prove
#   files: [golden/dual-base.yaml, golden/dual-addon.yaml]
```

Pack Python is loaded in an isolated module namespace with a `PACK` handle injected. Pack modules import **only** the public `nxs.cam` API (absolute imports) and reach their own data and sibling chips through the handle — `PACK.descriptor("cam1")`, `PACK.chip_module("des1")`. A pack never imports another pack.

The pack a release ships is the pack without its sensors: the deserializer and serializer chips, the flows, and the topology, comments stripped, inside the wheel as package data. Its sensors ship as sealed camera personalities in the personalities asset (`/opt/aliensense/personalities/<name>.nxs`, versioned with the wheel), and a unit serves its personality's descriptors back, so a host that never saw a sensor's source still checks a manifest, lists modes, and generates the device tree for it.

### 2.1 Extensions: a sensor the pack does not know

A directory whose `pack.yaml` says `extends: <pack>` is a pack extension — sensor pairs only, no flows:

```yaml
extends: acmerig
api: 1
chips: [cam9]                 # cam9/cam9.yaml + cam9.py + its tables beside this file
```

Its chip directories join the named pack as if they lived inside it; a more specific search path wins on a name clash. This is one of two ways to ship a personality for a sensor the pack does not know: drop the directory on `$NXS_CAM_DESCRIPTORS`, and `nxs upload cam9` compiles the pair. The other is the personality store: `nxs personality install ./cam9` copies the pair with its tables under `/opt/aliensense/personalities/`, where a name resolves before the pack ([Integration & Operation Manual §4.1](nxs-integration-manual.md)). The `nxs-generate-camera-personality` skill shipped with the SDK writes such a pair from a datasheet and a vendor setting file; an IMX335 pair generated that way writes the sensor identically to the shipped personality for every action, the settles and the alive timeout apart; the pair is authored against this document either way.

## 3. Chip descriptors

A chip's YAML is its fact sheet. The core keys are shown; `modes` applies to sensors, `windows` to deserializers. A pack may carry additional sections (sync roles, test patterns, a mode catalog) consumed by its own modules — the loader is permissive above the core schema (`nxs/schemas/cam-descriptor.schema.json`, named by the modeline).

```yaml
meta:
  compatible: acme,cam1       # the socket key: topology links match on this
  role: SEN                   # SEN / SER / DES
  i2c_addr: 0x36
  chip: generic               # the law family (sensors): generic, sony_imx
  reg_bits: 16                # register address width
  val_bits: 16                # value width per register
  provenance: ACME CAM1 datasheet r1.2

registers:                    # named registers: addr, width, order
  CTRL:  {addr: 0x1000, width: 1}
  TRIG:  {addr: 0x1004, width: 1}
  SPEED: {addr: 0x1010, width: 2, order: le}

modes:                        # sensor operating points, by name
  mode_full:
    geometry: {width: 1920, height: 1080, bit_depth: 10, lanes: 2, rate_mbps: 800}
    timing:   {speed: 500}
    mipi:     {data_type: RAW10, embedded_lines: 0}
    table:    cam1_mode_full.yaml   # the delta table the unit program writes for this mode
    serializer_csi: true            # the personality writes the serializer's CSI block for this mode

limits:                       # numeric laws the family enforces
  speed_max: 1000
limits_source: {speed_max: datasheet}

status:                       # declarative health probes for status / get
  - {name: running, reg: CTRL, decode: {0x00: "no", 0x01: "yes"}, desc: "run state"}
  - {name: speed,   reg: SPEED, format: int, desc: "line rate"}

runtime_forbidden: []         # registers no runtime verb may ever write
```

A deserializer adds its link-window map — the register values that direct control traffic at one link, both links, or broadcast:

```yaml
windows: {A: 0x11, B: 0x12, broadcast: 0x13}
runtime_forbidden: [0x0FFF]   # e.g. a soft-reset register: bring-up only
```

**Status probes** power three verbs from one declaration: `nxs cam0 status` renders every probe per chip, `nxs cam0 get <name>` serves a single one, and diagnostics walk them across the topology. `expect:` marks a probe pass/fail, `mask:` selects bits, `decode:` maps values to words, `format: int` prints the number. The unit carries them as the STATUS record (§4, record type 10): name, register, format, mask, expected value and decode table; the `desc` text stays in the pack, so a unit-served probe prints its name alone.

**Modes** name the program that selects them. A mode with a `table:` is offered by the personality; its value in the `mode` parameter is its index among the offered modes, in file order. A mode flagged `host_only: true` is known to the laws (`status`, `tune`) and offered by no program; the tool refuses to bring it up.

**`runtime_forbidden`** is the safety rail: registers that may appear in bring-up tables but must never be touched by a composed runtime flow. The composer scans every composed stream against it and refuses on a hit.

### 3.1 Sensor personalities

A sensor's directory is a **personality source**: the yaml above, the behaviour module (§4), and its register tables. Four sections make a sensor a full citizen of the stack.

**Identity.** A sensor with an identity register declares it; the tool detects the part through the link window, records it, and refuses a declaration the silicon contradicts. A sensor without one is declared only. A deserializer descriptor must declare its identity register: the identity read is the gate every write stands behind, so `on` refuses to program a hub the pack cannot verify and `probe` names what answered.

```yaml
meta:
  device_id_reg: 0x0000       # MSB first
  device_id: 0x1616
  device_id_width: 2          # bytes (1 when absent)
```

**The capture table.** The capture host's contract is generated from the descriptors: one capture-side mode per `capture.table` entry, in order. The sensor states what the capture stack must know about it; the host adds its own facts (lane count, port, virtual channel, lane polarity), and one overlay per port and lane count carries the table on both virtual channels. Entries are transcribed from the vendor's capture-side driver where one exists.

**The vendor's tuning override.** `capture.tuning` names the vendor's ISP override for the sensor, by `override_url` and `sha256`; the host fetches it once and folds it into the capture stack's tuning for the sensor's modes. A descriptor without it leaves those modes on the stack's defaults.

```yaml
capture:
  mclk_khz: 24000
  pix_clk_hz: 182400000       # the pixel clock the capture stack books exposure against
  clock_noncontinuous: true   # the sensor gates its clock lane between bursts
  gain: {factor: 16, min: 16, max: 170, step: 1, default: 16}
  hdr_ratio: {min: 1, max: 1}
  exposure: {factor: 1000000, max_us: 683709, step: 1, default_us: 2495}
  framerate: {factor: 1000000, min_fps: 2, step: 1}
  table:
    - {width: 1920, height: 1080, bit_depth: 10, pixel_phase: rggb, line_length: 3448, max_fps: 30, min_exp_us: 13}
```

A mode's table index, the number a capture session names the mode by, is its row's position in the generated table; nothing declares it, and the booted tree's index wins at run time.

A port's table carries the rows of one Bayer phase, the phase of the row with the longest exposure: the capture stack demosaics every mode of a node by the phase of the node's first mode. A row whose `pixel_phase` differs takes no index and boots on no node, and a declaration of its mode is refused with the phase named.

`exposure.max_us` is the sensor's own limit. A booted row's exposure ceiling is the smaller of that limit and the frame at the row's rate less a shutter margin of 32 lines, so no consumer's exposure loop asks for an exposure the frame cannot hold.

`clock_noncontinuous: true` says the sensor gates its clock lane between bursts. The clock is continuous when the key is absent. The key names the same fact as the kernel's `clock-noncontinuous` endpoint property (§1.1).

A mode flagged `serializer_csi` ends its unit program on the serializer: the personality reaches the pod's serializer as an I²C companion and writes, after the mode's tables, the CSI block that carries the mode's contract (PHY standby, lanes, packing and doubling, the data-type filter, the non-continuous clock). The host composes no serializer CSI block for such a link and keeps the link bring-up, which runs before the unit is reachable. The flag rides the MODES record, so a unit-served descriptor carries it.

**The program settles and the trigger presets.** `program:` carries the waits the family's timing program and the behaviour class use (a standby settle, a release settle, a start settle, per program), `trigger:` the register values of each conversion (`freerun`, `fast`), and `sync:` whether the head takes the hub's trigger pulse. The behaviour class reads them, so a settle is changed in one place.

```yaml
program:
  timing_start: {standby_ms: 200, release_ms: 50, start_ms: 600}
  trigger_switch: {standby_ms: 200, release_ms: 50, start_ms: 100}
  stop_ms: 300
  start: {release_ms: 20, start_ms: 100}
trigger:
  freerun: {trigmode: 0x00, vint_en: 0x1A}
  fast:    {trigmode: 0x0A, vint_en: 0x18}
sync: {takes_trigger: true, text: "XTRIG on the serializer's MFP pin"}
```

**Controls.** For a `generic` family part, `controls:` names the registers a host control writes directly (gain, black level, a test pattern), with the range each accepts; the family derives the kernel control table from them (§4). A `sony_imx` part derives its controls from the register vocabulary and the laws.

A `generic` part's stream-enable register is its `standby` control. It states the register's value in standby and its value while streaming:

```yaml
controls:
  standby: {reg: CTRL, min: 0, max: 1, standby: 0, streaming: 1}
```

The `standby` control is the part's stream gate, and it states both values. The unit's program leaves the part in standby, so the register tables stop short of the vendor sequence's stream start, and the behaviour class never writes that register. The flows start and stop the stream with the gate: after the unit's run, around a capture consumer's start, and at `off`. The gate is no live knob, so `caps` does not list it and `set` does not take it. It rides the control table as the `standby` row, with its two values.

### Shipped points: what a mode ships per camera count

A customer runs a mode as a resolution and bit depth with a rate range, and the range is the point the mode ships. The sensor yaml carries the points per mode under `shipped:`, one per camera count on the port (a link alone, or the pair) and CSI lane count. A point is the operating point that many cameras run: the free-run fps range, the line length they run (`hmax`), and for a pair shipped under frame sync the trigger frame (`trigger_vmax`).

```yaml
shipped:
  mode_full:
  - cameras: 1
    csi_lanes: 2
    fps: {floor: 20, ceiling: 74}
    hmax: 364                           # the mode's own line
  - cameras: 2
    csi_lanes: 2
    fps: {floor: 20, ceiling: 74}
    hmax: 728                           # the pair's line: the hub's output drains it
    trigger_vmax: 1240                  # the fast-trigger frame the synced pair runs
```

The camera count decides the line. A pair reads both heads out together and the hub serializes their lines through its line memory, so a pair may need a longer line than the mode's own (the line-rate law), and its point carries that line. The loader holds every point to its mode: a point may name only a mode the personality's program offers, `hmax` is the mode's `timing.hmax` or a longer line, never shorter, and a `trigger_vmax` holds at least the mode's rows. The tool composes the port at its point: one camera on a two-link port runs the one-camera point's line, the pair runs the pair's line, and `set sync fsync` converts the pair at the point's trigger frame; a pair whose point carries no trigger frame is not offered under frame sync. A mode with a point for the port's camera count and lane count is shipped there. A mode with a one-camera point alone is shipped in a pair too, at the pair line the hub's line law leaves it beside the partner (the partner at its declared pair line, or at its own): the tool derives the point, never shorter than the mode's own line, the one-camera range scaled to it, free-running only, and `caps` marks the mode `[pair line … ns]`. A mode without a point is experimental. A point carries nothing about how it was proven: no bench, no tool version and no date, on the unit, in the yaml or in `caps`.

`nxs <port> caps` heads its table with the port's lane count and camera count (`cam0 (2 CSI lanes, 2 cameras)`), then lists the shipped modes, each as its resolution and bit depth with its range (`  1920x1080  RAW10  20–74 fps`), a mode that runs the pair at a derived line marked `[pair line 9414 ns]`, then the sync pairs a two-camera port carries a synced point for, and nothing else; with no shipped mode it prints `no shipped mode for this port: the unit's shipped points carry none (--experimental lists the unshipped ones)`, and under `--experimental` the other modes follow, marked `[experimental: unshipped]` or `[experimental: no unit program]`. `on --mode` refuses an unshipped mode (`link A: 1920x1080 RAW10 is not shipped with 2 cameras on 2 CSI lanes`) and names the shipped modes as the alternatives (`…, shipped`) beside the experimental surface; `status` judges a declared rate by the shipped range (`link A: 100 fps is outside 1920x1080 RAW10's 20–74 fps (shipped range with 2 cameras on 2 CSI lanes)`); `set sync fsync` refuses a pair whose point carries no trigger frame (`1920x1080 RAW10 is not shipped under frame sync with 2 cameras on 2 CSI lanes (no trigger frame in its shipped point)`).

The shipped points of a mode are the pack's word: a point is measured on the bench with `nxs <port> [<link>] capture --frames N` at the floor, the middle and the ceiling of the range, and written into the descriptor's `shipped` block by hand.

The unit's personality carries the points as the SHIPPED record (§4, record type 9), so a host learns them from the unit: a count u8, then per point the mode index u8 (the MODES order), the camera count u8 (1 or 2), `csi_lanes` u8, the fps floor and ceiling as u32 thousandths, `hmax` u16 and `trigger_vmax` u32 (0 for a point that ships free-running only). No strings, no stamps.

A serdes limit a bench measured and the product needs (the serializer's pixel tail, which bounds every shipped range; the deserializer's pulse fraction) ships as a measured fact: `limits_source: {pixel_tail_us: measured}`, with no stamp.

### Experimental overlays: what never ships

A chip descriptor states what the part is — registers, readout modes, timing formulas, each traceable to a datasheet or a vendor driver — and what a bench proved. Operating points a bench found on the way (a clean frame length, a jump threshold, a pacing cap, an exposure and gain that showed a picture, a trigger delay) are experimental material: they live in an experimental directory beside the pack's tree, never in the pack and never in a release:

```
descriptors-experimental/acmerig/
  experimental.yaml                  # {experimental: acmerig, api: 1, overlays: {cam1: acme_cam1}, chips: [cam2]}
  acme_cam1/acme_cam1.yaml    # the overlay: what it overrides, nothing else
```

The overlay names the part it layers onto and carries only what it overrides. Mappings merge key by key, so it sets one limit without restating the section; anything else replaces outright. It is attached only under `nxs --experimental`, from the experimental directory beside the pack's tree or the one `$NXS_CAM_EXPERIMENTAL` names; a personality is compiled from the shipped descriptor alone, and the release export refuses a manifest naming overlays, an experimental limit, or an experimental timing key.

Every limit declares its origin in `limits_source:` as `datasheet`, `driver`, `measured` or `experimental`. The loader refuses a limit with no source, and a source naming no limit. An overlay answers to its own schema, which accepts nothing but `experimental` — a value that is merely transcribed belongs in the part description, a value the product needs is measured and ships as a fact. `nxs --experimental <port> caps` prints the experimental limits and the laws; without the flag neither appears.

## 4. Sensor personalities and law families

The behaviour module is what the unit runs. It is a `CameraSensor` class in the same DSL as a unit driver ([Personality Authoring Reference §4.14](nxs-personality-authoring.md)): `probe()` is the alive or identity check, `configure()` the shared init table followed by one block per value of each enum parameter a `select()` names. It carries writes, bursts, sleeps, polls, and the arithmetic that turns a staged parameter into a register value (`param()`, `write_wide()`, `store_param()`), and nothing else: no law, no host-side hook.

A run is staged by parameter. `mode` and `trigger` are the enums the MODES and TRIGGERS records name. `action` says what the run does: 0 the whole program (the tables, the timing, the start), 1 a standby, 2 a start from standby. `line_time` and `frame_period`, in nanoseconds, are the line and frame periods the timing program turns into the sensor's line length and frame length; the RUN_PARAMS record carries their ranges and defaults (the default mode's datasheet line and its recommended frame). Once the run ends the same parameters read back what it achieved: `line_time` as the line length's period and `frame_length` in lines ([Interface Description §6.11](nxs-host-interface.md)); `nxs <port> <link> status` prints them on the link's NXS line, after the personality, its mode count, its slot and its last run.

```python
from pathlib import Path

import yaml

from nxs import CameraSensor, I2cProfile

FACTS = yaml.safe_load(Path(__file__).with_name("cam1.yaml").read_text())
REG = {name: int(str(spec["addr"]), 0) for name, spec in FACTS["registers"].items()}
MODES = [(name, mode["table"]) for name, mode in FACTS["modes"].items() if "table" in mode]
SETTLE = FACTS["program"]


class Cam1(CameraSensor):
    I2C_ADDRS = [int(str(FACTS["meta"]["i2c_addr"]), 0)]
    I2C_PROFILE = I2cProfile(addr_bytes=2, data_width=2, byte_order="big")

    def probe(self):
        self.poll(REG["ID"], 0xFFFF, 0x1616, timeout_ms=200, poll_ms=10)

    def configure(self, config):
        self.declare_param("mode", values=list(range(len(MODES))), default=0)
        self.declare_param("trigger", values=[0, 1], default=0)
        self.write_table(self.load_table("cam1_init.yaml"))
        self.select("mode", {i: (lambda t=table: self.write_table(self.load_table(t)))
                             for i, (_, table) in enumerate(MODES)})
        self.select("trigger", {0: lambda: None, 1: self._fast})

    def _fast(self):
        self.write(REG["TRIG"], 0x0001)
        self.sleep_ms(int(SETTLE["trigger_switch"]["standby_ms"]))
        self.write(REG["TRIG"], 0x0003)
```

The module reads its own yaml for register addresses, presets, and settles, so a fact lives once. A second device on the pod bus is an I²C companion (`I2C_COMPANIONS`, reached with `dev=` on a write or table, its identity register checked ahead of `probe()`); the pack's serializer is one, and its captured CSI block loads from the serializer descriptor's blobs. A **register table** beside the module is a YAML list of rows: `[reg, value]` (hex or decimal), or `{sleep_ms: N}`, a settle the unit sleeps between the writes around it. `load_table()` reads it at compile time; a run of consecutive registers compiles into one burst. The vendor's init table goes into the shared table, each mode's delta into its own, and the compiler's report prints the bytes each block costs against the 4096-byte program cap.

The **law family** named in `meta.chip` serves everything the host computes for the sensor: the timing law (frame length from the declared rate), exposure and gain conversions, the standby-wrapped timing program the host writes through the link window after the unit's run, the stream start and stop, the alive expectation `on` polls, the runtime knobs `caps` lists, and the control table the device tree carries for the kernel driver. `generic` serves a table-only part from its `controls:` and `registers:`: fixed-rate modes, controls written as they are and clamped to their range, and its stream start and stop from the `standby` control. `sony_imx` serves the parts whose registers follow the FRAMOS vocabulary, parameterized by the descriptor's limits, timing, and presets. A family is public tool code; a sensor's numbers are data.

A table-only part has no timing law, so behind a hub it runs alone, at the one rate its mode's table sets. The bring-up repairs the link as for any head and then starts the part with its stream gate, where a part with a frame law gets its timing program. `caps` lists the mode with a range whose floor is its ceiling (`30–30 fps`). What the family cannot serve is refused in one line. Another rate answers `link A: <mode> runs at <N> fps only` and names `fps <N>`. A second camera on the port answers `<sensor> runs its mode at one rate and takes no frame sync, so it runs alone on the port` and names `link A alone`.

The **compiled image** is an NXS image of kind CAMERA: the probe and configure programs as bytecode, the `mode` and `trigger` parameters with their value sets, the I²C profile (address, widths, byte order), and a descriptor trailer the unit stores without interpreting and serves back page by page ([Interface Description §6.11](nxs-host-interface.md)):

| Record | Carries |
|---|---|
| IDENTITY | address, register and value widths, identity and alive registers, the default mode, name, compatible |
| MODES | the index of the `mode` parameter; per mode its value, geometry, lanes, rate, data type, flags (default, triggerable, serializer CSI by the personality), name, and the timing facts the yaml declares for it (line length, the datasheet frame length, readout kind, the VINT_EN field); an experimental overlay's operating points never ride |
| TRIGGERS | the index of the `trigger` parameter; per conversion its value and name |
| RUN_PARAMS | the run parameters a host stages in physical units: per parameter its index, the range the unit accepts, the default, its name (the quantity) and its unit |
| CONTROLS | per control its register, width, byte order, form, and parameters; a table-only part's stream gate rides as the `standby` row, its parameters the standby value and the streaming value |
| LAWS | the family and its parameters as the yaml's `limits` declare them: clock, integration offset, minimum integration lines, gain law and ceiling, HCG floor, minimum rate, the captured wait registers, the shutter floor's waits, the frame-length deltas by readout kind |
| PROGRAM | the settles and the repairs the family's timing program applies; a repair is carried by register address, width, byte order, and value |
| CAPTURE | the capture facts above, entry by entry, each row's line length and top rate derived from the mode the program runs, and the sensor's clock mode where it gates its clock |
| SHIPPED | the shipped points: per mode, camera count (1 or 2) and CSI lane count the fps range the mode ships, the line length they run, the trigger frame of a pair shipped under frame sync (0 for a point that ships free-running only); nothing about how a point was proven; a tool that predates the record skips it and offers no shipped mode. A mode that ships a one-camera point alone also runs a pair, at the pair line: the line the hub's output leaves it beside the partner at the partner's declared pair line, never shorter than the mode's own, its one-camera range scaled to that line, free-running only. The tool derives it from the hub's line law; the unit carries no such point |
| STATUS | the status probes: per probe its name, register (address, width, byte order), whether the value prints as an integer, whether a non-zero value warns, an optional mask and expected value, and the decode table; a probe's description stays in the pack |

The records carry indices and values, never parameter names: the unit's peek view stages a run's parameters by index, so a host that only has the unit stages a mode by name from the MODES record alone. The tool builds the trailer from the yaml at compile time (`nxs.personality.records`), and a unit's records rebuild the descriptor the laws need on a host that never saw the source.

A shipped image is **sealed**: its bytecode section is AES-128-CTR ciphertext behind a per-image nonce, decrypted by the firmware into the VM's program buffer at load and never served back. The trailer, the parameters, and the profile stay plain. A sealed image compiles, uploads, runs, and reports like a plain one.

**On the unit.** `nxs <port> <link> upload <name>` compiles the pair (or takes the image) and lands it in a store slot, where it persists; the driver personality in slot 0 keeps auto-loading. `on` reads the unit's records, stages the `mode` and `trigger` values by index, issues the run, and waits for its terminal state; the family's alive expectation, read through the link window, proves the bus token came back before the host writes the declared timing, or the stream gate of a table-only part. A link whose unit holds no camera personality, or one for another sensor, is refused before the first bus write, naming the upload command.

**Serdes modules.** A serializer's or deserializer's `<chip>.py` keeps the knob-module contract: `descriptor()` returns the facts, `knob_<name>(value)` returns write steps or raises `InfeasibleConfig` naming the nearest achievable alternative, and hooks discovered by name refine the generic verbs:

| Hook | Called by | Contract |
|---|---|---|
| `derive_status(readings) -> [str]` | `status` | extra report lines computed from raw probe readings |
| `knob_readback(readings) -> {knob: str}` | `get` | derived knob values (e.g. a rate that is arithmetic over two registers) |
| `select_window(link)`, `expect_locked()` | flows | the deserializer's link window and lock proof |

A chip without a hook simply lacks the refinement — no registration, no base class. The CLI never clamps silently — `nxs cam0 A set speed 2000` prints the refusal and the suggestion, exit 2.

## 5. Flows

The pack's flows assemble the serdes chips and the unit runs into streams. `flows:` in `pack.yaml` names a module or a package; a package splits the surface by subject — `laws.py` (the frame arithmetic and the mode and rate resolution), `port.py` (the bring-up, with the order laws in its docstring), `fsync.py` (the hub's generator), `knobs.py` (the runtime knobs, the window and gate steps, training). Steps are plain dicts built with the public helpers — `w` (write), `rd` (read), `expect` (check, polled when `timeout_ms` is set; with `soft=True` a settle poll that proceeds at the deadline with a warning instead of failing — the replacement for a fixed sleep, never a gate), `wait_ms`, `retry` (bounded re-run of a step block) — each targeting a device role (`DES`, `SER`, `SEN`) the executor resolves through the link window. The sensor's own program is never composed on the host: where it runs, the flow places a **unit-program marker**, and the executor hands the bus token to that link's unit there. A sequence added with `best_effort=True` ends in a note instead of an error when its device answers nothing (`<sequence>: the device did not answer; already off`); `build_park` adds its standby sequences that way, since a head the port never brought up is already off.

```python
from nxs.cam.contracts import CsiContract, VcGeometry
from nxs.cam.plan import RawConfig, scan_forbidden

DEFAULT_MODE = "mode_full"

def build_port(pack, topology, links, mode=None, vmax=None, fps=None):
    mode = mode or DEFAULT_MODE
    sen, des = pack.descriptor("cam1"), pack.chip_module("des1")
    cfg = RawConfig("port-" + "".join(l.name for l in links))
    for link in links:
        cfg.add("window", des.select_window(link.name))   # direct traffic at the link
        cfg.add_unit_program(f"sensor-{link.name}", link.name, mode, "freerun")
        cfg.add("alive", pack.family(sen).expect_alive(1500))  # the token came back
    cfg.add("verify", des.expect_locked())                # prove the link, not hope
    scan_forbidden(cfg, {topology.des_addr: pack.descriptor("des1").runtime_forbidden})
    geo = sen.modes[mode]["geometry"]
    return cfg, CsiContract(port=0, virtual_channels=tuple(
        VcGeometry(vc=i, dt=sen.modes[mode]["mipi"]["data_type"],
                   width=geo["width"], height=geo["height"],
                   bit_depth=geo["bit_depth"]) for i, _ in enumerate(links)))
```

The full surface the CLI dispatches to — every function takes the pack handle first:

| Function | Verb(s) |
|---|---|
| `build_port(pack, topology, links, mode, vmax, fps)` | `on` — one link, or the pair |
| `build_solo(pack, topology, link, …)` / `build_dual(pack, topology, …)` | the one-link and both-link wrappers over `build_port` |
| `build_fsync(pack, topology, *, fps, exposure_us, modes, method)` / `build_trigger_off(pack, topology)` | `set sync fsync` / `set sync free_run` |
| `build_knob(pack, topology, knob, value, *, link, readings)` / `knob_names(pack, link=None)` | `set`, `caps` |
| `build_park(pack, topology)` | `off` |
| `window_steps(...)` / `csi_gate_steps(...)` | the stream choreography |
| `open_window` / `close_windows` / `links_reachable` (imperative, take an open bus) | `status`, `get` |
| `train(pack, i2c, topology, *, links, rounds)` / `recover(...)` | `on` (training, and the recovery round it runs itself) |
| `viewer_hints(pack, topology, mode=None, triggered=False)` | `stream` (capture caps) |
| `fps_range(pack, topology, link, mode)` | `on`, `status`, `caps`, `tune` — the rates a mode may run at for the port's camera count and lane count, the shipped point where there is one |
| `fsync_plan(pack, topology, fps, exposure_us=None, modes=None)` (optional) | `set sync fsync --exposure`, `on` under a declared `camera.exposure_us`, `status` — the generator's pulse multiple, the exposure the pulse's low time sets, and the trigger frame each synced link runs; the port record carries them |
| `fsync_fps_ceiling(pack, topology, modes=None)` (optional) | `tune` — the top of the fps menu under fsync: the highest pulse rate the readout law admits for the selected modes (the menu then drops the rates the resonance and lane laws refuse) |

`build_knob` receives `readings` — the sensor's live status register values — so knobs whose arithmetic depends on the running timing (an exposure in line periods of the current frame length) compute against the live configuration, not a mode default. `build_fsync` re-runs each unit with `trigger` set to the synced conversion, then arms the hub's generator.

Composed configs are inert data until executed: `--dry-run` on a mutating verb prints the composed stream instead of writing it, and execution runs under a bus lock with per-step retries.

## 6. Topology

The pack's topology file names the hardware a carrier wires by default — ports (one deserializer on one host bus with its CSI geometry) and links (the chains behind it). It is the manifest-less fallback: on a deployed host the suite manifest's `ports:` section is the address space, commands name the port and the link (`nxs cam0 on`, `nxs cam1 A on`), and `nxs tune --freeze --ports` writes what a bench found into the manifest.

```yaml
ports:
  "0":
    carrier: acmerig/cam0
    i2c_bus: /dev/i2c-1        # prefer a stable udev alias where the platform has one
    des_compatible: acme,des1
    csi_lanes: 4
    links:
      "0": {name: A, compatible: "acme,cam1", ser_compatible: "acme,ser1",
            des_window: 0x11, csi_vc: 1, capture_id: 0,
            nxs_units: [{alias_addr: 0x31}]}
      "1": {name: B, compatible: "acme,cam1", ser_compatible: "acme,ser1",
            des_window: 0x12, csi_vc: 0, capture_id: 1,
            nxs_units: [{alias_addr: 0x32}]}
default_port: 0
```

Compatibles are the socket keys: the pack supplies a descriptor per compatible, so replacing a deserializer is a topology edit plus a pack that covers the new part — the verbs do not change. Every link carries its `nxs_units`: the NXS unit wired between serializer and camera on the same link, presented to the host at a translated I²C address (`alias_addr`) so several units behind one hub answer apart. With more than one unit behind a hub, every `alias_addr` differs from the unit's `target_addr` and from the other units' aliases, or the topology is refused naming the unit. A link declared without a unit cannot bring its sensor up, because the sensor's program runs on the unit.

## 7. Validation

Five gates, in the order they catch:

1. **Strict parsing.** Manifest, topology, and descriptors reject unknown keys and malformed values at load, with the offending file and key named. A table row's register and value are validated against the profile's widths; a wider value is refused rather than masked on the wire.
2. **The compile.** `nxs upload <pair> -o <name>.nxs` compiles the pair with no device and prints the budget: bytes per block, the dispatch overhead per `select()`, the trailer against its cap. The compiler refuses a host-side hook on a camera class, a `mode` value set that does not cover the offered modes in order, and a program over the cap; `nxs personality check <dir>` judges a pair's shape without compiling: the schema, the behaviour module beside the yaml, and a table file for every offered mode.
3. **Laws at compose time.** Every family law fires before a byte is written; `--dry-run` exercises the full composition with no hardware and prints the stream it would write.
4. **The forbidden scan.** Every composed stream is scanned against each chip's `runtime_forbidden` set; a hit refuses it.
5. **Golden sequences.** A pack may pin byte-exact reference streams (`golden:` in the manifest) and test that composition reproduces them: the host's serdes stream write for write, and the compiled image's sensor write stream against the captured full-chain program's sensor writes — the regression anchor for a flow that has been proven on hardware. The pack's own test suite runs the comparison.

Then prove the personality on the bench the way any camera bring-up is proven: `nxs <port> status` → `upload` → `on` → `status` → `capture --frames 60` → `capture --frames 6000`, a sustained capture over minutes, not seconds, under `nxs --experimental` until the captures give the mode its shipped point; lock bits on serializer links routinely read locked while frames are not delivering, so judge streams by delivered frames over time, never by a lock register alone.

## 8. Stability

The pack contract — the `pack.yaml` keys, the chip YAML schema, the table row grammar, the flow and hook signatures above — is a public API from SDK 1.0, versioned by the manifest's `api:` integer. Additions (new optional keys, new hooks, new flow functions with defaults) arrive without an `api` bump and never invalidate an existing pack; a change that would, bumps `api`, and the loader states the version it expects. The behaviour DSL follows the [Personality Authoring Reference §6](nxs-personality-authoring.md): additions never invalidate a source, a compiled image is bound to its image format, and the trailer grows by new record types that an older tool skips.
