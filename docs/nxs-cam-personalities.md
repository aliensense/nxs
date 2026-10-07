# NXS — Cam Personality Reference

Applies to: NXS SDK 1.1.x · `nxs cam` · `hub.yaml` API 1 · NXS image format 2.0

| Document set | |
|---|---|
| [Glossary](nxs-glossary.md) | one word for each thing |
| [Technical Specifications](nxs-specifications.md) | capability summary tables |
| [Device Reference](nxs-device-reference.md) | interfaces, performance and limits, shipped personalities, versioning |
| [Interface Description](nxs-host-interface.md) | transports, register map, commands, procedures |
| [Integration & Operation Manual](nxs-integration-manual.md) | design-in, host setup, workflows |
| [Click Personality Reference](nxs-click-personalities.md) | authoring personalities for unsupported sensors |
| **Cam Personality Reference** (this document) | describing camera chains for `nxs cam` |
| [MCP Tool Reference](nxs-mcp.md) | operating and configuring through an AI agent |

## 1. The authoring model

A camera chain is an image sensor behind a serializer behind a deserializer link, with an NXS unit on the sensor's pod. Three programs share the chain and none of them knows a sensor by name. The kernel drivers know what the device tree tells them: the sensor's address, its register and value widths, the modes, the control table. The unit holds the **cam personality**: the sensor's program and the descriptors it serves back. The unit runs the program on the pod's bus when the host hands it the bus token.

The `nxs` tool holds the compiler, the deserializer and serializer choreography, and the orchestration verbs. It also holds the overlay generator that turns the unit's descriptors into the host's device tree. Every fact about a sensor comes from its personality. Those facts are the I²C address, the register and value widths, the alive check, the modes, the controls, the laws and the capture facts.

Two things are authored against this document. A **hub directory** describes a Hub: its deserializer and serializer chips, the flows that assemble a stream, and its default wiring. A **cam personality** describes an image sensor: `<name>.yaml`, the digitized datasheet, `<name>.py`, the behaviour the unit runs, and its register tables. The same compiler as for a click personality compiles it into an NXS image of kind CAMERA ([Interface Description §10.1](nxs-host-interface.md)).

A cam personality is its own node, and no hub owns one. Every port offers every installed cam personality, behind a hub or on the connector, and the laws refuse a pairing the port cannot run. What ships to a robot is the compiled image, uploaded into the unit's store.

Everything in this document uses an invented hardware family (vendor `acme`). No value in any example is a real device's.

### 1.1 Relation to Linux conventions

A cam personality's facts file and a Linux devicetree binding do the same job in the same schema language. Each states what a node of one `compatible` may carry, and each is json-schema. A binding is in the kernel's YAML subset ([devicetree binding schemas](https://docs.kernel.org/devicetree/bindings/writing-schema.html)), a facts file under `nxs/schemas/cam-personality.schema.json`. Where a key of the facts file states a fact the kernel already names, the two map as follows:

| Facts file | Linux |
|---|---|
| `meta.compatible` | the node's `compatible` |
| a mode's `geometry.lanes`, and `csi_lanes` on the port | the count of an endpoint's `data-lanes` ([video-interfaces.yaml](https://www.kernel.org/doc/Documentation/devicetree/bindings/media/video-interfaces.yaml)) |
| `capture.clock_noncontinuous` | the endpoint's `clock-noncontinuous`, same file |
| the frame law: line length and frame length against the clock | the frame interval from the blanking controls and the pixel rate ([camera sensor drivers](https://docs.kernel.org/userspace-api/media/drivers/camera-sensor.html)) |

A binding constrains one node. The tool also judges feasibility across nodes, such as a pair on one line or the lanes against the hub's output. It answers a refusal with the lawful alternatives. The kernel still gets a standard description: the tool generates the devicetree overlay for the capture stack from the same facts (§3.1).

## 2. Hub and cam personality directories

`nxs cam` keeps two registries, hub directories and cam personalities, since a Hub and a camera head are separate nodes of the rig. Each registry has its own search path. A hub's flows reach the cam personality a link declares through the registry.

### 2.1 A hub directory

`nxs cam` searches for hub directories in order: every path in `$NXS_CAM_HUBS`, the hub inside the wheel, then a source checkout's own `hubs/`. The paths of `$NXS_CAM_HUBS` are colon-separated, read only under `nxs --experimental`, and every one must exist. A path is a hub directory, or a directory of them. The first hub whose `flows_for` list covers the port's deserializer compatible wins. No hub found is an error naming the searched paths:

```
no hub serves 'acme,des1'; searched: <paths>. This nxs ships its hub inside the wheel; a source checkout finds its tree, a bench sets $NXS_CAM_HUBS under --experimental.
```

A port that names no `hub:` is the connector, one camera wired straight to the host. It takes the tool's own flows and reads no hub directory.

```
acmerig/
  hub.yaml             # the hub's index
  topology.yaml        # the default wiring: the carrier's ports and links
  flows.py             # stream assembly (python)
  des1/  ser1/         # deserializer / serializer: <chip>.yaml + <chip>.py (knobs, program)
```

`hub.yaml` declares the contents, and unknown keys are rejected:

```yaml
hub: acmerig
api: 1                        # the hub.yaml API this hub targets
chips: [des1, ser1]           # the hub's own chip directories
flows: flows.py               # a module, or a package directory
flows_for: ["acme,des1"]      # deserializer compatibles these flows drive
topology: topology.yaml       # fallback port set; suite.yaml ports win
# golden:                     # optional byte-exact reference sequences
#   flow: dual                # flow the references prove
#   files: [golden/dual-base.yaml, golden/dual-addon.yaml]
```

Hub Python is loaded in an isolated module namespace with a `HUB` handle injected. Hub modules import **only** the public `nxs.cam` API (absolute imports) and reach their own data and sibling chips through the handle: `HUB.descriptor("des1")`, `HUB.chip_module("ser1")`. The handle answers for every installed cam personality too, by name or by compatible. A hub never imports another hub.

The hub a release ships sits inside the wheel as package data, `nxs/hubs/<hub>/`, comments stripped, under its `LICENSE-hub`. It carries no cam personality and nothing experimental. The release compiles each of its chips that has a program class into a sealed image in the assets. The host runs those images through `libnxs` when it brings a port up.

### 2.2 A cam personality directory

A cam personality is one directory, named for the sensor:

```
cam1/
  cam1.yaml            # the datasheet: registers, modes, laws, capture facts
  cam1.py              # the behaviour the unit runs: probe(), configure()
  cam1_init.yaml       # register tables the behaviour loads
  cam1_mode_full.yaml
  blobs/               # captured programs the modes name, if any
```

`nxs cam` reads cam personalities in order, the most specific first. Under `nxs --experimental` the paths in `$NXS_CAM_PERSONALITIES` come first, colon-separated, and every path must exist. Then come the personality store, `/opt/aliensense/personalities/<name>/`, and a source checkout's own `cam_personalities/`. The checkout's own is read only where `$NXS_CAM_PERSONALITIES` names no path. The sealed images the assets put in the store come last, read from their trailers. A name resolves on the first path that carries it.

`nxs personality install ./cam1` checks the directory and copies it, its tables included, into the store as `/opt/aliensense/personalities/cam1/`. A click personality lands there in the same shape ([Click Personality Reference §5](nxs-click-personalities.md)). Every port offers every installed cam personality, on a hub's links and on the connector. A sensor that nothing installed describes is refused by name:

```
no installed cam personality describes 'acme,cam9' (installed: cam1, cam2; searched: <paths>); install one: nxs personality install <directory>
```

The release compiles every cam personality with a behaviour class into a sealed image in the assets, `/opt/aliensense/personalities/<name>.nxs`, versioned with the wheel. A unit serves its personality's descriptors back. A host that never saw a sensor's source therefore still checks a declaration, lists modes, and generates the device tree for it.

The `nxs-generate-cam-personality` skill shipped with the SDK writes a cam personality from a datasheet and a vendor setting file. An IMX335 cam personality generated that way writes the sensor identically to the shipped one for every action, the settles and the alive timeout apart. The cam personality is authored against this document either way.

## 3. Facts files

A chip's facts file, `<chip>.yaml`, is its digitized datasheet. The core keys are shown. `modes` applies to sensors, `windows` to deserializers. The schema, `nxs/schemas/cam-personality.schema.json`, named by the modeline, also names the sections a hub's own modules consume (sync roles, test patterns, a mode catalog). The loader refuses a key it does not name.

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

`mipi.embedded_lines` and the capture rows' `embedded_lines` count the embedded-data lines the sensor emits with each frame. The overlay declares them on a connector port and declares none behind the Hub, which parks that line on another channel. The capture stack faults every frame that carries a line it did not expect.

A deserializer adds its link-window map, the register values that direct control traffic at one link, both links, or broadcast:

```yaml
windows: {A: 0x11, B: 0x12, broadcast: 0x13}
runtime_forbidden: [0x0FFF]   # e.g. a soft-reset register: bring-up only
```

**Status probes** power three verbs from one declaration. `nxs cam0 status` renders every probe per chip, `nxs cam0 get <name>` serves a single one, and diagnostics walk them across the topology. `expect:` marks a probe pass/fail, `mask:` selects bits, `decode:` maps values to words, `format: int` prints the number. The unit carries them as the STATUS record (§4, record type 10): name, register, format, mask, expected value and decode table. The `desc` text stays in the facts file, so a unit-served probe prints its name alone.

A hub probe named `link_lock_<link>` with a `mask:` gates the walks. `status`, the diagnosis and `generate` read nothing through the window of a link whose probe reads zero. A probe the hub does not answer leaves its link unwalked as `link lock not read`.

**Modes** name the program that selects them. A mode with a `table:` is offered by the personality. Its value in the `mode` parameter is its index among the offered modes, in file order. A mode flagged `host_only: true` is known to the laws (`status`, `tune`) and offered by no program. The tool refuses to bring it up.

**The datasheet frame.** A `sony_imx` mode carries its row of the datasheet's register lists under `sony:`. The row holds the recommended frame length V_TR (`vmax`), at which the datasheet guarantees the imaging characteristics. It also holds the line length (`hmax`), the rate at them and the section that states them. The loader refuses a row without `vmax`, `hmax` or `section`.

The frame laws read V_TR from the row. A mode without one computes it from `timing`: a declared `min_frame_length`, else the rows plus the family's blanking formula. The row stays in the source. The MODES record carries the mode's timing facts, and a host that reads the unit computes V_TR from them.

```yaml
modes:
  mode_roi:
    geometry: {width: 1600, height: 1200, bit_depth: 10, lanes: 4, rate_mbps: 1500}
    timing: {hmax: 400}
    sony: {vmax: 1250, hmax: 400, max_fps: 148.5, section: "7.3.2"}
```

**`runtime_forbidden`** is the safety rail: registers that may appear in bring-up tables but must never be touched by a composed runtime flow. The composer scans every composed stream against it and refuses on a hit.

### 3.1 Cam personalities

A sensor's directory is a **personality source**: the yaml above, the behaviour module (§4), and its register tables. Four sections make a sensor a full citizen of the stack.

**Identity.** A sensor with an identity register declares it. The tool detects the part through the link window, records it, and refuses a declaration the silicon contradicts. A sensor without one is declared only. A deserializer's facts file must declare its identity register, because the identity read is the gate every write stands behind. `on` therefore refuses to program a hub it cannot verify, and `probe` names what answered.

```yaml
meta:
  device_id_reg: 0x0000       # MSB first
  device_id: 0x1616
  device_id_width: 2          # bytes (1 when absent)
```

**The capture table.** The capture host's contract is generated from the cam personalities: one capture-side mode per `capture.table` entry whose mode a pod program runs. The rows of the cam personalities the product ships come first, ordered by their exposure ceilings, the longest last. A cam personality installed into the store, or an experimental head, is an extension, and its rows follow. The sensor states what the capture stack must know about it, and the host adds its own facts (lane count, port, virtual channel, lane polarity). One overlay per port and lane count carries the table on both virtual channels. Entries are transcribed from the vendor's capture-side driver where one exists.

**The vendor's tuning override.** `capture.tuning` names the vendor's ISP override for the sensor, by `override_url` and `sha256`. The host fetches it once and folds it into the capture stack's tuning for the sensor's modes. Only a host that builds that tuning folds it in: JetPack 7.2.1 does, and JetPack 6.2.1 streams on the capture stack's built-in tuning. A cam personality without it leaves those modes on the stack's defaults.

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

A mode's table index, the number a capture session names the mode by, is its row's position in the generated table. Nothing declares it, and the booted tree's index wins at run time.

A port's table carries the rows of one bit depth and one Bayer phase. The capture stack takes every mode of a node in the bit depth and the phase of the node's first mode. The depth is the one the port's declaration runs: each link's declared mode, else the highest mode the port admits. Where two links run two depths, it is the depth of the mode with the longest exposure. An extension's rows set the depth only where the declaration runs no row of a cam personality the product ships. Within that depth the table takes the phase of its row with the longest exposure.

A row of another `bit_depth` or `pixel_phase` takes no index and boots on no node. A declaration of its mode is refused with the depth or the phase named. `on --mode` with such a mode is refused before any write. The refusal names the format the table carries, the declaration that carries a mode of another depth, and the modes the table carries: `cam0/A: 2064x1552 RAW12 boots on no row of this port's table (the table carries one pixel format, RAW10, chosen by the declaration)`. Declaring a mode of another bit depth changes the table, so `nxs switch` installs the port's overlay again and asks for a reboot.

A row's rates are the host's. A row that states no `max_fps` tops out at the mode's own ceiling. That ceiling is the lower of the mode's datasheet frame at its line and one camera's lane law on the port's lanes. The lane law is the rate at which one stream of the mode fills the hub's CSI output, its lines at the deserializer datasheet's line. That line is the pixels over the bandwidth factor, then the minimum blanking.

The row admits every rate the laws give the mode on the port, and no rate past the sensor's own. Its default rate is the rate the port's declaration runs the mode at, 30 fps where it declares none, never above the mode's own ceiling. A change of the declared rate moves the rows, so `nxs switch` installs the port's overlay again and asks for a reboot.

`exposure.max_us` is the sensor's own limit. A booted row's exposure ceiling is the smaller of that limit and the frame at the row's default rate less 32 lines of shutter margin. So no consumer's exposure loop asks for an exposure the frame cannot hold.

`clock_noncontinuous: true` says the sensor gates its clock lane between bursts. The clock is continuous when the key is absent. The key names the same fact as the kernel's `clock-noncontinuous` endpoint property (§1.1).

A mode flagged `serializer_csi` ends its unit program on the serializer. The personality reaches the pod's serializer as an I²C companion and writes, after the mode's tables, the CSI block that carries the mode's contract. That block is PHY standby, lanes, packing and doubling, the data-type filter, and the non-continuous clock. The host composes no serializer CSI block for such a link and keeps the link bring-up, which runs before the unit is reachable. The flag rides the MODES record, so a unit-served descriptor carries it.

**The program settles and the trigger presets.** `program:` carries the waits the family's timing program and the behaviour class use: a standby settle, a release settle, a start settle, per program. `trigger:` carries the register values of each conversion (`freerun`, `fast`), and `sync:` whether the head takes the hub's trigger pulse. The behaviour class reads them, so a settle is changed in one place. `trigger.timing.pulse_width_is_exposure: true` says the head integrates for the trigger pulse's low time under fast trigger and ignores its shutter register. The IDENTITY record carries the fact (§4).

Under frame sync the exposure is the pulse's low time plus the sensor's trigger offset. The tool states it as `the trigger pulse's low time`, without a number, and refuses an exposure asked of a synced link.

```yaml
program:
  timing_start: {standby_ms: 200, release_ms: 50, start_ms: 600}
  trigger_switch: {standby_ms: 200, release_ms: 50, start_ms: 100}
  stop_ms: 300
  start: {release_ms: 20, start_ms: 100}
trigger:
  freerun: {trigmode: 0x00, vint_en: 0x1A}
  fast:    {trigmode: 0x0A, vint_en: 0x18}
  timing:  {pulse_width_is_exposure: true}
sync: {takes_trigger: true, text: "XTRIG on the serializer's MFP pin"}
```

**Controls.** For a `generic` family part, `controls:` names the registers a host control writes directly (gain, black level, a test pattern), with the range each accepts. The family derives the kernel control table from them (§4). A `sony_imx` part derives its controls from the register vocabulary and the laws.

A `generic` part's stream-enable register is its `standby` control. It states the register's value in standby and its value while streaming:

```yaml
controls:
  standby: {reg: CTRL, min: 0, max: 1, standby: 0, streaming: 1}
```

The `standby` control is the part's stream gate, and it states both values. The unit's program leaves the part in standby, so the register tables stop short of the vendor sequence's stream start. The behaviour class never writes that register. The flows start and stop the stream with the gate: after the unit's run, around a capture consumer's start, and at `off`. The gate is no live knob, so `caps` does not list it and `set` does not take it. It rides the control table as the `standby` row, with its two values.

### Rates: the datasheet frame, the lanes and the delivery check

A mode runs at any rate from the kernel driver's minimum to the lower of two ceilings at the line the port runs it at. The kernel driver's minimum is `capture.framerate.min_fps`, or the rate of the longest frame the frame-length register holds where that is higher. The first ceiling is the rate of the mode's recommended frame V_TR. The second is the lane law, the rate at which the port's streams fill its CSI lanes. Under the lane law each line is its pixels at the lane rate over the deserializer datasheet's bandwidth factor, plus the minimum blanking. That is the line a pair runs.

A table-only part runs its mode at the one rate its table sets. One camera runs the mode's own line. The two heads of a pair read out together. The deserializer's line memory passes each pipe's lines to the CSI output one after the other. So a pair runs the deserializer datasheet's pair line: per head its pixels at the lanes' rate over the bandwidth factor, then the minimum blanking. The pair line is never shorter than a head's own line.

Under frame sync each head converts at V_TR. A synced rate is a whole rate of the range whose trigger frame ends inside the pulse period and that the lanes carry. A rate slower than V_TR's runs a longer frame, where the datasheet guarantees no imaging characteristics: the rate runs, and `caps` marks it.

`on`, `set sync` and `set fps` then check the rate on the rig. They count two seconds of frames on every camera link through the capture stack and refuse a rate a link does not deliver within 1 %. The refusal names the rate each link delivered and the highest lawful whole rate at or under the lowest: `cam0: 70 fps asked, 68.2 delivered on A, 68.4 on B`, then `- nxs cam0 set sync fsync --fps 68`. A delivery under the lowest lawful rate names that rate and says so (`under the 2 fps floor`). `set` puts back what ran before, and `status` prints the last check that passed.

On a pair behind a hub, `on` also reads the hub's line-memory overflow probe around the count. An overflow lengthens the pair line to 105 % of it, up to twice the datasheet's, and the port records the line it found. `set`, `caps` and `status` judge the port at it. No cam personality or trailer record carries a proof.

`nxs <port> caps` heads its table with the port's lane count and camera count, then lists every mode as its resolution and bit depth with its range. On a pair it lists the line each mode runs (`[pair line 10774 ns]`, with `, found` where the delivery check found a longer line than the datasheet's). It also lists the rate below which the frame stretches past V_TR, and the sync pairs a two-camera port runs under frame sync.

A mode the unit's program does not carry is marked `[no unit program]`. A mode the pair's program does not carry is marked `[runs alone on the port]` with no range or line. A mode the port's table boots no row for is marked `[no capture row]` with no range. `--json` carries them as `runs_alone` and `capture_row` beside the mode's `sensor_mode` index.

```
cam0 (2 CSI lanes, 2 cameras)
link A  acme,cam2
  1600x1200  RAW10  1.50–74.25 fps  [pair line 10774 ns]
    below 74.25 fps: beyond the datasheet's guaranteed frame (8.1)
sync pairs (fsync, one rate):
  1600x1200 + 1600x1200  2–74 fps
knobs: exposure fps gain sync
trigger modes: fast freerun
```

### Experimental overlays: a bench's own values

A chip's facts file states what the part is: registers, readout modes, timing formulas. Each is traceable to the datasheet or, where the vendor driver is the only public source, to that driver. A development bench may layer its own values over a part for an experiment: a line, a trigger frame, or a starting exposure and gain. They live in the experimental tree, never in a facts file and never in a release:

```
cam_personalities-experimental/
  experimental.yaml           # {api: 1, overlays: {cam1: acme_cam1}}
  acme_cam1/acme_cam1.yaml    # the overlay: what it overrides, nothing else
  cam2/                       # a head the product does not ship: a cam personality directory
```

`experimental.yaml` maps a cam personality to its overlay directory beside the file. The overlay names the part it layers onto and carries only what it overrides. Mappings merge key by key, so it sets one limit without restating the section. Anything else replaces outright. The tree is read only under `nxs --experimental`, from the path in `$NXS_CAM_EXPERIMENTAL`, else beside a source checkout's `cam_personalities/`. Its other directories are cam personalities the product does not ship, read after the directories of §2.2.

A cam personality is compiled from its shipped facts file alone. The hub's export into the wheel refuses an experimental limit, an experimental timing key and any file of the experimental tree. The flag gates the paths in `$NXS_CAM_HUBS` and `$NXS_CAM_PERSONALITIES`, the experimental tree and a development personality store, and nothing else. Every mode a personality's program carries runs without it.

Every limit declares its origin in `limits_source:`: `datasheet`, or `driver` where the vendor driver is the only public source. The loader refuses a limit with no source, and a source naming no limit. An overlay answers to its own schema, whose limits read `experimental` and nothing else: a value a datasheet states belongs in the part description.

## 4. Cam personalities and law families

The behaviour module is what the unit runs. It is a `CamPersonality` class in the same DSL as a click personality ([Click Personality Reference §4.14](nxs-click-personalities.md)). `probe()` is the alive or identity check. `configure()` is the shared init table followed by one block per value of each enum parameter a `select()` names. It carries writes, bursts, sleeps, polls, and the arithmetic that turns a staged parameter into a register value (`param()`, `write_wide()`, `store_param()`). It carries nothing else: no law, no host-side hook.

A run is staged by parameter. `mode` and `trigger` are the enums the MODES and TRIGGERS records name. `action` says what the run does. 0 is the whole program: the tables, the timing, the start. 1 is a standby, and 2 a start from standby. 3 is the timing program alone: it writes the staged line and frame periods under standby, then starts.

`line_time` and `frame_period`, in nanoseconds, are the line and frame periods the timing program turns into the sensor's line length and frame length. The RUN_PARAMS record carries their ranges and defaults, the default mode's datasheet line and its recommended frame. Once the run ends the same parameters read back what it achieved: `line_time` as the line length's period and `frame_length` in lines ([Interface Description §6.11](nxs-host-interface.md)). `nxs <port> <link> status` prints them on the link's NXS line, after the personality, its mode count, its slot and its last run (`never run` before the first).

```python
from pathlib import Path

import yaml

from nxs import CamPersonality, I2cProfile

FACTS = yaml.safe_load(Path(__file__).with_name("cam1.yaml").read_text())
REG = {name: int(str(spec["addr"]), 0) for name, spec in FACTS["registers"].items()}
MODES = [(name, mode["table"]) for name, mode in FACTS["modes"].items() if "table" in mode]
SETTLE = FACTS["program"]


class Cam1(CamPersonality):
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

On the connector itself the enable line releases the sensor's reset before its clock runs. `probe()` therefore begins with the part's software reset, where the part has one, and polls the identity after it. The behaviour module reads its own yaml for register addresses, presets, and settles, so a fact lives once. A second device on the pod bus is an I²C companion, `I2C_COMPANIONS`, reached with `dev=` on a write or table. Its identity register is checked ahead of `probe()`. The hub's serializer is one, and its captured CSI block loads from the blobs the serializer's facts file names.

A **register table** beside the behaviour module is a YAML list of rows. A row is `[reg, value]` (hex or decimal), or `{sleep_ms: N}`, a settle the unit sleeps between the writes around it. `load_table()` reads it at compile time, and a run of consecutive registers compiles into one burst. The vendor's init table goes into the shared table, each mode's delta into its own. The compiler's report prints the bytes each block costs against the 4096-byte program cap.

The **law family** named in `meta.chip` serves everything the host computes for the sensor. That is the timing law (frame length from the declared rate), and the exposure and gain conversions. It is the standby-wrapped timing program the host writes through the link window after the unit's run. It is also the stream start and stop, the alive expectation `on` polls, and the runtime knobs `caps` lists. And it is the control table the device tree carries for the kernel driver.

`generic` serves a table-only part from its `controls:` and `registers:`. It serves fixed-rate modes, controls written as they are and clamped to their range, and the stream start and stop from the `standby` control. `sony_imx` serves the parts whose registers follow the FRAMOS vocabulary, parameterized by the cam personality's limits, timing, and presets. A family is public tool code, and a sensor's numbers are data. A part whose gain law is not linear in dB declares no `gain` control, and the capture stack drives exposure alone.

A table-only part has no timing law, so behind a hub it runs alone, at the one rate its mode's table sets. The bring-up repairs the link as for any head, and then starts the part with its stream gate. A part with a frame law gets its timing program there instead. `caps` lists the mode with a range whose floor is its ceiling (`30–30 fps`).

What the family cannot serve is refused in one line. Another rate answers `link A: <mode> runs at <N> fps only` and names `fps <N>`. A second camera on the port answers `<sensor> runs its mode at one rate and takes no frame sync, so it runs alone on the port` and names `link A alone`.

The **compiled image** is an NXS image of kind CAMERA. It holds the probe and configure programs as bytecode, and the `mode` and `trigger` parameters with their value sets. It holds the I²C profile (address, widths, byte order), and a descriptor trailer the unit stores without interpreting and serves back page by page ([Interface Description §6.11](nxs-host-interface.md)):

| Record | Carries |
|---|---|
| IDENTITY | The address, the register and value widths, the identity and alive registers, the default mode, the name and the compatible. Also whether the head takes the hub's trigger and whether its pulse width is the exposure. |
| MODES | The index of the `mode` parameter. Per mode its value, geometry, lanes, rate, data type, name and flags (default, triggerable, serializer CSI by the personality). Also the timing facts the yaml declares for it: line length, the datasheet frame length, readout kind, the VINT_EN field. An experimental overlay's operating points never ride. |
| TRIGGERS | The index of the `trigger` parameter. Per conversion its value and name. |
| RUN_PARAMS | The run parameters a host stages in physical units. Per parameter its index, the range the unit accepts, the default, its name (the quantity) and its unit. |
| CONTROLS | Per control its register, width, byte order, form, and parameters. A table-only part's stream gate rides as the `standby` row, its parameters the standby value and the streaming value. |
| LAWS | The family and its parameters as the yaml's `limits` declare them. They are the clock, the integration offset, the minimum integration lines, the gain law and ceiling, the HCG floor and the minimum rate. Also the captured wait registers, the shutter floor's waits, and the frame-length deltas by readout kind. |
| PROGRAM | The settles and the repairs the family's timing program applies. A repair is carried by register address, width, byte order, and value. |
| CAPTURE | The capture facts above, entry by entry. Each row's line length and top rate derive from the mode the program runs. The sensor's clock mode rides where it gates its clock. |
| SHIPPED | Retired: the record type (9) stays reserved, no compiler writes it, and every decoder skips it. An image that carries one decodes to the same descriptor. |
| STATUS | The status probes. Per probe its name, its register (address, width, byte order), whether the value prints as an integer, and whether a non-zero value warns. Also an optional mask and expected value, and the decode table. A probe's description stays in the facts file. |

The records carry indices and values, never parameter names. The unit's peek view stages a run's parameters by index. So a host that only has the unit stages a mode by name from the MODES record alone. The tool builds the trailer from the yaml at compile time (`nxs.personality.records`). A unit's records rebuild the descriptor the laws need on a host that never saw the source.

A shipped image is **sealed**: its bytecode section is AES-128-CTR ciphertext behind a per-image nonce. The firmware decrypts it into the VM's program buffer at load and never serves it back. The trailer, the parameters, and the profile stay plain. A sealed image compiles, uploads, runs, and reports like a plain one.

**On the unit.** `nxs <port> <link> upload <name>` compiles the cam personality (or takes the image) and lands it in a store slot, where it persists. The click personality in slot 0 keeps auto-loading. `on` reads the unit's records, stages the `mode` and `trigger` values by index, issues the run, and waits for its terminal state.

The family's alive expectation, read through the link window, proves the bus token came back. Only then does the host write the declared timing, or the stream gate of a table-only part. A link whose unit holds no cam personality, or one for another sensor, is refused before the first bus write, naming the upload command.

**Serdes modules.** A serializer's or deserializer's `<chip>.py` keeps the knob-module contract. `descriptor()` returns the facts, and `knob_<name>(value)` returns write steps or raises `InfeasibleConfig` naming the nearest achievable alternative. Hooks discovered by name refine the generic verbs:

| Hook | Called by | Contract |
|---|---|---|
| `derive_status(readings) -> [str]` | `status` | extra report lines computed from raw probe readings |
| `knob_readback(readings) -> {knob: str}` | `get` | derived knob values, for example a rate that is arithmetic over two registers |
| `select_window(link)`, `expect_locked()` | flows | the deserializer's link window and lock proof |

A chip without a hook lacks the refinement: no registration, no base class. The CLI never clamps silently. `nxs cam0 A set speed 2000` prints the refusal and the suggestion, exit 2.

## 5. Flows

The hub's flows assemble its serdes chips and the unit runs into streams. `flows:` in `hub.yaml` names a module or a package, loaded with the `HUB` handle (§2.1). A package splits the surface by subject:

- `laws.py`: the frame arithmetic and the mode and rate resolution
- `csi.py`: the hub's CSI output, the lane law, the pair line, the transport check
- `port.py`: the bring-up and the pair's retime, with the order laws in its docstring
- `fsync.py`: the hub's generator, and who sets a synced pair's exposure and gain
- `knobs.py`: the runtime knobs, the window and gate steps, the line-memory probe, training

Steps are plain dicts built with the public helpers, each targeting a device role (`DES`, `SER`, `SEN`) the executor resolves through the link window:

- `w` writes and `rd` reads
- `expect` checks, polled when `timeout_ms` is set. With `soft=True` it is a settle poll that proceeds at the deadline with a warning instead of failing. That is the replacement for a fixed sleep, never a gate
- `wait_ms` waits
- `retry` is a bounded re-run of a step block

The sensor's own program is never composed on the host. Where it runs, the flow places a **unit-program marker**, and the executor hands the bus token to that link's unit there. A sequence added with `best_effort=True` ends in a note instead of an error when its device answers nothing (`<sequence>: the device did not answer; already off`). `build_park` adds its standby sequences that way, since a head the port never brought up is already off.

```python
from nxs.cam.contracts import CsiContract, VcGeometry
from nxs.cam.plan import RawConfig, scan_forbidden

DEFAULT_MODE = "mode_full"

def build_port(hub, topology, links, mode=None, vmax=None, fps=None):
    mode = mode or DEFAULT_MODE
    sen = hub.descriptor(links[0].sensor_compatible)    # the cam personality the link declares
    des = hub.chip_module("des1")
    cfg = RawConfig("port-" + "".join(l.name for l in links))
    for link in links:
        cfg.add("window", des.select_window(link.name))   # direct traffic at the link
        cfg.add_unit_program(f"sensor-{link.name}", link.name, mode, "freerun")
        cfg.add("alive", hub.chip_module(link.sensor_compatible).expect_alive(1500))  # the token came back
    cfg.add("verify", des.expect_locked())                # prove the link, not hope
    scan_forbidden(cfg, {topology.des_addr: hub.descriptor("des1").runtime_forbidden})
    geo = sen.modes[mode]["geometry"]
    return cfg, CsiContract(port=0, transport="pixel", virtual_channels=tuple(
        VcGeometry(vc=i, dt=sen.modes[mode]["mipi"]["data_type"],
                   width=geo["width"], height=geo["height"],
                   bit_depth=geo["bit_depth"]) for i, _ in enumerate(links)))
```

The full surface the CLI dispatches to, where every function takes the hub handle first:

| Function | Verb(s) |
|---|---|
| `build_port(hub, topology, links, mode, vmax, fps)` | `on` — one link, or the pair |
| `build_solo(hub, topology, link, …)` / `build_dual(hub, topology, …)` | the one-link and both-link wrappers over `build_port` |
| `build_fsync(hub, topology, fps, *, modes=None, method="manual")` / `build_trigger_off(hub, topology)` | `set sync fsync` / `set sync free_run` |
| `build_knob(hub, topology, knob, value, *, link, readings)` / `knob_names(hub, link=None)` | `set`, `caps` |
| `build_park(hub, topology)` | `off` |
| `window_steps(...)` / `csi_gate_steps(...)` | the stream choreography |
| `open_window` / `close_windows` / `links_reachable` (imperative, take an open bus) | `status`, `get` |
| `train(hub, i2c, topology, *, links, rounds)` / `recover(...)` | `on` (training, and the recovery round it runs itself) |
| `viewer_hints(hub, topology, mode=None, triggered=False, …)` / `viewer_hints_by_link(hub, topology, links=None, modes=None, triggered=False, …)` (the second optional) | `on` and `set`, which record each link's capture caps for `stream`, `capture` and every other consumer: the mode's capture index and rate. Under a trigger the caps crop the filler lines the sensor's data-output delay leaves at the window's end (`crop_bottom`). Where the pulse sets the exposure they pin the capture stack's exposure (`exposure_us`). With `ae_roles` and `gain_db` from `pair_ae` they carry the link's part in the pair's gain (`ae_role`, `ae_peer`, a locked link's `gain_db`) |
| `fps_range(hub, topology, link, mode)` | `on`, `status`, `caps`, `tune`: the rates a mode may run at on the port, `RateRange(floor, ceiling, binds)`. The range runs from the kernel driver's minimum to the lower of the datasheet frame's and the lane law's ceilings at the port's line. `binds` names the law that sets the ceiling |
| `sensor_descriptor(hub, link, topology)` (optional) | every verb that judges a link: the facts of the link's cam personality as the port runs it. On a pair that is each program mode at the pair line, or at the line the delivery check found (`topology.lines`). The pair line is `csi.pair_lines`: the deserializer datasheet's bandwidth formula and minimum blanking |
| `frame_guarantee(hub, topology, link, mode)` (optional) | `caps`: the rate at which the mode runs its recommended frame V_TR at the port's line. `caps` marks the rates below it, citing the flows' `GUARANTEED_FRAME_SECTION` |
| `lane_ceiling(lanes, mipi)` (optional) | the capture table — one camera's lane law on `lanes`, the top rate of the mode's row where it is below the mode's datasheet ceiling |
| `line_overflow(hub, i2c, topology)`, `build_timing(hub, topology, links, modes, rates)` (optional, together) | `on` on a pair behind the hub: the hub's line-memory overflow probe, which the read clears, read around the delivery count. And the running pair retimed at the line `topology.lines` names, each head at the frame its rate takes at that line |
| `fsync_plan(hub, topology, fps, modes=None)` (optional) | `set sync fsync`, `on` under a declared frame sync, `status`, and the refusal of an exposure asked under sync. The plan is one generator pulse per frame, and whether the pulse sets the exposure (the synced sensor's `pulse_width_is_exposure`). It is also the trigger frame each synced link runs, the mode's V_TR, and the port record carries them. A rate a law refuses names the whole rates nearest below and above it that `fsync_rates` lists |
| `fsync_rates(hub, topology, modes=None)` (optional) | `caps` for the sync pairs, `tune` for the fps menu under frame sync, `set`, and the alternatives of a refused synced rate. The rates are the whole rates of the synced links' range whose trigger frame ends inside the pulse period and that the lanes carry. A fast trigger refuses the next pulse for the frame's lines |
| `pair_ae(hub, topology, source)` (optional) | `on` and `set sync`, which record it with the sync (`sync.ae`) for `status`, `nxsd` and every consumer of a link's capture caps. It says who sets the exposure and the gain of the port's camera links under `source`. Under frame sync a pair of one sensor kind, each head at its own host address, follows its first synced link (`{mode: follow, leader, follower}`). A declared `camera.gain_db` locks both links at it instead. Under free run a declared `camera.exposure_us` and `camera.gain_db` lock every camera link (`{mode: locked, gain_db, links}`). Any other port runs each capture session's own loop and names why (`{mode: per_link, reason}`) |
| `build_gain_lock(hub, topology, gain_db, synced)` (optional) | `on` under a declared `camera.gain_db`, and `status`, which judges it. The gain is written once to the head of every camera link through the broadcast window, each head at its own host address. The write runs under the sensor's register hold. A gain past a sensor's ceiling or between two of its steps (the `gain_reg_per_db` law up to `gain_max`) is refused naming the gains that fit. So is a port the lock does not fit. Under frame sync (`synced`) the lock fits a pair of one sensor kind that takes the pulse. In free run it fits the camera links with a declared `camera.exposure_us` |
| `build_follow(hub, topology, leader, follower)` (optional) | `nxsd` on a synced pair whose leader's capture session decides the gain: the `FollowPlan` it copies each frame. The plan names the leader head's register by the kernel control table's `gain` row. The register is read at the leader's host address and written to the follower's under the `group-hold` row's hold. The plan carries the dB one register step is. It is refused for heads at one host address, two sensor kinds, or a sensor without either row |

`build_knob` receives `readings`, the sensor's live status register values. So knobs whose arithmetic depends on the running timing compute against the live configuration, not a mode default. An exposure in line periods of the current frame length is one such knob.

`build_fsync` arms the hub's generator over the free-running port at one pulse per frame, then converts the synced sensors. A pair of one sensor kind in one mode converts through the hub's broadcast window, each head written at its own host address. A mixed hub re-runs the synced link's unit with `trigger` set to the synced conversion. `build_trigger_off` turns the generator off and writes the free-run preset to the synced heads through the broadcast window, each at its own host address. So neither link loses its video.

Composed configs are inert data until executed. `--dry-run` on a mutating verb prints the composed stream instead of writing it, and execution runs under a bus lock with per-step retries.

## 6. Topology

The hub's topology file names the hardware its carrier wires by default. The ports are one deserializer on one host bus with its CSI geometry, and the links are the chains behind it. It is the fallback where no declaration exists. On a deployed host the declaration's `ports:` section is the address space, and commands name the port and the link (`nxs cam0 on`, `nxs cam1 A on`). `nxs tune --freeze --ports` writes what a bench found into the declaration.

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

Compatibles are the socket keys: a hub directory supplies its chips' facts files, and the installed cam personalities supply the sensors'. Replacing a deserializer is therefore a topology edit plus a hub directory that covers that part, and the verbs do not change. Every link carries its `nxs_units`: the NXS unit wired between serializer and camera on the same link. The unit is presented to the host at a translated I²C address (`alias_addr`), so several units behind one hub answer apart.

With more than one unit behind a hub, every `alias_addr` differs from the unit's `target_addr` and from the other units' aliases. Otherwise the topology is refused naming the unit. A link declared without a unit cannot bring its sensor up, because the sensor's program runs on the unit.

## 7. Validation

Five gates, in the order they catch:

1. **Strict parsing.** `hub.yaml`, the topology and the facts files reject unknown keys and malformed values at load. The refusal names the offending file and key. A limit names its source, `datasheet` or `driver`, and a mode's `sony:` row names its section. The loader validates a table row's register and value against the profile's widths. It refuses a wider value rather than masking it on the wire.
2. **The compile.** `nxs upload <personality> -o <name>.nxs` compiles the cam personality with no device and prints the budget. The budget is bytes per block, the dispatch overhead per `select()`, the trailer against its cap. The compiler refuses a host-side hook on a camera class and a program over the cap. It also refuses a `mode` value set that does not cover the offered modes in order.

   `nxs personality check <dir>` judges a cam personality's shape without compiling. It checks the schema, the behaviour module beside the yaml, and a table file for every offered mode.
3. **Laws at compose time.** Every family law fires before the first byte goes to the wire. `--dry-run` exercises the full composition with no hardware and prints the stream it would write.
4. **The forbidden scan.** The composer scans every composed stream against each chip's `runtime_forbidden` set, and a hit refuses it.
5. **Golden sequences.** A hub may pin byte-exact reference streams (`golden:` in `hub.yaml`) and test that composition reproduces them. The test compares the host's serdes stream write for write. It also compares the compiled image's sensor write stream against the captured full-chain program's sensor writes. That is the regression anchor for a flow proven on hardware. The hub's own test suite runs the comparison.

Then prove the personality on the bench the way any camera bring-up is proven. Run `nxs <port> status`, then `upload`, `on`, `status`, `capture --frames 60` and `capture --frames 6000`, a sustained capture over minutes, not seconds. `on` counts two seconds of frames on every camera link and refuses a rate the rig does not deliver. So a mode that comes up delivers its rate on this rig. Lock bits on serializer links routinely read locked while frames are not delivering. So judge streams by delivered frames over time, never by a lock register alone.

## 8. Stability

The hub contract is a public API from SDK 1.1, versioned by the `api:` integer of `hub.yaml`. The contract is the `hub.yaml` keys, the chip YAML schema, the table row grammar, and the flow and hook signatures above. Additions (optional keys, hooks, flow functions with defaults) arrive without an `api` bump and never invalidate an existing hub directory. A change that would, bumps `api`, and the loader refuses a `hub.yaml` of another API, naming the one it speaks.

The behaviour DSL follows the [Click Personality Reference §6](nxs-click-personalities.md). Additions never invalidate a source, and a compiled image is bound to its image format. The trailer grows by new record types that an older tool skips.
