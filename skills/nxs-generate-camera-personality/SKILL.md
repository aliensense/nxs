---
name: nxs-generate-camera-personality
description: >
  Generate an NXS camera personality for a CSI-2 image sensor from its
  datasheet and a vendor setting file: the descriptor YAML, the behaviour
  class the unit runs, and the register tables. Use when a user names an
  image sensor or a camera module for a camera pod.
allowed-tools: Read WebFetch WebSearch Write Bash Glob Grep
argument-hint: [sensor-name-or-datasheet-url] [setting-file]
---

# Generate an NXS camera personality

A camera personality is a sensor pair: `<name>.yaml`, the digitized datasheet, and `<name>.py`, the behaviour the NXS unit on the sensor's pod runs once, under the host's bus token, to program the sensor; the register tables the behaviour loads sit beside them. The tool compiles the pair into an image of kind CAMERA. The laws (frame rate, exposure, gain, the control table of the host's kernel driver) are the tool's law families, parameterized by the YAML. The class carries writes, sleeps and polls only.

A mikroBUS part is a unit personality: run `/nxs-generate-sensor-personality` instead.

**Resolve, don't ask.** Every choice is a datasheet fact or a rule below. Ask nothing; state a judgement call in a YAML comment.

**Never invent a register value.** Every row of every table comes from the vendor's setting file, in its order, sleeps included. Without one, stop and print the card with `tables    MISSING — provide the vendor setting file`.

**Stop on a gate the compiler cannot express.** A `CompileError` you cannot fix within the documented surface ends the run with the NOT EXPRESSIBLE card.

## Step 1: Read the references

Whichever exists in the checkout:

```
find docs -name nxs-camera-personalities.md    # prints the file to read, wherever the docs tree keeps it
```

§3 is the descriptor, §3.1 what makes a sensor a full citizen, §4 the behaviour class and the law families, §7 the validation gates. The DSL verbs a camera class uses are §4.14 of `nxs-personality-authoring.md` beside it. The references win over this skill where they disagree. The compiler is a black box: `nxs upload … -o` is the feedback loop.

## Step 2: The part, the inputs, the family

From `$ARGUMENTS`: the part number and the module it sits on. Two inputs:

1. **The datasheet.** The I²C address; the register address and value widths; the identity register and its value, or without one a register that answers as soon as the core is out of reset; the stream gate; the readout modes with geometry, bit depth, lane count, link rate and top frame rate; the Bayer origin; the input clock; the timing, shutter and gain registers with their units and ranges; whether the clock lane is gated between bursts; whether the part takes an external trigger.
2. **The vendor setting file.** The register sequence that programs each mode: the vendor driver's mode tables, an application note's setting list, or a capture of a working chain.

A published kernel driver stands in for both inputs when the part has no datasheet in hand: its register defines and their comments carry the addresses, widths, units and ranges, its mode tables are the setting file, and the host's devicetree binding for the module carries the lane count, link rate, pixel clock, line length and frame length per mode. Take the facts only from what the source states; a value the source leaves out is `limits_source: driver` with the judgement call in a YAML comment, and a register the source never names does not exist. `meta.provenance` names each file with its repository, tag and licence.

`meta.chip` names the law family:

| Family | When | The family serves |
|---|---|---|
| `generic` | any part whose runtime controls are direct register writes | the controls named in `controls:` clamped to their ranges, the stream gate from the `standby` control, the capture facts; no timing law, so a mode runs at the one rate its table sets |
| `sony_imx` | the registers carry the `sony_imx` vocabulary: `STANDBY`, `XMSTA`, `HMAX`, `VMAX`, a shutter counted from the frame end (`SHS` or `SHR0`), `GAIN`, `REGHOLD`, and for a trigger `TRIGMODE` with `VINT_EN` | the timing law `INCK / (HMAX × VMAX)`, exposure and gain conversions, the standby-wrapped timing program, the trigger conversions, live rate, exposure and gain knobs |

Pick `sony_imx` only when every register of the vocabulary exists on the part under that meaning. A near miss is `generic`.

## Step 3: Write the files

```
./<name>/<name>.yaml           the descriptor
./<name>/<name>.py             the behaviour
./<name>/<name>_init.yaml      the table every mode shares
./<name>/<name>_<mode>.yaml    one table per offered mode
./<name>/<name>_final.yaml     the rows every mode ends on, when the setting file has them
```

`<name>` is lower-case letters, digits and underscores: the part number without its vendor prefix. Every fence below is a fictional part; no value in them is a real device's.

### The descriptor, table-only family

```yaml
# cam1.yaml
default_mode: mode_full

meta:
  compatible: acme,cam1
  role: SEN
  chip: generic
  i2c_addr: 0x36
  reg_bits: 16
  val_bits: 8
  device_id_reg: 0x0000
  device_id: 0xC1
  device_id_width: 1
  provenance: ACME CAM1 datasheet r1.2; setting file cam1_settings_v3.txt

registers:
  ID:       {addr: 0x0000, width: 1}
  RUN:      {addr: 0x0100, width: 1}
  GAIN:     {addr: 0x0204, width: 1}
  BLKLEVEL: {addr: 0x0208, width: 2, order: be}

modes:
  mode_full:
    geometry: {width: 1920, height: 1080, bit_depth: 10, lanes: 2, rate_mbps: 800}
    timing: {fps: 30}
    mipi: {data_type: RAW10, embedded_lines: 0}
    table: cam1_mode_full.yaml
  mode_bin:
    geometry: {width: 960, height: 540, bit_depth: 10, lanes: 2, rate_mbps: 800}
    timing: {fps: 60}
    mipi: {data_type: RAW10, embedded_lines: 0}
    table: cam1_mode_bin.yaml

controls:
  standby: {reg: RUN, min: 0, max: 1, standby: 0, streaming: 1}
  gain: {reg: GAIN, min: 0, max: 240, num: 10, den: 3}
  black_level: {reg: BLKLEVEL, min: 0, max: 1023}

sync: {takes_trigger: false, text: "no external trigger input"}

capture:
  mclk_khz: 24000
  pix_clk_hz: 160000000
  clock_noncontinuous: true
  gain: {factor: 10, min: 0, max: 720, step: 1, default: 0}
  hdr_ratio: {min: 1, max: 1}
  exposure: {factor: 1000000, max_us: 33000, step: 1, default_us: 10000}
  framerate: {factor: 1000000, min_fps: 30, step: 1}
  table:
    - {width: 1920, height: 1080, bit_depth: 10, pixel_phase: rggb, line_length: 2400, max_fps: 30, min_exp_us: 15}
    - {width: 960, height: 540, bit_depth: 10, pixel_phase: rggb, line_length: 1200, max_fps: 60, min_exp_us: 8}

status:
  - {name: streaming, reg: RUN, decode: {0x00: standby, 0x01: streaming}, desc: "RUN"}

runtime_forbidden: []
```

- `meta.compatible` is `vendor,part` as a kernel binding would name it, and the socket key a declaration names.
- `modes` lists the modes the setting file programs, in the order the `mode` parameter counts them; `default_mode` is one of them.
- `mipi.embedded_lines` counts the embedded-data lines the capture stack receives. Behind the Hub it is 0: the hub parks the sensor's embedded-data line on another channel.
- `hdr_ratio` is `{min: 1, max: 1}` for a part without an HDR mode.
- `timing.fps` is the rate the mode's table sets. The capture row of the same geometry states `line_length` in clocks of `pix_clk_hz` and `max_fps` equal to it, so `pix_clk_hz / (line_length × max_fps)` is the frame in lines, at or above the height.
- `controls.standby` is the stream gate with its two values. The tables stop short of the setting file's stream start and the class never writes the gate: the host starts and stops the stream with it.
- Every other control is a register the host writes as it is, clamped to `min..max`; `gain` also states its register steps per dB as `num/den`.
- `capture.gain`, `exposure` and `framerate` are the kernel's ranges: `factor` is the unit denominator (gain in `1/factor` dB, exposure and rate in `1/factor` of their unit), `default` the value a capture opens with.
- `clock_noncontinuous: true` when the datasheet says the clock lane is gated between bursts.
- `provenance` names the documents. `status` probes are what `nxs <port> status` prints for the chip.

### The descriptor, `sony_imx` family

The same file with the family's facts in place of `controls:`:

```yaml
# cam2.yaml
default_mode: mode_full

meta:
  compatible: acme,cam2
  role: SEN
  chip: sony_imx
  i2c_addr: 0x34
  reg_bits: 16
  val_bits: 8
  provenance: ACME CAM2 datasheet r2.0; setting file cam2_settings_v1.txt

registers:
  STANDBY:  {addr: 0x3000, width: 1}
  REGHOLD:  {addr: 0x3001, width: 1}
  XMSTA:    {addr: 0x3002, width: 1}
  HMAX:     {addr: 0x3010, width: 2, order: le}
  VMAX:     {addr: 0x3014, width: 3, order: le}
  SHS:      {addr: 0x3020, width: 3, order: le}
  GAIN:     {addr: 0x3030, width: 2, order: le}
  TRIGMODE: {addr: 0x3040, width: 1}
  VINT_EN:  {addr: 0x3041, width: 1}
  BLKLEVEL: {addr: 0x3050, width: 2, order: le}

modes:
  mode_full:
    geometry: {width: 1920, height: 1080, bit_depth: 10, lanes: 4, rate_mbps: 1188}
    timing: {hmax: 600, min_frame_length: 1250}
    mipi: {data_type: RAW10, trigger_input: XTRIG, embedded_lines: 0}
    table: cam2_mode_full.yaml
  mode_roi:
    geometry: {width: 1280, height: 720, bit_depth: 10, lanes: 4, rate_mbps: 1188}
    timing: {hmax: 600, min_frame_length: 850}
    mipi: {data_type: RAW10, trigger_input: XTRIG, embedded_lines: 0}
    table: cam2_mode_roi.yaml

limits_source:
  inck_hz: datasheet
  integration_offset_us: datasheet
  min_integration_lines: datasheet
  shs_floor: datasheet
  gain_max: datasheet
  gain_reg_per_db: datasheet

limits:
  inck_hz: 72000000
  integration_offset_us: 1.2
  min_integration_lines: 1
  shs_floor: 8
  gain_max: 300
  gain_reg_per_db: 10

trigger:
  freerun: {trigmode: 0x00, vint_en: 0x01}
  fast:    {trigmode: 0x03, vint_en: 0x00}

sync: {takes_trigger: true, text: "fast trigger on XTRIG"}

program:
  timing_start: {standby_ms: 100, release_ms: 20, start_ms: 200}
  trigger_switch: {standby_ms: 100, release_ms: 20, start_ms: 50}
  fast_trigger: {standby_ms: 100, release_ms: 20, start_ms: 200}
  stop_ms: 100
  start: {release_ms: 20, start_ms: 50}

capture:
  mclk_khz: 24000
  pix_clk_hz: 144000000
  gain: {factor: 10, min: 0, max: 300, step: 1, default: 0}
  hdr_ratio: {min: 1, max: 1}
  exposure: {factor: 1000000, max_us: 10000, step: 1, default_us: 2000}
  framerate: {factor: 1000000, min_fps: 5, step: 1}
  table:
    - {width: 1920, height: 1080, bit_depth: 10, pixel_phase: rggb, min_exp_us: 10}
    - {width: 1280, height: 720, bit_depth: 10, pixel_phase: rggb, min_exp_us: 10}

status:
  - {name: streaming, reg: STANDBY, decode: {0x00: streaming, 0x01: standby}, desc: "STANDBY"}
  - {name: vmax, reg: VMAX, format: int, desc: "frame length (lines)"}
  - {name: hmax, reg: HMAX, format: int, desc: "line length (clocks)"}

runtime_forbidden: []
```

- Every `limits` entry has a `limits_source` (`datasheet`, `driver` or `measured`).
- `inck_hz` is the clock `HMAX` counts, `1H = HMAX / inck_hz`, which is not always the INCK pin's frequency: the datasheet's line-period formula names it.
- A mode's `timing.hmax` is its line in that clock's counts and `timing.min_frame_length` the frame the datasheet recommends for it; the ceiling is `inck_hz / (hmax × min_frame_length)`. The capture row states neither `line_length` nor `max_fps`: the family derives them, `line_length = hmax × pix_clk_hz / inck_hz`, so `pix_clk_hz` is a whole multiple of `inck_hz`.
- `trigger` presets carry the full register values of each conversion (`freerun`, `fast`), `mipi.trigger_input` names the pin the pulse arrives on, and `sync.takes_trigger` says the head takes the hub's pulse. A part without a trigger input has no `trigger:` block, no `trigger_input`, and `takes_trigger: false`.
- `program` carries the settles, in ms after the write, of each standby-wrapped program; `stop_ms` is the park settle. The values come from the vendor's launch script or the datasheet's settling figures; where neither states one, use 200 after standby, 50 after release, 600 after the start.
- `shs_floor` is the shutter's minimum over the offered modes, `integration_offset_us` the exposure offset, `gain_max` the register ceiling, `gain_reg_per_db` register steps per dB, as a number or as `[numerator, denominator]`.
- `serializer_csi` stays out: the host composes the pod's serializer from the mode's contract.

### The tables

A table is a YAML list: `[reg, value]` (hex or decimal) or `{sleep_ms: N}`. Split the setting file: the prefix every mode shares becomes `<name>_init.yaml`, each mode's remainder its own table, and a suffix every mode ends on (the timing, gain and shutter defaults a vendor writes after each mode) becomes `<name>_final.yaml`, which the class writes after each mode's table. A value the setting file leaves to the mode but the datasheet ties to it (an A/D depth, a black level per bit depth) goes into that mode's table. Keep the vendor's order and its sleeps; consecutive registers compile into one burst on their own. Comment each table with the setting file's own section name. A value wider than `val_bits` is refused at compile: split it into bytes in the register's byte order.

```yaml
# cam1_init.yaml
# cam1_settings_v3.txt, "common init"
- [0x0110, 0x02]
- [0x0111, 0x18]
- [0x0112, 0x00]
- {sleep_ms: 2}
- [0x0120, 0x01]
```

```yaml
# cam1_mode_full.yaml
# cam1_settings_v3.txt, "1920x1080 30 fps"
- [0x0300, 0x07]
- [0x0301, 0x80]
- [0x0302, 0x04]
- [0x0303, 0x38]
- [0x0310, 0x01]
```

```yaml
# cam1_mode_bin.yaml
# cam1_settings_v3.txt, "960x540 60 fps, 2x2 binning"
- [0x0300, 0x03]
- [0x0301, 0xC0]
- [0x0302, 0x02]
- [0x0303, 0x1C]
- [0x0310, 0x03]
```

A `sony_imx` part's tables hold the part in standby (the first row) and end on the mode's line and frame length, which the timing program rewrites from the staged periods:

```yaml
# cam2_init.yaml
# cam2_settings_v1.txt, "common init"
- [0x3000, 0x01]
- [0x3100, 0x02]
- [0x3101, 0x00]
- {sleep_ms: 1}
- [0x3120, 0x0A]
```

```yaml
# cam2_mode_full.yaml
# cam2_settings_v1.txt, "1920x1080 all-pixel"
- [0x3200, 0x00]
- [0x3201, 0x00]
- [0x3010, 0x58]
- [0x3011, 0x02]
- [0x3014, 0xE2]
- [0x3015, 0x04]
- [0x3016, 0x00]
```

```yaml
# cam2_mode_roi.yaml
# cam2_settings_v1.txt, "1280x720 window"
- [0x3200, 0x01]
- [0x3201, 0x00]
- [0x3010, 0x58]
- [0x3011, 0x02]
- [0x3014, 0x52]
- [0x3015, 0x03]
- [0x3016, 0x00]
```

### The class, table-only family

```python
# cam1.py
"""ACME CAM1 camera personality: what the unit runs under the bus token."""

from pathlib import Path

import yaml

from nxs import CameraSensor, I2cProfile

_FACTS = yaml.safe_load(Path(__file__).with_name("cam1.yaml").read_text())
_REG = {name: int(str(spec["addr"]), 0) for name, spec in _FACTS["registers"].items()}
_MODES = [(name, str(mode["table"])) for name, mode in _FACTS["modes"].items()
          if "table" in mode]
#: How long the head gets to answer after its reset line is released.
ALIVE_TIMEOUT_MS = 1500


class Cam1(CameraSensor):
    I2C_ADDRS = [int(str(_FACTS["meta"]["i2c_addr"]), 0)]
    I2C_PROFILE = I2cProfile(addr_bytes=2)

    def probe(self):
        self.poll(_REG["ID"], 0xFF, int(str(_FACTS["meta"]["device_id"]), 0),
                  timeout_ms=ALIVE_TIMEOUT_MS, poll_ms=20)

    def configure(self, config):
        self.declare_param("mode", values=list(range(len(_MODES))), default=0,
                           kind="reload")
        self.write_table(self.load_table("cam1_init.yaml"))
        self.select("mode", {i: (lambda t=table: self.write_table(self.load_table(t)))
                             for i, (_, table) in enumerate(_MODES)})
```

`probe()` polls the identity register with the mask and value the datasheet gives; a part without one polls a register that answers as soon as the core is out of reset, with mask 0 (any ACK). `configure()` declares `mode` over the offered modes in YAML order, writes the shared table, then one block per mode. Nothing else: the family starts the stream. `I2cProfile` frames every value the program moves: `addr_bytes` from `reg_bits`, `data_width` and `byte_order` when a value is wider than a byte.

### The class, `sony_imx` family

```python
# cam2.py
"""ACME CAM2 camera personality: what the unit runs under the bus token."""

import math
from pathlib import Path

import yaml

from nxs import CameraSensor, I2cProfile
from nxs.personality.records import (ACTION_PARAM, ACTIONS, FRAME_LENGTH_PARAM,
                                     FRAME_PERIOD_PARAM, LINE_TIME_PARAM)

_FACTS = yaml.safe_load(Path(__file__).with_name("cam2.yaml").read_text())
_REG = {name: int(str(spec["addr"]), 0) for name, spec in _FACTS["registers"].items()}
_MODES = [(name, str(mode["table"]), int(mode["timing"]["min_frame_length"]))
          for name, mode in _FACTS["modes"].items() if "table" in mode]
_TRIGGER = _FACTS["trigger"]
_SETTLE = _FACTS["program"]
_INCK_HZ = int(_FACTS["limits"]["inck_hz"])
_G = math.gcd(_INCK_HZ, 1_000_000_000)
#: INCK over one gigahertz, reduced: HMAX = line_ns * _NUM / _DEN.
_NUM, _DEN = _INCK_HZ // _G, 1_000_000_000 // _G
_HMAX_MAX = (1 << (8 * int(_FACTS["registers"]["HMAX"]["width"]))) - 1
_VMAX_MAX = (1 << (8 * int(_FACTS["registers"]["VMAX"]["width"]))) - 1
_DEFAULT = _FACTS["modes"][_FACTS["default_mode"]]["timing"]
_LINE_MIN = min(int(_FACTS["modes"][n]["timing"]["hmax"]) for n, *_ in _MODES) * _DEN // _NUM
_LINE_MAX = _HMAX_MAX * _DEN // _NUM
_LINE_DEFAULT = round(int(_DEFAULT["hmax"]) * _DEN / _NUM)
_PERIOD_DEFAULT = int(_DEFAULT["min_frame_length"]) * _LINE_DEFAULT
_PERIOD_MAX = 4_000_000_000
#: How long the head gets to answer after its reset line is released.
ALIVE_TIMEOUT_MS = 1500


class Cam2(CameraSensor):
    I2C_ADDRS = [int(str(_FACTS["meta"]["i2c_addr"]), 0)]
    I2C_PROFILE = I2cProfile(addr_bytes=2)

    def probe(self):
        # Any ACK on STANDBY: the head answers.
        self.poll(_REG["STANDBY"], 0x00, 0x00, timeout_ms=ALIVE_TIMEOUT_MS, poll_ms=20)

    def configure(self, config):
        self.declare_param("mode", values=list(range(len(_MODES))), default=0,
                           kind="reload")
        self.declare_param("trigger", values=[0, 1], default=0, kind="reload")
        self.declare_param(ACTION_PARAM, values=sorted(ACTIONS.values()),
                           default=ACTIONS["configure"], kind="reload")
        self.declare_param(LINE_TIME_PARAM, values=[_LINE_MIN, _LINE_MAX],
                           default=_LINE_DEFAULT, param_type="range", unit="ns",
                           kind="live")
        self.declare_param(FRAME_PERIOD_PARAM, values=[_LINE_MIN, _PERIOD_MAX],
                           default=_PERIOD_DEFAULT, param_type="range", unit="ns",
                           kind="live")
        self.declare_param(FRAME_LENGTH_PARAM, values=[0, _VMAX_MAX], default=0,
                           param_type="range", unit="lines", kind="live")
        configure, park, start, timing = (ACTIONS[a] for a in ("configure", "park",
                                                               "start", "timing"))
        self.select_grouped(ACTION_PARAM, {frozenset({configure}): self._configure,
                                           frozenset({park}): self._park,
                                           frozenset({start}): self._start,
                                           frozenset({timing}): lambda: None})
        self.select_grouped(ACTION_PARAM, {frozenset({configure, timing}): self._timing_start,
                                           frozenset({park, start}): lambda: None})
        self.select_grouped(ACTION_PARAM, {
            frozenset({configure}): lambda: self.select("trigger", {0: lambda: None,
                                                                    1: self._fast_for_mode}),
            frozenset({park, start, timing}): lambda: None})

    def _configure(self):
        self.write_table(self.load_table("cam2_init.yaml"))
        self.select("mode", {i: (lambda t=table: self.write_table(self.load_table(t)))
                             for i, (_, table, _) in enumerate(_MODES)})

    def _timing_start(self):
        settle = _SETTLE["timing_start"]
        line = self.param(LINE_TIME_PARAM)
        hmax = (line * _NUM + _DEN // 2) // _DEN
        # HMAX is a whole clock count: the frame follows the quantized line.
        achieved = hmax * _DEN // _NUM
        vmax = (self.param(FRAME_PERIOD_PARAM) + achieved // 2) // achieved
        self.write(_REG["STANDBY"], 0x01)
        self.sleep_ms(int(settle["standby_ms"]))
        self.write(_REG["TRIGMODE"], int(str(_TRIGGER["freerun"]["trigmode"]), 0))
        self.write(_REG["VINT_EN"], int(str(_TRIGGER["freerun"]["vint_en"]), 0))
        self.write_wide(_REG["HMAX"], hmax, 2, byte_order="little")
        self.write_wide(_REG["VMAX"], vmax, 3, byte_order="little")
        self.write(_REG["STANDBY"], 0x00)
        self.sleep_ms(int(settle["release_ms"]))
        self.write(_REG["XMSTA"], 0x00)
        self.sleep_ms(int(settle["start_ms"]))
        self.store_param(LINE_TIME_PARAM, achieved)
        self.store_param(FRAME_LENGTH_PARAM, vmax)

    def _park(self):
        self.write(_REG["STANDBY"], 0x01)
        self.sleep_ms(int(_SETTLE["stop_ms"]))

    def _start(self):
        settle = _SETTLE["start"]
        self.write(_REG["STANDBY"], 0x00)
        self.sleep_ms(int(settle["release_ms"]))
        self.write(_REG["XMSTA"], 0x00)
        self.sleep_ms(int(settle["start_ms"]))

    def _fast_for_mode(self):
        self.select("mode", {i: (lambda v=frame: self._fast(v))
                             for i, (_, _, frame) in enumerate(_MODES)})

    def _fast(self, trigger_vmax):
        # Through the free-running preset first: fast trigger entered from a
        # triggered state stalls the part.
        self._trigger("freerun", _SETTLE["trigger_switch"])
        self._trigger("fast", _SETTLE["fast_trigger"], vmax=trigger_vmax)
        self.store_param(FRAME_LENGTH_PARAM, trigger_vmax)

    def _trigger(self, preset, settle, vmax=None):
        values = _TRIGGER[preset]
        self.write(_REG["STANDBY"], 0x01)
        self.sleep_ms(int(settle["standby_ms"]))
        self.write(_REG["TRIGMODE"], int(str(values["trigmode"]), 0))
        self.write(_REG["VINT_EN"], int(str(values["vint_en"]), 0))
        if vmax is not None:
            self.write_table([(_REG["VMAX"] + i, (vmax >> (8 * i)) & 0xFF)
                              for i in range(3)])
        self.write(_REG["STANDBY"], 0x00)
        self.sleep_ms(int(settle["release_ms"]))
        self.write(_REG["XMSTA"], 0x00)
        self.sleep_ms(int(settle["start_ms"]))
```

The run parameters are the vocabulary the host stages: `action` (0 the whole program, 1 a standby, 2 a start from standby, 3 the timing program alone), `line_time` and `frame_period` in nanoseconds, and `frame_length` in lines, which the timing program stores beside the achieved `line_time`. The timing program is the family's: standby, both registers of the free-run preset, HMAX and VMAX from the staged periods, release, master start, with the `program.timing_start` settles, so a free-running head ends on the free-run preset whatever the tables left. The fast-trigger frame is the mode's own frame; the host's laws hold a synced pair to it. A part without a trigger input drops the `trigger` parameter, the third `select_grouped` and the `_fast*` and `_trigger` blocks. A `<name>_final.yaml` is written by `_configure` after each mode's table. Register addresses, widths, presets and settles come from the YAML so a fact lives once. The class computes nothing but the timing registers: no `knob_*`, `expect_*`, `derive_*` or `export_*` method, the compiler refuses them.

## Step 4: Self-check

```bash
nxs personality check ./<name>
nxs upload ./<name>/<name>.py -o <name>.nxs
```

1. `check` judges the shape against the schema and names the file and key of anything wrong, a mode table that is missing included. It passes with `personality <name>: camera · <compatible> · <N> mode(s)` and `ok`.
2. The compile prints the budget: bytes per block, the dispatch overhead per `select()`, the trailer. Bytecode sits under 4096 B and the trailer under 2048 B. Over the cap: drop the least useful mode with its table and say so on the card; never trim a table.
3. Arithmetic, by hand, per mode: `lanes × rate_mbps × 10^6 ≥ width × height × bit_depth × fps × 1.15`; the datasheet's top rate is at or above the mode's; table-only, `pix_clk_hz / (line_length × max_fps) ≥ height`; `sony_imx`, `inck_hz / (hmax × min_frame_length)` reproduces the datasheet's rate within 1 % and `pix_clk_hz` is a whole multiple of `inck_hz`. A miss is a transcription error in the YAML, never a reason to change a table.
4. Every offered mode has a table, every table row fits `reg_bits` and `val_bits`, and the identity value has `device_id_width` bytes.

## Step 5: Report

Print exactly this card and nothing else:

```
nxs personality: <name> — camera

  files     ./<name>/<name>.py · ./<name>/<name>.yaml
  tables    ./<name>/<name>_init.yaml · ./<name>/<name>_<mode>.yaml[ · …]
  family    <generic|sony_imx>
  address   0x<addr> · registers <reg_bits>-bit · values <val_bits>-bit
  modes     <mode> <W>x<H> RAW<n> <lanes>-lane <fps> fps[ · …]
  trigger   freerun[ · fast]
  budget    <n> B bytecode of 4096 · trailer <n> B of 2048

  nxs personality check ./<name>
  nxs upload ./<name>/<name>.py -o <name>.nxs
  nxs personality install ./<name>
  nxs --experimental <port> <link> on --sensor <name>
  nxs <port> status
  nxs <port> <link> capture --frames 60
```

`<fps>` is `timing.fps` for a table-only part and `inck_hz / (hmax × min_frame_length)` for a `sony_imx` one. The last three lines are the bench: a mode without a `shipped` point runs under `--experimental` until a capture proves its rate; the point then goes into the YAML by hand (the camera reference, §3.1) and the pair is installed again.

When the part is not expressible, print this card instead, write no file, and end the run:

```
nxs personality: <name> — NOT EXPRESSIBLE

  part      <chip> — <interface in one line>
  needs     <the wire behaviour the datasheet requires>
  missing   <the DSL construct that does not exist; quote the exact CompileError when one was raised>
  closest   <the nearest documented construct and why it falls short>
  unblock   <the smallest DSL extension that would cover this part>
```
