# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs mcp`: the nxs verbs as tools an AI agent can call. Every tool runs the
`nxs` command line as a subprocess, so the agent gets the same verbs and
refusals; needs the `mcp` extra (`pip install 'aliensense-nxs[mcp]'`), runs where the buses are."""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from nxs import term


INSTRUCTIONS = """\
nxs drives Aliensense NXS sensor units and camera ports on this machine.
Three doors edit one declared config (suite.yaml): hand-written YAML,
`nxs tune`, and these tools, all validated by the same laws. Read
before you write: generate lists what answers on the rig, caps lists what
a port's sensor offers, status tells you whether the declaration is
lawful and what deviates from it, suite_get shows every option.
Hardware effects are real: on writes registers and trains links, off
parks sensors, reload reconverges the running daemon. A refusal is a fact
and its lawful alternatives; under a JSON tool it is the document
{"refused": {"fact": "…", "alternatives": ["…"]}}. Use an alternative
instead of retrying the same value."""


class Refusal(Exception):
    """The verb refused (non-zero exit); the message is the refusal."""


@dataclass(frozen=True)
class ToolSpec:
    """One tool: its verb, how to describe it, and how it may act."""
    name: str
    fn: Callable[..., Any]
    purpose: str
    arguments: str
    effect: str
    refusal: str
    read_only: bool = False
    destructive: bool = False
    idempotent: bool = False


def _cli() -> List[str]:
    return [sys.executable, "-m", "nxs.cli"]


def _run(argv: List[str], timeout: float) -> "tuple[int, str]":
    """Run `nxs argv…`; (exit code, output without colour). A JSON surface
    (`--json` in argv) is stdout alone: the verb's progress lines on stderr
    stay diagnostics, and a refused verb's document rides stdout too. A
    text verb's output is stdout and stderr together."""
    try:
        proc = subprocess.run(_cli() + argv, capture_output=True, text=True,
                              timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, f"nxs {' '.join(argv)}: no answer within {timeout:.0f} s"
    stdout = term.strip_ansi(proc.stdout or "").strip()
    stderr = term.strip_ansi(proc.stderr or "").strip()
    if "--json" in argv:
        return proc.returncode, stdout or stderr
    return proc.returncode, "\n".join(part for part in (stdout, stderr) if part)


def _call(argv: List[str], timeout: float = 60.0) -> str:
    rc, out = _run(argv, timeout)
    if rc != 0:
        raise Refusal(out or f"nxs {' '.join(argv)}: exit {rc}")
    return out


def _call_json(argv: List[str], timeout: float = 60.0) -> Dict[str, Any]:
    out = _call(argv, timeout)
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        raise Refusal(f"nxs {' '.join(argv)}: not a JSON surface:\n{out}")


def _node(port: str, link: Optional[str], *rest: str) -> List[str]:
    """`nxs <port> [<link>] <verb> …` in the node grammar."""
    argv = [port]
    if link:
        argv.append(link.upper())
    argv.extend(rest)
    return argv


# ---------------------------
# Live verbs: the hardware
# ---------------------------

def probe(port: Optional[str] = None, link: Optional[str] = None) -> dict:
    """The units that answer. Without a port: config-free discovery of
    every bus. With a port: the units answering on that port's bus. The
    hub, the links and the sensors of a port are `status`'s."""
    if port is None:
        return _call_json(["probe", "--json"])
    return _call_json(_node(port, link, "probe", "--json"))


def generate(dry_run: bool = True) -> dict:
    """Walk the rig and return what answered: every camera port with its hub,
    its links, their serializers, sensors and units, or the sensor and the
    unit on the port's own bus, then the units on the bare buses. A node the
    wiring names and nothing answers for is `unanswered`. With `dry_run`
    (the default) nothing is written; without it hardware.yaml, the report
    of what answered, is written from the walk, suite.yaml is seeded when
    there is none, and a unit running nothing is named by trying every
    personality on it."""
    argv = ["generate", "--json"]
    if dry_run:
        argv.append("--dry-run")
    return _call_json(argv, timeout=300.0)


def _unit(unit: Optional[str], *rest: str) -> List[str]:
    """`nxs [--unit <name>] <verb> …`: a declared unit by name, else the
    one unit answering on the camera buses."""
    return (["--unit", unit] if unit else []) + list(rest)


def identify(unit: Optional[str] = None) -> str:
    """Strobe a unit's LED for about ten seconds so a person can tell
    which box it is. `unit` is a declared name; omitted, the one unit
    answering on the camera buses."""
    return _call(_unit(unit, "identify"), timeout=30.0)


def samples(unit: Optional[str] = None, count: int = 5) -> dict:
    """Read `count` decoded samples from a unit: the fields with their
    units, then each sample's values in SI, as `nxs stream` decodes
    them. `unit` as for identify."""
    if int(count) < 1:
        raise Refusal(f"samples: count is at least 1, not {count}")
    return _call_json(_unit(unit, "stream", "--count", str(int(count)), "--json"),
                      timeout=60.0)


def status(port: Optional[str] = None, link: Optional[str] = None) -> dict:
    """Health. Without a port: the declared-vs-actual tree from the
    manifest. With a port: the descriptor-driven diagnosis (link locks,
    video lock per pipe, CSI gate, sensor timing readbacks)."""
    if port is None:
        return _call_json(["status", "--json"])
    return _call_json(_node(port, link, "status", "--json"))


def caps(port: str, link: Optional[str] = None) -> dict:
    """What the port's sensor offers on this port: the shipped modes with
    their fps ranges and the proof's stamp, the knob names, the trigger
    modes. No register is touched."""
    return _call_json(_node(port, link, "caps", "--json"))


def on(port: str, link: Optional[str] = None, mode: Optional[str] = None,
       fps: Optional[float] = None, dry_run: bool = False,
       sensor: Optional[str] = None) -> str:
    """Bring a port up: one link's, or with `link` omitted the whole port's (every
    declared link together). `mode` is a mode token (WxH, WxH-rawN, or a mode name)
    among the shipped modes `caps` lists, `fps` a rate inside the mode's shipped
    range (the ceiling without one), `sensor` names the pack sensor; dry_run prints
    the plan without touching the bus."""
    argv = _node(port, link, "on")
    if sensor:
        argv += ["--sensor", sensor]
    if mode:
        argv += ["--mode", mode]
    if fps is not None:
        argv += ["--fps", str(float(fps))]
    if dry_run:
        argv.append("--dry-run")
    return _call(argv, timeout=180.0)


def off(port: str) -> str:
    """Park the whole port: stop its viewers, put every sensor in standby,
    close the CSI gate. There is no one-link park (the gate and the park
    program are port-wide); a single viewer closes from its own window."""
    return _call(_node(port, None, "off"), timeout=60.0)


def capture(port: str, link: str, frames: int = 4,
            timeout_s: Optional[float] = None) -> str:
    """Headless delivery check on an up link: a choreographed capture session must
    deliver `frames` frames; a locked link says nothing about frames."""
    argv = _node(port, link, "capture", "--frames", str(int(frames)))
    if timeout_s:
        argv += ["--timeout", str(float(timeout_s))]
    return _call(argv, timeout=(timeout_s or 20.0 + frames / 5.0) + 60.0)


def get(port: str, link: str, knob: str) -> str:
    """Read a knob back: the port's `sync`, a sensor knob (fps, exposure,
    gain) on an up link, else the link's unit's parameter."""
    return _call(_node(port, link, "get", knob))


def set_knob(port: str, link: Optional[str], knob: str, value: str,
             dry_run: bool = False, fps: Optional[float] = None,
             exposure_us: Optional[float] = None) -> str:
    """Change a knob under the pack's laws; an infeasible value is refused
    naming the lawful alternatives. `sync` is the port's frame sync (`link`
    omitted): `fsync` starts the hub's generator at `fps` with the synced
    sensors on their trigger and `exposure_us` under it, `free_run` returns
    to free run. dry_run prints the write stream. Refused on a live sibling
    link."""
    argv = _node(port, link, "set", knob, str(value))
    if fps is not None:
        argv += ["--fps", str(float(fps))]
    if exposure_us is not None:
        argv += ["--exposure", str(float(exposure_us))]
    if dry_run:
        argv.append("--dry-run")
    return _call(argv, timeout=120.0)


# ---------------------------
# Declarative verbs: the manifest
# ---------------------------

def suite_get() -> dict:
    """The declared config as options: every channel, its sections, each field's
    value and the values it may take. A channel with declared: false is a
    platform port or an answering unit whose DECLARE knobs write its entry."""
    return _call_json(["tune", "--list", "--json"])


def suite_schema() -> dict:
    """The rig's rules as JSON Schema (2020-12): the manifest schema narrowed
    by the nodes on this rig. Each port lists the keys its nodes bring, each
    link the sensors its port's pack serves, each sensor its modes and each
    mode the rates it ships at; a unit's personality lists its config keys.
    Validate a declaration against it before writing it. It is necessary,
    not sufficient: `status` judges the arithmetic across nodes."""
    return _call_json(["tune", "--schema", "--json"])


def suite_set(channel: str, field: str, value: str,
              section: Optional[str] = None) -> dict:
    """Set one declared value from its offered options (see suite_get) and save
    the manifest with a timestamped backup; the result carries the check
    findings. Nothing converges until `switch` (units) or `reload` (ports)."""
    address = f"{channel}:{section}:{field}" if section else f"{channel}:{field}"
    rc, out = _run(["tune", "--set", f"{address}={value}", "--json"],
                   timeout=60.0)
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        raise Refusal(out)


def switch(dry_run: bool = False, accept_new_serial: bool = False) -> str:
    """Apply the saved declaration to the units: a unit whose personality or settings
    differ is retuned or re-uploaded, a matching one is left alone. Refused
    while the manifest is out of tune or a declared unit does not answer."""
    argv = ["switch"]
    if dry_run:
        argv.append("--dry-run")
    if accept_new_serial:
        argv.append("--accept-new-serial")
    return _call(argv, timeout=300.0)


def reload() -> dict:
    """Apply the saved declaration to the ports: signal the running nxsd to
    reconverge the ports whose declaration changed. Refused while the manifest
    is out of tune."""
    rc, out = _run(["tune", "--play", "--json"], timeout=60.0)
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        raise Refusal(out)


def freeze(unit: Optional[str] = None, dry_run: bool = False) -> str:
    """Adopt live tuning into the manifest (the device wins): one unit
    by name, or every declared unit. The live-first flow's last step."""
    argv = ["tune", "--freeze"] + (["--unit", unit] if unit else [])
    if dry_run:
        argv.append("--dry-run")
    return _call(argv, timeout=180.0)


def _target(port: Optional[str], link: Optional[str], unit: Optional[str]) -> List[str]:
    """How a unit verb addresses its unit: the node (`cam1 A`), a declared
    unit by name, or nothing (the one unit answering on the camera buses)."""
    if port:
        return _node(port, link)
    return ["--unit", unit] if unit else []


def upload(name: str, port: Optional[str] = None, link: Optional[str] = None,
           unit: Optional[str] = None, params: Optional[List[str]] = None,
           slot: Optional[int] = None, compile_only: bool = False) -> str:
    """Compile and upload a personality (a name, a `.nxs`, a `.py`, or the
    `.yaml` of a source pair) to a unit: a driver runs, a camera personality
    lands in a store slot the camera verbs run it from. The unit is the node
    (`port`, `link`), a declared `unit`, or the one unit answering on the
    camera buses. `params` are `key=value` strings (mode=1). With
    `compile_only` no unit is touched and the budget report is returned."""
    import os
    import tempfile

    if compile_only:
        fd, path = tempfile.mkstemp(suffix=".nxs", prefix="nxs-personality-")
        os.close(fd)
        try:
            argv = ["upload", name, "-o", path]
            if params:
                argv += ["--param", *[str(p) for p in params]]
            return _call(argv, timeout=120.0)
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass
    argv = _target(port, link, unit) + ["upload", name]
    if params:
        argv += ["--param", *[str(p) for p in params]]
    if slot is not None:
        argv += ["--slot", str(int(slot))]
    return _call(argv, timeout=180.0)


def host_info() -> dict:
    """The capture host: which box, its boot label and overlays, and per
    port the booted lane count, capture ids and the sensors its table
    serves."""
    return _call_json(["host", "info", "--json"])


def host_modes(port: str) -> dict:
    """The booted capture table of a port: every mode the host can capture there
    and the sensors it serves. A sensor mode missing here needs `switch` and a
    reboot."""
    return _call_json(["host", "modes", port, "--json"])


TOOLS: List[ToolSpec] = [
    ToolSpec("probe", probe, "the units that answer, on every bus or a port's",
             "port?, link?", "none (reads identity registers)",
             "no ACK rows; unknown port lists the ports", read_only=True),
    ToolSpec("generate", generate, "the rig as it answers: ports, hubs, links, "
                                   "sensors, units",
             "dry_run?",
             "walks every camera port and bus; with dry_run false it writes "
             "hardware.yaml (the report of what answered), seeds suite.yaml when "
             "there is none, and names a unit running nothing by trying every "
             "personality on it",
             "no pack and no camera port; the wiring file is not writable"),
    ToolSpec("status", status, "the declaration against the rig; a port's "
                               "presence and health",
             "port?, link?", "none (reads identity and status registers)",
             "hub does not answer", read_only=True),
    ToolSpec("identify", identify, "strobe a unit's LED to find the box",
             "unit?", "the unit's LED strobes ~10 s", "unit does not answer",
             idempotent=True),
    ToolSpec("samples", samples, "read decoded samples from a unit",
             "unit?, count?",
             "reads the sample window; over Cyphal it sets the output "
             "decimation for the read and puts the previous value back",
             "no personality measuring; unit does not answer"),
    ToolSpec("caps", caps, "what the sensor offers, with the laws",
             "port, link?", "none (descriptor data)",
             "no descriptor pack covers the chip", read_only=True),
    ToolSpec("on", on, "bring a link or the whole port up",
             "port, link?, sensor?, mode?, fps?, dry_run?",
             "writes the program, trains links, follows video lock",
             "the mode or rate is not shipped; hub does not answer; video "
             "did not lock"),
    ToolSpec("off", off, "park the whole port", "port",
             "sensors to standby, CSI gate closed, viewers stopped",
             "kernel-owned hub"),
    ToolSpec("capture", capture, "headless delivery proof", "port, link, frames?, timeout_s?",
             "opens a capture session on the up link",
             "link not up; frames not delivered"),
    ToolSpec("get", get, "read a knob: the port's sync, the sensor's, the unit's",
             "port, link, knob",
             "none (reads sensor registers)", "unknown knob lists the knobs",
             read_only=True),
    ToolSpec("set", set_knob, "change a knob under the laws; sync is the port's "
                              "frame sync",
             "port, link?, knob, value, dry_run?, fps?, exposure_us?",
             "writes sensor registers; sync starts or stops the hub's generator",
             "an unlawful value, with the lawful alternatives; live sibling link; "
             "port not up"),
    ToolSpec("suite_get", suite_get, "the declaration as options", "—",
             "none", "no camera port on the host and no manifest",
             read_only=True, idempotent=True),
    ToolSpec("suite_schema", suite_schema, "the rig's rules as JSON Schema", "—",
             "none", "the manifest does not parse",
             read_only=True, idempotent=True),
    ToolSpec("suite_set", suite_set, "set one declared value from its options",
             "channel, field, value, section?",
             "writes suite.yaml (timestamped backup; created when absent)",
             "value not among the options; ambiguous field"),
    ToolSpec("switch", switch, "apply the saved declaration to the units",
             "dry_run?, accept_new_serial?",
             "retunes or re-uploads the personality on units whose entry differs",
             "manifest out of tune; a declared unit does not answer"),
    ToolSpec("reload", reload, "apply the saved declaration to the ports", "—",
             "nxsd reconverges changed ports",
             "manifest out of tune; nxsd not running"),
    ToolSpec("freeze", freeze, "adopt live tuning into the manifest",
             "unit?, dry_run?", "writes suite.yaml", "unit unreachable"),
    ToolSpec("upload", upload,
             "compile and upload a personality to a unit (or compile only)",
             "name, port?, link?, unit?, params?, slot?, compile_only?",
             "a driver runs; a camera personality lands in a store slot",
             "unknown name; the source does not compile; store full; several "
             "units answer and none is named"),
    ToolSpec("host_info", host_info, "the capture host and what it booted",
             "—", "none (reads the device tree and boot config)", "never",
             read_only=True, idempotent=True),
    ToolSpec("host_modes", host_modes, "the booted capture table of a port",
             "port", "none (reads the device tree)",
             "unknown port; device tree silent", read_only=True, idempotent=True),
]


def tool_names() -> List[str]:
    return [t.name for t in TOOLS]


def doc_table() -> str:
    """The Tools table of the MCP Tool Reference, generated from TOOLS so the
    document cannot drift from the server."""
    lines = ["| Tool | Purpose | Arguments | Hardware effect | Refuses when |",
             "|---|---|---|---|---|"]
    for t in TOOLS:
        lines.append(f"| `{t.name}` | {t.purpose} | {t.arguments} | "
                     f"{t.effect} | {t.refusal} |")
    return "\n".join(lines)


def build_server():
    """An MCPServer carrying every tool (needs the `mcp` extra)."""
    from mcp.server.mcpserver import MCPServer
    from mcp.types import ToolAnnotations

    try:
        from importlib.metadata import version as _dist_version
        version = _dist_version("nxs")
    except Exception:  # a source checkout without the wheel's metadata
        version = "0"

    server = MCPServer(name="nxs", instructions=INSTRUCTIONS,
                       version=str(version))
    for spec in TOOLS:
        server.add_tool(
            _wrap(spec.fn), name=spec.name, description=spec.fn.__doc__,
            annotations=ToolAnnotations(
                title=spec.purpose, read_only_hint=spec.read_only,
                destructive_hint=spec.destructive,
                idempotent_hint=spec.idempotent, open_world_hint=False),
        )
    return server


def _wrap(fn):
    """A refusal becomes the tool error the agent reads."""
    import functools

    try:
        from mcp.server.mcpserver.exceptions import ToolError
    except ImportError:  # pragma: no cover - the extra is present here
        ToolError = RuntimeError

    @functools.wraps(fn)
    def call(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except Refusal as exc:
            raise ToolError(str(exc)) from None
    return call


def cmd_mcp(args) -> int:
    if getattr(args, "doc_table", False):
        print(doc_table())
        return 0
    try:
        import mcp  # noqa: F401
    except ImportError:
        raise SystemExit("nxs mcp needs: pip install 'aliensense-nxs[mcp]'")
    build_server().run(transport="stdio")
    return 0
