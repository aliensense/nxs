# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A camera link's head as a port's graph image runs it: the declared
personality as this host holds it, its run parameters, the rows its camera
door serves and the law its apply converts with (`head`). The composer
stages a head from it, and `set` turns a knob into the one parameter
`nxs_graph_set` stages on a bare head (`bare_knob`)."""

from __future__ import annotations

import dataclasses
import math
from typing import List, Dict, Mapping, Optional, Tuple

from nxs.cam.contracts import InfeasibleConfig, LinkSpec, Topology
from nxs.cam.descriptors import FREERUN
from nxs.personality import records_graph as rg

GraphParam = rg.GraphParam
#: The personality's run parameters by `GraphParam`.
PARAM_NAMES = {GraphParam.MODE: "mode", GraphParam.TRIGGER: "trigger", GraphParam.ACTION: "action",
               GraphParam.LINE_TIME: "line_time", GraphParam.FRAME_PERIOD: "frame_period",
               GraphParam.EXPOSURE: "exposure", GraphParam.GAIN: "gain",
               GraphParam.FRAME_LENGTH: "frame_length"}
#: The knobs a bare head takes through its personality's apply, with the run
#: parameter each stages.
BARE_KNOBS = {"exposure": GraphParam.EXPOSURE, "fps": GraphParam.FRAME_LENGTH,
              "gain": GraphParam.GAIN, "gain_db": GraphParam.GAIN}


@dataclasses.dataclass(frozen=True)
class Law:
    """What a personality's CAMERA_MAP record states its apply converts
    with: the capture units of a decibel, the gain's range and step in
    them, and the line of the mode the link runs, in nanoseconds."""

    gain_factor: int
    gain_min: int
    gain_max: int
    gain_step: int
    line_ns: int


@dataclasses.dataclass(frozen=True)
class Head:
    """A camera link's personality as the walks run it: its name, its
    build (the trailer's CRC-32) and image, its parameter indices in
    `GraphParam` order, the mode's value, the free-run and fast trigger
    values, every row its camera door serves, `(row, kind)` by `ParamId`,
    each door row's name and unit by its index, and its apply's law (None
    for a personality that states none)."""

    name: str
    crc: int
    image: bytes
    params: Tuple[Optional[int], ...]
    mode: int
    trigger_free: int
    trigger_fast: Optional[int]
    rows: Mapping[int, Tuple[int, int]]
    names: Mapping[int, Tuple[str, str]] = dataclasses.field(default_factory=dict)
    law: Optional[Law] = None


def install_hint(sen) -> str:
    """The command that puts a declared personality on this host: the
    assets for one the product ships, else the install of the user's own
    directory."""
    from nxs.cam import cam_personalities

    if cam_personalities.registry().is_shipped(sen.name):
        return "nxs assets install"
    return f"nxs personality install ./{sen.name}"


def personality(hub, spec: LinkSpec, sen):
    """The declared personality of a camera link as this host holds it: its
    name, image, the image read back and its parameter map, from the store
    else compiled from the hub's pair. InfeasibleConfig for a personality
    the host does not hold, an image cut short, one that is no cam
    personality's, a malformed record, or one that describes another
    sensor than `sen`."""
    from nxs import personality_cli
    from nxs.cam import pods
    from nxs.image import IMAGE_KIND_NAMES, ImageKind, deserialize
    from nxs.personality import records

    name = pods.declared_name(hub, spec)
    hint = install_hint(sen)
    try:
        source = personality_cli.resolve(name)
    except personality_cli.ResolveError as exc:
        raise InfeasibleConfig(f"link {spec.name}: no personality {name} on this host ({exc})",
                               alternatives=[hint]) from exc
    if source.kind == personality_cli.KIND_IMAGE:
        image = source.image
        try:
            compiled = deserialize(image)
        except ValueError as exc:
            raise InfeasibleConfig(f"link {spec.name}: {name}: {exc}") from exc
    else:
        try:
            compiled, image = personality_cli.compile_source(source, {})
        except personality_cli.CompileFailed as exc:
            raise InfeasibleConfig(f"link {spec.name}: {name}: {exc}") from exc
    if compiled.kind != ImageKind.CAMERA:
        raise InfeasibleConfig(f"link {spec.name}: {name} is a {IMAGE_KIND_NAMES[compiled.kind]} "
                               f"image, not a cam personality", alternatives=[hint])
    # The store serves the image by its file name alone: the trailer's
    # identity says which sensor's program it is.
    try:
        described = str(records.decode_trailer(compiled.trailer)["meta"]["compatible"])
        params = records.param_map(compiled.trailer)
    except records.RecordError as exc:
        raise InfeasibleConfig(f"link {spec.name}: {name}: {exc}") from exc
    if described != sen.compatible:
        raise InfeasibleConfig(f"link {spec.name}: {name} describes {described}, not "
                               f"{sen.compatible}", alternatives=[hint])
    return name, image, compiled, params


def run_timing(sen, mode: str, frame_length: Optional[int] = None) -> Dict[str, int]:
    """The line and frame periods a run of `mode` stages, in nanoseconds by
    run-param name: the descriptor's line (a port view's is the line the
    pair leaves the mode) and its recommended frame, or `frame_length`
    lines when the caller fixes the frame. A family without a timing law
    stages the line and the frame floor its apply law states for the mode,
    as a mode commit on its camera map does."""
    from nxs.cam import laws
    from nxs.personality.records import FRAME_PERIOD_PARAM, LINE_TIME_PARAM

    module = laws.bind(sen)
    if not hasattr(module, "line_time_us"):
        law = module.apply_law()
        index = sen.program_modes().index(mode)
        line_ns = law.line_times_ns[index]
        floors = law.frame_length_floors
        floor = floors[index] if len(floors) == len(law.line_times_ns) else law.frame_length_min
        lines = floor if frame_length is None else frame_length
        return {LINE_TIME_PARAM: line_ns, FRAME_PERIOD_PARAM: int(lines) * line_ns}
    hmax = int(sen.modes[mode]["timing"]["hmax"])
    line_ns = round(module.line_time_us(hmax) * 1000)
    lines = module.recommended_frame_length(mode) if frame_length is None else frame_length
    return {LINE_TIME_PARAM: line_ns, FRAME_PERIOD_PARAM: int(lines) * line_ns}


def _law(info: Optional[Mapping], mode_value: int) -> Optional[Law]:
    """The apply's law a CAMERA_MAP record states, at the mode `mode_value`."""
    if not info:
        return None
    position = next((i for i, m in enumerate(info["modes"]) if int(m["value"]) == mode_value), 0)
    laws = info["law_modes"]
    line_ns = int(laws[position]["line_time_ns"]) if position < len(laws) else 0
    gain = info["gain"]
    return Law(gain_factor=int(info["gain_factor"]), gain_min=int(gain["min"]),
               gain_max=int(gain["max"]), gain_step=max(1, int(gain["step"])), line_ns=line_ns)


def head(hub, port: Topology, spec: LinkSpec, mode: str, frame_length: Optional[int]
         ) -> Tuple[Head, Dict[str, int]]:
    """A camera link's head and the periods its run stages."""
    from nxs.image import trailer_bytes
    from nxs.personality import records
    from nxs.personality.records import FRAME_PERIOD_PARAM, LINE_TIME_PARAM
    from nxs.personality.records_fields import trailer_crc
    from nxs.cam.select import _link_descriptor

    sen = hub.descriptor(spec.sensor_compatible)
    name, image, compiled, params = personality(hub, spec, sen)
    if params.mode_index is None:
        raise InfeasibleConfig(f"link {spec.name}: the personality {name} carries no parameter "
                               f"records", alternatives=[f"nxs <port> <link> upload {sen.name} "
                                                         f"(the image compiled from its yaml and "
                                                         f"py pair)"])
    if mode not in params.modes:
        raise InfeasibleConfig(f"link {spec.name}: the personality {name} offers no mode {mode!r}",
                               alternatives=sorted(params.modes))
    triggers = dict(params.triggers)
    if params.trigger_index is not None and FREERUN not in triggers:
        raise InfeasibleConfig(f"link {spec.name}: the personality {name} offers no {FREERUN!r} "
                               f"conversion", alternatives=[f"trigger {t}" for t in triggers])
    names = [p.name for p in compiled.params]
    indices = tuple(names.index(PARAM_NAMES[which]) if PARAM_NAMES[which] in names else None
                    for which in sorted(PARAM_NAMES))
    rows: Dict[int, Tuple[int, int]] = {}
    door: Dict[int, Tuple[str, str]] = {}
    info = records.camera_map_info(compiled.trailer)
    for index, row in enumerate((info or {}).get("rows") or []):
        door[index] = (str(row.name), str(row.unit or ""))
        rows.setdefault(int(row.id), (index, int(row.kind)))
    timed = (indices[GraphParam.LINE_TIME] is not None
             and indices[GraphParam.FRAME_PERIOD] is not None)
    periods = run_timing(_link_descriptor(hub, port, spec), mode, frame_length) if timed else {}
    found = Head(name=name, crc=trailer_crc(trailer_bytes(compiled.trailer)), image=bytes(image),
                 params=indices, mode=int(params.modes[mode]),
                 trigger_free=int(triggers.get(FREERUN, 0)), trigger_fast=triggers.get("fast"),
                 rows=rows, names=door, law=_law(info, int(params.modes[mode])))
    return found, {"line": int(periods.get(LINE_TIME_PARAM, 0)),
                   "frame": int(periods.get(FRAME_PERIOD_PARAM, 0))}


def gain_units(found: Head, compatible: str, gain_db: float, place: str = "gain_db") -> int:
    """`gain_db` in the capture units the head's apply takes, held to the
    range and the step its law states; `place` names where the gain is
    written in the alternatives, the declaration's key or the knob.

    Raises:
        InfeasibleConfig: A head whose personality states no gain law; a
            gain below its floor or past its ceiling, or between two of
            its steps, naming the gains that fit.
    """
    law = found.law
    if law is None or law.gain_factor < 1:
        raise InfeasibleConfig(f"{compatible} states no gain law", alternatives=["drop the key"])
    floor, ceiling = law.gain_min / law.gain_factor, law.gain_max / law.gain_factor
    if gain_db < floor - 1e-9:
        raise InfeasibleConfig(f"{gain_db:g} is below {compatible}'s {floor:g} dB",
                               alternatives=[f"{place}: {floor:g}"])
    if gain_db > ceiling + 1e-9:
        raise InfeasibleConfig(f"{gain_db:g} is past {compatible}'s {ceiling:g} dB",
                               alternatives=[f"{place}: {ceiling:g}"])
    steps = (gain_db * law.gain_factor - law.gain_min) / law.gain_step
    if abs(steps - round(steps)) > 1e-6:
        step_db = law.gain_step / law.gain_factor
        places = next((p for p in range(4) if abs(step_db * 10 ** p - round(step_db * 10 ** p))
                       < 1e-9), 3)
        below = math.floor(steps)
        raise InfeasibleConfig(
            f"{gain_db:g} is not a gain {compatible} takes ({step_db:g} dB steps)",
            alternatives=[f"{place}: {(law.gain_min + n * law.gain_step) / law.gain_factor:.{places}f}"
                          for n in (below, below + 1)])
    return law.gain_min + round(steps) * law.gain_step


def bare_knobs(found: Head) -> List[str]:
    """The knobs `bare_knob` takes on `found`: those its apply declares a
    parameter for, `gain_db` where its law scales decibels."""
    names = [knob for knob, which in BARE_KNOBS.items() if found.params[which] is not None]
    if found.params[GraphParam.ACTION] is None or found.law is None:
        return []
    if found.law.gain_factor < 1 and "gain_db" in names:
        names.remove("gain_db")
    return sorted(names)


def bare_knob(hub, port: Topology, spec: LinkSpec, found: Head, knob: str, value: str,
              mode: str, line_ns: int = 0) -> Tuple[int, int, str]:
    """A knob on a bare head as its personality's apply takes it: the run
    parameter's index, the value in its unit, and the value in the
    knob's words for the line `set` prints. The exposure is in lines and
    the rate a frame length, at the line the link runs (`line_ns`, the
    graph's, else the mode's own); the gain is in capture units, from the
    analog register's value or from decibels.

    Raises:
        InfeasibleConfig: A knob the apply takes no parameter for; a value
            no number or none the law takes; a personality without the
            parameter.
    """
    from nxs.cam import laws
    from nxs.cam.map_knob import parse_us, rate
    from nxs.cam.select import _link_descriptor

    takes = bare_knobs(found)
    if knob not in takes:
        raise InfeasibleConfig(f"link {spec.name}: {knob} is no parameter of {found.name}'s apply, "
                               f"which a bare head takes its knobs through",
                               alternatives=[f"{k} <value>" for k in takes])
    index = found.params[BARE_KNOBS[knob]]
    sen = _link_descriptor(hub, port, spec)
    module = laws.bind(sen)
    line_ns = line_ns or found.law.line_ns
    if knob == "exposure":
        try:
            us = parse_us(value)
        except ValueError:
            raise InfeasibleConfig(f"exposure takes a time: 4ms, 4000us or 4000, not {value!r}",
                                   alternatives=["exposure <us|ms>"]) from None
        if not math.isfinite(us) or us <= 0 or line_ns <= 0:
            raise InfeasibleConfig(f"exposure takes a time: 4ms, 4000us or 4000, not {value!r}",
                                   alternatives=["exposure <us|ms>"])
        return index, max(1, round(us * 1000 / line_ns)), f"{us:g} us"
    if knob == "fps":
        fps = rate(value)
        hmax = (sen.modes.get(mode) or {}).get("timing", {}).get("hmax")
        if hasattr(module, "vmax_for_fps") and hmax is not None:
            return index, int(module.vmax_for_fps(fps, int(hmax))), f"{fps:g} fps"
        return index, max(1, round(1e9 / (fps * line_ns))), f"{fps:g} fps"
    if knob == "gain_db":
        try:
            db = float(value)
        except ValueError:
            db = math.nan
        if not math.isfinite(db):
            raise InfeasibleConfig(f"gain_db takes decibels, not {value!r}",
                                   alternatives=["gain_db <dB>"])
        return index, gain_units(found, sen.compatible, db), f"{db:g} dB"
    try:
        register = int(value, 0)
    except ValueError:
        raise InfeasibleConfig(f"gain takes the analog gain register's value, not {value!r}",
                               alternatives=["gain <register value>", "gain_db <dB>"]) from None
    law = module.apply_law()
    units = round(register * law.gain_den / law.gain_num)
    if not found.law.gain_min <= units <= found.law.gain_max:
        top = found.law.gain_max * law.gain_num // law.gain_den
        raise InfeasibleConfig(f"gain {register} outside 0..{top}",
                               alternatives=[f"gain {min(max(register, 0), top)}"])
    return index, units, f"{register}"
