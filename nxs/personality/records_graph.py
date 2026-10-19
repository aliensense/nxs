# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The records of a graph image and the HUB_PHASES record of a hub image.
A graph image is a hub program's sealed section under the records of one
rig. GRAPH_SOURCE names the program the trailer composes against and the
declaration it compiles. GRAPH_LINKS states each link, GRAPH_OUTPUTS each
CSI output of the hub, and GRAPH_WALKS the base staging and the walks the
host library runs. A hub image names its phases in HUB_PHASES, and a
graph image carries its program's. The layouts are `GraphSourceField`,
`GraphLinkField`, `GraphOutputField`, `GraphWalksField`, `GraphStageField`,
`GraphWalkField`, `GraphStepField` and `HubPhaseField` in
constants/nxs_map.yaml."""

from __future__ import annotations

import dataclasses
import hashlib
import struct
import zlib
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from nxs._generated_constants import NxsDriverImage, NxsMap
from nxs.personality.records_fields import RecordError, _u

TrailerRecord = NxsDriverImage.TrailerRecord
ParamKind = NxsDriverImage.ParamKind
GraphWalk = NxsMap.GraphWalk
StepKind = NxsMap.GraphStepKind
HostOp = NxsMap.GraphHostOp
Owner = NxsMap.GraphOwner
OutputMode = NxsMap.GraphOutputMode
Consumer = NxsMap.GraphConsumer
GraphParam = NxsMap.GraphParam

GRAPH_SOURCE = TrailerRecord.GRAPH_SOURCE
GRAPH_LINKS = TrailerRecord.GRAPH_LINKS
GRAPH_OUTPUTS = TrailerRecord.GRAPH_OUTPUTS
GRAPH_WALKS = TrailerRecord.GRAPH_WALKS
HUB_PHASES = TrailerRecord.HUB_PHASES
#: The records a composer hands a graph image; the program's HUB_PHASES joins them.
GRAPH_RECORDS = (GRAPH_SOURCE, GRAPH_LINKS, GRAPH_OUTPUTS, GRAPH_WALKS)

NONE = NxsMap.GRAPH_NONE
REG_NONE = NxsMap.GRAPH_REG_NONE
#: The Hub's companions, compiled into its program: a declaration that puts
#: the serializer or the expander elsewhere is refused.
SER_ADDR = NxsMap.GRAPH_SER_ADDR
TCA_ADDR = NxsMap.GRAPH_TCA_ADDR
#: The pod actions a POD step names, the firmware's `CamAction`.
ACTIONS = ("configure", "park", "start", "timing", "apply", "trigger")
CONFIGURE = 0

_SOURCE = struct.Struct("<I16s16sBBBx")
_LINK = struct.Struct("<cBBBBBBB4sBBBB16sI16sI8sBB2xIIII4x")
_OUTPUT = struct.Struct("<BBHBBBB")
_WALKS_HEAD = struct.Struct("<BBBBHH")
_STAGE = struct.Struct("<BI")
_WALK_HEAD = struct.Struct("<BB")
_STEP = struct.Struct("<BBBBI")
_PHASE = struct.Struct("<B16s")
assert _SOURCE.size == NxsMap.GRAPH_SOURCE_SIZE
assert _LINK.size == NxsMap.GRAPH_LINK_ROW_SIZE
assert struct.calcsize("<cBBBBBBB4sBBBB") == NxsMap.GraphLinkField.PERSONALITY
assert struct.calcsize("<cBBBBBBB4sBBBB16sI16sI") == NxsMap.GraphLinkField.PARAMS
assert struct.calcsize("<cBBBBBBB4sBBBB16sI16sI8sBB2x") == NxsMap.GraphLinkField.LINE_TIME_NS
assert _OUTPUT.size == NxsMap.GRAPH_OUTPUT_ROW_SIZE
assert _WALKS_HEAD.size == NxsMap.GRAPH_WALKS_HEAD_SIZE
assert _STAGE.size == NxsMap.GRAPH_STAGE_SIZE
assert _WALK_HEAD.size == NxsMap.GRAPH_WALK_HEAD_SIZE
assert _STEP.size == NxsMap.GRAPH_STEP_SIZE
assert _PHASE.size == NxsMap.HUB_PHASE_ROW_SIZE

#: Bytes of a name field; a name keeps its last byte NUL.
_NAME_LEN = 16
_PARAM_COUNT = NxsMap.GraphLinkField.TRIGGER_FREE - NxsMap.GraphLinkField.PARAMS
_INDEX_SHIFT = 4
_PARAMS_MAX = NxsDriverImage.MAX_PARAMS


def program_crc(nonce: bytes, bytecode: bytes) -> int:
    """The CRC-32 of a program section as the image stores it: a sealed
    section's nonce, then its ciphertext; a plain one's bytes alone."""
    return zlib.crc32(bytes(nonce) + bytes(bytecode)) & 0xFFFFFFFF


def declaration_digest(declaration: bytes) -> bytes:
    """The digest GRAPH_SOURCE carries: the first 16 bytes of the SHA-256
    of the declaration's bytes."""
    return hashlib.sha256(bytes(declaration)).digest()[:NxsMap.GRAPH_DIGEST_SIZE]


# ── the records ──────────────────────────────────────────────────────

@dataclasses.dataclass(frozen=True)
class GraphSource:
    """The program a graph image composes against and the declaration it
    compiles: the program's name and the CRC-32 of its section
    (`program_crc`), the declaration's digest, the hub's address, the
    owner, and what the declaration asks past the bring-up."""

    program: str
    program_crc: int
    digest: bytes
    des_addr: int
    owner: int = Owner.HOST
    homogeneous: bool = False
    sync: bool = False
    gain: bool = False


@dataclasses.dataclass(frozen=True)
class GraphLink:
    """One link as a walk stages it. `params` holds the personality's
    parameter indices in `GraphParam` order, None for one it does not
    declare. A link with a pod names the personality its slot holds and
    the CRC-32 of that build's trailer; a camera without a pod names the
    image the owner runs at its head."""

    name: str
    head_addr: int = 0
    lanes: int = 4
    data_type: int = 0
    present: bool = True
    camera: bool = True
    host_csi: bool = False
    frame_fixed: bool = False
    ser_addr: int = SER_ADDR
    tca_addr: int = TCA_ADDR
    entries: Tuple[Tuple[int, int], ...] = ()
    pod: bool = False
    pod_alias: int = 0
    pod_target: int = 0
    pod_map: int = 0
    personality: str = ""
    personality_crc: int = 0
    head_image: str = ""
    head_image_crc: int = 0
    params: Tuple[Optional[int], ...] = (None,) * _PARAM_COUNT
    mode: int = 0
    trigger_free: int = 0
    trigger_fast: Optional[int] = None
    line_time_ns: int = 0
    frame_period_ns: int = 0
    gain_mdb: int = 0
    exposure_us: int = 0

    def param(self, which: int) -> Optional[int]:
        """The personality's index of `which`, a `GraphParam`, None when it
        declares none."""
        return self.params[int(which)]


@dataclasses.dataclass(frozen=True)
class GraphOutput:
    """One CSI output of the hub: its lanes and rate, the pipes it carries
    (`GRAPH_STREAM_*` bits), how it shares them, who receives it, and the
    virtual channel of each link's stream on it."""

    output: int
    lanes: int
    rate_mbps: int
    streams: int = 0
    mode: int = OutputMode.OFF
    consumer: int = Consumer.NONE
    vc_a: int = 0
    vc_b: int = 0


@dataclasses.dataclass(frozen=True)
class GraphStep:
    """One step of a walk: its kind, the link row it acts on (`NONE` for
    none), the phase, action, row or host operation, the flags with an
    override's parameter in their high bits, and the value."""

    kind: int
    link: int = NONE
    op: int = 0
    flags: int = 0
    value: int = 0

    @property
    def index(self) -> int:
        """The parameter an override stages, or a MAP step's `ParamKind`."""
        return (self.flags & NxsMap.GRAPH_STEP_INDEX) >> _INDEX_SHIFT

    @property
    def overrides(self) -> bool:
        return bool(self.flags & NxsMap.GRAPH_STEP_OVERRIDE)

    @property
    def soft(self) -> bool:
        return bool(self.flags & NxsMap.GRAPH_STEP_SOFT)

    @property
    def again(self) -> bool:
        return bool(self.flags & NxsMap.GRAPH_STEP_AGAIN)


@dataclasses.dataclass(frozen=True)
class GraphWalks:
    """The walks of a graph image: the program's `phase` parameter, the
    base staging every HUB step stages (`(index, value)` pairs), the walks
    by `GraphWalk`, the parameter a phase stores the lock bits in, the
    training passes, and the hub registers whose failed poll is a
    video-lock failure."""

    phase_param: int
    walks: Mapping[int, Tuple[GraphStep, ...]]
    base: Tuple[Tuple[int, int], ...] = ()
    lock_param: Optional[int] = None
    train_rounds: int = 1
    vlock: Tuple[int, ...] = ()


def _flags(override: Optional[Tuple[int, int]], soft: bool, again: bool = False) -> int:
    flags = (NxsMap.GRAPH_STEP_SOFT if soft else 0) | (NxsMap.GRAPH_STEP_AGAIN if again else 0)
    if override is not None:
        flags |= NxsMap.GRAPH_STEP_OVERRIDE | (int(override[0]) << _INDEX_SHIFT)
    return flags


def hub_step(phase: int, override: Optional[Tuple[int, int]] = None, soft: bool = False,
             again: bool = False) -> GraphStep:
    """A phase of the program; `override` stages `(index, value)` over the base."""
    return GraphStep(StepKind.HUB, NONE, int(phase), _flags(override, soft, again),
                     int(override[1]) if override is not None else 0)


def pod_step(link: int, action: int, override: Optional[Tuple[int, int]] = None,
             soft: bool = False) -> GraphStep:
    """A camera run of `action` on link row `link`; `override` stages
    `(index, value)` in the personality's table over the link's staging."""
    return GraphStep(StepKind.POD, int(link), int(action), _flags(override, soft),
                     int(override[1]) if override is not None else 0)


def map_step(link: int, row: int, value: int, kind: int, soft: bool = False) -> GraphStep:
    """Row `row` committed at `value` on link row `link`'s camera door, a
    commit of `kind`."""
    return GraphStep(StepKind.MAP, int(link), int(row),
                     _flags(None, soft) | (int(kind) << _INDEX_SHIFT), int(value))


def host_step(op: int, link: int = NONE, value: int = 0, soft: bool = False) -> GraphStep:
    """A call back to the owner, `GraphHostOp`."""
    return GraphStep(StepKind.HOST, int(link), int(op), _flags(None, soft), int(value))


def converge_step(link: int) -> GraphStep:
    """Link row `link`'s pod brought to its personality and build."""
    return GraphStep(StepKind.CONVERGE, int(link))


# ── encode and decode ────────────────────────────────────────────────

def _name(value: str, what: str, allow_empty: bool = False) -> bytes:
    try:
        raw = str(value).encode("ascii")
    except UnicodeEncodeError:
        raise RecordError(f"{what}: {value!r} is not ASCII") from None
    if (not raw and not allow_empty) or len(raw) >= _NAME_LEN:
        raise RecordError(f"{what}: {value!r} takes 1 to {_NAME_LEN - 1} characters")
    return raw


def _text(raw: bytes, what: str) -> str:
    try:
        return raw.rstrip(b"\0").decode("ascii")
    except UnicodeDecodeError:
        raise RecordError(f"{what} carries a byte past ASCII") from None


def _rows(payload: bytes, size: int, what: str) -> List[bytes]:
    if len(payload) % size:
        raise RecordError(f"{what} record of {len(payload)} B is not whole {size}-byte rows")
    return [payload[pos:pos + size] for pos in range(0, len(payload), size)]


def encode_source(source: GraphSource, what: str = "graph") -> bytes:
    """The GRAPH_SOURCE payload."""
    digest = bytes(source.digest)
    if len(digest) != NxsMap.GRAPH_DIGEST_SIZE:
        raise RecordError(f"{what}: a declaration digest is {NxsMap.GRAPH_DIGEST_SIZE} B, "
                          f"got {len(digest)}")
    if int(source.owner) not in Owner._NAMES:
        raise RecordError(f"{what}: owner {source.owner!r} is not a GraphOwner")
    flags = ((NxsMap.GRAPH_FLAG_HOMOGENEOUS if source.homogeneous else 0)
             | (NxsMap.GRAPH_FLAG_SYNC if source.sync else 0)
             | (NxsMap.GRAPH_FLAG_GAIN if source.gain else 0))
    return _SOURCE.pack(_u(source.program_crc, 32, f"{what} program CRC"),
                        _name(source.program, f"{what} program"), digest,
                        _u(source.des_addr, 7, f"{what} hub address"), int(source.owner), flags)


def decode_source(payload: bytes) -> GraphSource:
    """The GRAPH_SOURCE payload back."""
    if len(payload) != _SOURCE.size:
        raise RecordError(f"GRAPH_SOURCE record of {len(payload)} B, the record is "
                          f"{_SOURCE.size} B")
    crc, program, digest, des_addr, owner, flags = _SOURCE.unpack(payload)
    return GraphSource(program=_text(program, "GRAPH_SOURCE program"), program_crc=crc,
                       digest=bytes(digest), des_addr=des_addr, owner=owner,
                       homogeneous=bool(flags & NxsMap.GRAPH_FLAG_HOMOGENEOUS),
                       sync=bool(flags & NxsMap.GRAPH_FLAG_SYNC),
                       gain=bool(flags & NxsMap.GRAPH_FLAG_GAIN))


def _index(value: Optional[int], what: str) -> int:
    """A parameter's index in an image's table, `NONE` for None."""
    if value is None:
        return NONE
    index = int(value)
    if not 0 <= index < _PARAMS_MAX:
        raise RecordError(f"{what}: parameter {index} is past the {_PARAMS_MAX} an image declares")
    return index


def encode_links(links: Sequence[GraphLink], what: str = "graph") -> bytes:
    """The GRAPH_LINKS payload, `GRAPH_LINKS_MAX` rows at most, each name once."""
    if len(links) > NxsMap.GRAPH_LINKS_MAX:
        raise RecordError(f"{what}: {len(links)} links, a graph carries "
                          f"{NxsMap.GRAPH_LINKS_MAX} at most")
    names = [link.name for link in links]
    twice = sorted({n for n in names if names.count(n) > 1})
    if twice:
        raise RecordError(f"{what}: link {', '.join(twice)} declared twice")
    return b"".join(_encode_link(link, what) for link in links)


def _encode_link(link: GraphLink, what: str) -> bytes:
    where = f"{what} link {link.name}"
    if len(str(link.name)) != 1 or not str(link.name).isascii() or not str(link.name).isalpha():
        raise RecordError(f"{where}: a link's name is one letter")
    if len(link.entries) > NxsMap.GRAPH_MAP_MAX:
        raise RecordError(f"{where}: {len(link.entries)} translations, the serializer has "
                          f"{NxsMap.GRAPH_MAP_MAX} slots")
    if len(link.params) != _PARAM_COUNT:
        raise RecordError(f"{where}: {len(link.params)} parameter indices, the row holds "
                          f"{_PARAM_COUNT}")
    entries = bytearray(4)
    for i, (alias, target) in enumerate(link.entries):
        entries[2 * i] = _u(alias, 7, f"{where} alias")
        entries[2 * i + 1] = _u(target, 7, f"{where} target")
    flags = ((NxsMap.GRAPH_LINK_PRESENT if link.present else 0)
             | (NxsMap.GRAPH_LINK_CAMERA if link.camera else 0)
             | (NxsMap.GRAPH_LINK_HOST_CSI if link.host_csi else 0)
             | (NxsMap.GRAPH_LINK_FRAME_FIXED if link.frame_fixed else 0)
             | (NxsMap.GRAPH_LINK_POD if link.pod else 0))
    params = bytes(_index(p, f"{where} {GraphParam._NAMES[i].lower()}")
                   for i, p in enumerate(link.params))
    fast = NONE if link.trigger_fast is None else _u(link.trigger_fast, 8, f"{where} fast trigger")
    if fast == NONE and link.trigger_fast is not None:
        raise RecordError(f"{where}: trigger value {NONE} stands for none")
    return _LINK.pack(
        str(link.name).encode("ascii"), flags, _u(link.ser_addr, 7, f"{where} serializer"),
        _u(link.head_addr, 7, f"{where} head"), _u(link.tca_addr, 7, f"{where} expander"),
        _u(link.lanes, 8, f"{where} lanes"), _u(link.data_type, 8, f"{where} data type"),
        len(link.entries), bytes(entries), _u(link.pod_alias, 7, f"{where} pod alias"),
        _u(link.pod_target, 7, f"{where} pod target"), _u(link.pod_map, 7, f"{where} camera door"),
        _u(link.mode, 8, f"{where} mode"),
        _name(link.personality, f"{where} personality", allow_empty=True),
        _u(link.personality_crc, 32, f"{where} personality CRC"),
        _name(link.head_image, f"{where} head image", allow_empty=True),
        _u(link.head_image_crc, 32, f"{where} head image CRC"), params,
        _u(link.trigger_free, 8, f"{where} free-run trigger"), fast,
        _u(link.line_time_ns, 32, f"{where} line"), _u(link.frame_period_ns, 32, f"{where} frame"),
        _u(link.gain_mdb, 32, f"{where} gain"), _u(link.exposure_us, 32, f"{where} exposure"))


def decode_links(payload: bytes) -> List[GraphLink]:
    """The GRAPH_LINKS payload back to its rows."""
    links = []
    for raw in _rows(payload, _LINK.size, "GRAPH_LINKS"):
        (name, flags, ser, head, tca, lanes, dt, count, entries, alias, target, door, mode,
         personality, crc, head_image, head_crc, params, free, fast, line, frame, gain,
         exposure) = _LINK.unpack(raw)
        if count > NxsMap.GRAPH_MAP_MAX:
            raise RecordError(f"GRAPH_LINKS row counts {count} translations, past "
                              f"{NxsMap.GRAPH_MAP_MAX}")
        links.append(GraphLink(
            name=_text(name, "GRAPH_LINKS name"), head_addr=head, lanes=lanes, data_type=dt,
            present=bool(flags & NxsMap.GRAPH_LINK_PRESENT),
            camera=bool(flags & NxsMap.GRAPH_LINK_CAMERA),
            host_csi=bool(flags & NxsMap.GRAPH_LINK_HOST_CSI),
            frame_fixed=bool(flags & NxsMap.GRAPH_LINK_FRAME_FIXED), ser_addr=ser, tca_addr=tca,
            entries=tuple((entries[2 * i], entries[2 * i + 1]) for i in range(count)),
            pod=bool(flags & NxsMap.GRAPH_LINK_POD), pod_alias=alias, pod_target=target,
            pod_map=door, personality=_text(personality, "GRAPH_LINKS personality"),
            personality_crc=crc, head_image=_text(head_image, "GRAPH_LINKS head image"),
            head_image_crc=head_crc, params=tuple(None if p == NONE else p for p in params),
            mode=mode, trigger_free=free, trigger_fast=None if fast == NONE else fast,
            line_time_ns=line, frame_period_ns=frame, gain_mdb=gain, exposure_us=exposure))
    return links


def encode_outputs(outputs: Sequence[GraphOutput], what: str = "graph") -> bytes:
    """The GRAPH_OUTPUTS payload, `GRAPH_OUTPUTS_MAX` rows at most, each output once."""
    if len(outputs) > NxsMap.GRAPH_OUTPUTS_MAX:
        raise RecordError(f"{what}: {len(outputs)} outputs, the hub has "
                          f"{NxsMap.GRAPH_OUTPUTS_MAX}")
    seen = [int(o.output) for o in outputs]
    if len(set(seen)) != len(seen):
        raise RecordError(f"{what}: an output declared twice")
    rows = []
    for o in outputs:
        where = f"{what} output {o.output}"
        if int(o.mode) not in OutputMode._NAMES:
            raise RecordError(f"{where}: mode {o.mode!r} is not a GraphOutputMode")
        if int(o.consumer) not in Consumer._NAMES:
            raise RecordError(f"{where}: consumer {o.consumer!r} is not a GraphConsumer")
        vc = (_u(o.vc_a, 4, f"{where} link A's channel")
              | (_u(o.vc_b, 4, f"{where} link B's channel") << 4))
        rows.append(_OUTPUT.pack(_u(o.output, 1, f"{where} output"),
                                 _u(o.lanes, 8, f"{where} lanes"),
                                 _u(o.rate_mbps, 16, f"{where} rate"),
                                 _u(o.streams, 4, f"{where} streams"), int(o.mode),
                                 int(o.consumer), vc))
    return b"".join(rows)


def decode_outputs(payload: bytes) -> List[GraphOutput]:
    """The GRAPH_OUTPUTS payload back to its rows."""
    outputs = []
    for raw in _rows(payload, _OUTPUT.size, "GRAPH_OUTPUTS"):
        output, lanes, rate, streams, mode, consumer, vc = _OUTPUT.unpack(raw)
        outputs.append(GraphOutput(output=output, lanes=lanes, rate_mbps=rate, streams=streams,
                                   mode=mode, consumer=consumer, vc_a=vc & 0x0F, vc_b=vc >> 4))
    return outputs


def encode_walks(walks: GraphWalks, what: str = "graph") -> bytes:
    """The GRAPH_WALKS payload: the head, the base staging, then the walks in
    `GraphWalk` order, each with its steps."""
    if len(walks.base) > _PARAMS_MAX:
        raise RecordError(f"{what}: {len(walks.base)} staged parameters, an image declares "
                          f"{_PARAMS_MAX} at most")
    if len(walks.vlock) > 2:
        raise RecordError(f"{what}: {len(walks.vlock)} video-lock registers, the record holds 2")
    if not 1 <= int(walks.train_rounds) <= 0xFF:
        raise RecordError(f"{what}: {walks.train_rounds} training passes, 1 to 255")
    if walks.phase_param is None:
        raise RecordError(f"{what}: no phase parameter, which every HUB step stages")
    vlock = [_u(r, 16, f"{what} video-lock register") for r in walks.vlock]
    vlock += [REG_NONE] * (2 - len(vlock))
    buf = bytearray(_WALKS_HEAD.pack(_index(walks.phase_param, f"{what} phase parameter"),
                                     _index(walks.lock_param, f"{what} lock parameter"),
                                     int(walks.train_rounds), len(walks.base), *vlock))
    for index, value in walks.base:
        buf += _STAGE.pack(_index(index, f"{what} staging"), _u(value, 32, f"{what} staged value"))
    for walk in sorted(int(w) for w in walks.walks):
        if walk not in GraphWalk._NAMES:
            raise RecordError(f"{what}: walk {walk} is not a GraphWalk")
        steps = tuple(walks.walks[walk])
        label = f"{what} walk {GraphWalk._NAMES[walk].lower()}"
        if len(steps) > NxsMap.GRAPH_STEPS_MAX:
            raise RecordError(f"{label}: {len(steps)} steps, a walk holds "
                              f"{NxsMap.GRAPH_STEPS_MAX} at most")
        buf += _WALK_HEAD.pack(walk, len(steps))
        for step in steps:
            buf += _encode_step(step, label)
    return bytes(buf)


def _encode_step(step: GraphStep, what: str) -> bytes:
    if int(step.kind) not in StepKind._NAMES:
        raise RecordError(f"{what}: step kind {step.kind!r} is not a GraphStepKind")
    if int(step.link) != NONE and int(step.link) >= NxsMap.GRAPH_LINKS_MAX:
        raise RecordError(f"{what}: link row {step.link} past the "
                          f"{NxsMap.GRAPH_LINKS_MAX} a graph carries")
    return _STEP.pack(int(step.kind), _u(step.link, 8, f"{what} link"),
                      _u(step.op, 8, f"{what} op"), _u(step.flags, 8, f"{what} flags"),
                      _u(step.value, 32, f"{what} value"))


def decode_walks(payload: bytes) -> GraphWalks:
    """The GRAPH_WALKS payload back."""
    if len(payload) < _WALKS_HEAD.size:
        raise RecordError("GRAPH_WALKS record truncated inside its head")
    phase, lock, rounds, count, vlock_a, vlock_b = _WALKS_HEAD.unpack_from(payload)
    pos = _WALKS_HEAD.size
    if pos + count * _STAGE.size > len(payload):
        raise RecordError("GRAPH_WALKS record truncated inside its staging")
    base = tuple(_STAGE.unpack_from(payload, pos + i * _STAGE.size) for i in range(count))
    pos += count * _STAGE.size
    walks: Dict[int, Tuple[GraphStep, ...]] = {}
    while pos < len(payload):
        if pos + _WALK_HEAD.size > len(payload):
            raise RecordError("GRAPH_WALKS record truncated inside a walk's head")
        walk, steps = _WALK_HEAD.unpack_from(payload, pos)
        pos += _WALK_HEAD.size
        if walk in walks:
            raise RecordError(f"GRAPH_WALKS names walk {walk} twice")
        if steps > NxsMap.GRAPH_STEPS_MAX or pos + steps * _STEP.size > len(payload):
            raise RecordError(f"GRAPH_WALKS walk {walk} counts {steps} steps past the record")
        walks[walk] = tuple(GraphStep(*_STEP.unpack_from(payload, pos + i * _STEP.size))
                            for i in range(steps))
        pos += steps * _STEP.size
    return GraphWalks(phase_param=phase, walks=walks, base=base,
                      lock_param=None if lock == NONE else lock, train_rounds=rounds,
                      vlock=tuple(r for r in (vlock_a, vlock_b) if r != REG_NONE))


def encode_phases(phases: Mapping[int, str], what: str = "hub") -> bytes:
    """The HUB_PHASES payload: each phase and its name, by phase,
    `HUB_PHASES_MAX` at most, each name once."""
    if len(phases) > NxsMap.HUB_PHASES_MAX:
        raise RecordError(f"{what}: {len(phases)} phases, a hub program names "
                          f"{NxsMap.HUB_PHASES_MAX} at most")
    names = [str(n) for n in phases.values()]
    twice = sorted({n for n in names if names.count(n) > 1})
    if twice:
        raise RecordError(f"{what}: phase {', '.join(twice)} named twice")
    return b"".join(_PHASE.pack(_u(phase, 8, f"{what} phase"), _name(name, f"{what} phase {phase}"))
                    for phase, name in sorted((int(p), n) for p, n in phases.items()))


def decode_phases(payload: bytes) -> Dict[int, str]:
    """The HUB_PHASES payload back: each phase's name by its value."""
    phases: Dict[int, str] = {}
    for raw in _rows(payload, _PHASE.size, "HUB_PHASES"):
        phase, name = _PHASE.unpack(raw)
        phases[phase] = _text(name, "HUB_PHASES name")
    return phases


# ── the composition ──────────────────────────────────────────────────

def graph_trailer(program, records: Sequence[Tuple[int, bytes]],
                  name: str) -> List[Tuple[int, bytes]]:
    """The trailer of a graph image named `name` that composes `records`
    against `program`, a hub image read back (`CompiledDriver`): the
    program's IDENTITY head, the GRAPH records in record order, then the
    program's HUB_PHASES. A source that
    names no hub address (0) is a port with no hub: its links name no
    serializer or expander and its walks run no hub phase. RecordError
    names what refuses: a record that is no GRAPH record or comes twice, a
    missing GRAPH_SOURCE, GRAPH_LINKS or GRAPH_WALKS, a source that names
    another program, a program that names no phases, a serializer or an
    expander the program does not reach, and a step or a staged value the
    program or the link cannot take."""
    by_type: Dict[int, bytes] = {}
    for rec_type, payload in records:
        rec_type = int(rec_type)
        if rec_type not in GRAPH_RECORDS:
            raise RecordError(
                f"{name}: record {TrailerRecord._NAMES.get(rec_type, rec_type)} is no GRAPH "
                f"record; a graph image carries the GRAPH records and its program's HUB_PHASES")
        if rec_type in by_type:
            raise RecordError(f"{name}: two {TrailerRecord._NAMES[rec_type]} records")
        by_type[rec_type] = bytes(payload)
    for required in (GRAPH_SOURCE, GRAPH_LINKS, GRAPH_WALKS):
        if required not in by_type:
            raise RecordError(f"{name}: no {TrailerRecord._NAMES[required]} record")
    source = decode_source(by_type[GRAPH_SOURCE])
    crc = program_crc(program.seal_nonce, program.bytecode)
    if source.program != program.name or source.program_crc != crc:
        raise RecordError(
            f"{name}: the records compose against {source.program} {source.program_crc:#010x}, "
            f"and the hub image is {program.name} {crc:#010x}")
    phases_payload = next((bytes(p) for t, p in program.trailer if int(t) == HUB_PHASES), None)
    if phases_payload is None:
        raise RecordError(f"{name}: {program.name} names no phases, and a graph's walks name "
                          f"the phases of its program (HubDevice.PHASES)")
    phases = decode_phases(phases_payload)
    links = decode_links(by_type[GRAPH_LINKS])
    direct = source.des_addr == 0
    for link in links:
        _check_companions(link, name, direct)
    if GRAPH_OUTPUTS in by_type:
        decode_outputs(by_type[GRAPH_OUTPUTS])
    walks = decode_walks(by_type[GRAPH_WALKS])
    _check_walks(program, phases, links, walks, name, direct)
    identity = [(t, bytes(p)) for t, p in program.trailer if int(t) == TrailerRecord.IDENTITY]
    trailer = [(t, by_type[t]) for t in GRAPH_RECORDS if t in by_type]
    return identity[:1] + trailer + [(HUB_PHASES, phases_payload)]


def _check_companions(link: GraphLink, name: str, direct: bool) -> None:
    for what, declared, fact in (("serializer", link.ser_addr, SER_ADDR),
                                 ("expander", link.tca_addr, TCA_ADDR)):
        if direct and int(declared) != 0:
            raise RecordError(
                f"{name}: link {link.name} declares a {what} at {int(declared):#04x}, and a port "
                f"with no hub carries none")
        if not direct and int(declared) != fact:
            raise RecordError(
                f"{name}: link {link.name} declares its {what} at {int(declared):#04x}, and the "
                f"Hub's program reaches it at {fact:#04x}, a board fact it is compiled with")


def _accepts(param, value: int) -> bool:
    values = [int(v) for v in param.values]
    if param.param_type == "range":
        return values[0] <= int(value) <= values[1]
    return int(value) in values


def _program_value(program, index: int, value: int, what: str) -> None:
    if index >= len(program.params):
        raise RecordError(f"{what}: parameter {index} past the program's "
                          f"{len(program.params)}")
    param = program.params[index]
    if not _accepts(param, value):
        raise RecordError(f"{what}: {param.name} takes {list(param.values)}, not {value}")


def _check_walks(program, phases: Mapping[int, str], links: Sequence[GraphLink],
                 walks: GraphWalks, name: str, direct: bool = False) -> None:
    if walks.phase_param >= len(program.params):
        raise RecordError(f"{name}: phase parameter {walks.phase_param} past the program's "
                          f"{len(program.params)}")
    if walks.lock_param is not None and walks.lock_param >= len(program.params):
        raise RecordError(f"{name}: lock parameter {walks.lock_param} past the program's "
                          f"{len(program.params)}")
    for index, value in walks.base:
        _program_value(program, index, value, f"{name} staging")
    for walk, steps in walks.walks.items():
        label = f"{name} walk {GraphWalk._NAMES.get(walk, walk).lower()}"
        for i, step in enumerate(steps):
            if direct and int(step.kind) == StepKind.HUB:
                raise RecordError(f"{label} step {i}: a port with no hub runs no hub phase")
            _check_step(program, phases, links, step, f"{label} step {i}")


def _check_step(program, phases: Mapping[int, str], links: Sequence[GraphLink], step: GraphStep,
                where: str) -> None:
    kind = int(step.kind)
    if kind == StepKind.HUB:
        if step.op not in phases:
            raise RecordError(f"{where}: phase {step.op} is none the program names "
                              f"({', '.join(f'{p} {n}' for p, n in sorted(phases.items()))})")
        if step.overrides:
            _program_value(program, step.index, step.value, where)
        return
    if kind == StepKind.HOST:
        if int(step.op) not in HostOp._NAMES:
            raise RecordError(f"{where}: host operation {step.op} is not a GraphHostOp")
        if step.link != NONE and step.link >= len(links):
            raise RecordError(f"{where}: link row {step.link} past the {len(links)} links")
        return
    if step.link >= len(links):
        raise RecordError(f"{where}: link row {step.link} past the {len(links)} links")
    link = links[step.link]
    if kind == StepKind.POD:
        if step.op >= len(ACTIONS):
            raise RecordError(f"{where}: action {step.op} is not a CamAction")
        if not link.pod and not link.head_image:
            raise RecordError(f"{where}: link {link.name} has no pod and no head image to run")
        if link.param(GraphParam.MODE) is None:
            raise RecordError(f"{where}: link {link.name}'s personality carries no mode parameter")
        if step.op != CONFIGURE and link.param(GraphParam.ACTION) is None:
            raise RecordError(f"{where}: link {link.name}'s personality declares no action "
                              f"parameter; it runs the whole program only")
    elif kind == StepKind.MAP:
        if not link.pod_map:
            raise RecordError(f"{where}: link {link.name} has no camera door: no pod and no "
                              f"smart camera")
        if step.index not in ParamKind._NAMES:
            raise RecordError(f"{where}: commit kind {step.index} is not a ParamKind")
    elif kind == StepKind.CONVERGE:
        if not link.pod or not link.personality:
            raise RecordError(f"{where}: link {link.name} has no pod personality to converge")


# ── the printer ──────────────────────────────────────────────────────

def _flag_words(pairs) -> str:
    return " ".join(word for on, word in pairs if on)


def step_text(step: GraphStep, links: Sequence[GraphLink] = (),
              phases: Optional[Mapping[int, str]] = None) -> str:
    """One step as `nxs personality records` prints it."""
    kind = int(step.kind)
    link = (links[step.link].name if step.link < len(links)
            else (f"row {step.link}" if step.link != NONE else ""))
    if kind == StepKind.HUB:
        named = (phases or {}).get(step.op)
        text = f"hub {step.op}" + (f" {named}" if named else "")
        if step.overrides:
            text += f" (+{step.index}={step.value})"
    elif kind == StepKind.POD:
        action = ACTIONS[step.op] if step.op < len(ACTIONS) else str(step.op)
        text = f"pod {link} {action}"
        if step.overrides:
            text += f" (+{step.index}={step.value})"
    elif kind == StepKind.MAP:
        text = (f"map {link} row {step.op} = {step.value} "
                f"({ParamKind._NAMES.get(step.index, step.index).lower()})")
    elif kind == StepKind.HOST:
        op = HostOp._NAMES.get(step.op, step.op)
        text = f"host {str(op).lower()}" + (f" {link}" if link else "")
    else:
        text = f"converge {link}"
    flags = _flag_words(((step.soft, "soft"), (step.again, "again")))
    return f"{text}{' ' + flags if flags else ''}"


def record_lines(rec_type: int, payload: bytes, trailer: Sequence[Tuple[int, bytes]] = ()
                 ) -> Optional[List[str]]:
    """The lines `nxs personality records` prints under a GRAPH or
    HUB_PHASES record, None for another record. `trailer` lends a walk's
    steps the link names and the phase names. RecordError for a record
    this codec cannot read."""
    rec_type = int(rec_type)
    by_type = {int(t): bytes(p) for t, p in trailer}
    if rec_type == GRAPH_SOURCE:
        s = decode_source(payload)
        flags = _flag_words(((s.homogeneous, "homogeneous"), (s.sync, "sync"), (s.gain, "gain")))
        return [f"program {s.program} {s.program_crc:#010x}, declaration {s.digest.hex()}, "
                f"hub @{s.des_addr:#04x}, owner {Owner._NAMES.get(s.owner, s.owner).lower()}"
                + (f", {flags}" if flags else "")]
    if rec_type == GRAPH_LINKS:
        return [f"{i} {_link_text(link)}" for i, link in enumerate(decode_links(payload))]
    if rec_type == GRAPH_OUTPUTS:
        return [f"{i} {_output_text(o)}" for i, o in enumerate(decode_outputs(payload))]
    if rec_type == GRAPH_WALKS:
        walks = decode_walks(payload)
        links = decode_links(by_type[GRAPH_LINKS]) if GRAPH_LINKS in by_type else []
        phases = decode_phases(by_type[HUB_PHASES]) if HUB_PHASES in by_type else {}
        head = (f"phase parameter {walks.phase_param}, lock parameter "
                f"{'none' if walks.lock_param is None else walks.lock_param}, "
                f"{walks.train_rounds} training passes, video lock "
                + (" ".join(f"{r:#06x}" for r in walks.vlock) or "none"))
        lines = [head, "staged " + (", ".join(f"{i}={v}" for i, v in walks.base) or "nothing")]
        for walk, steps in sorted(walks.walks.items()):
            lines.append(f"{GraphWalk._NAMES.get(walk, walk).lower()}: "
                         + (", ".join(step_text(s, links, phases) for s in steps) or "no step"))
        return lines
    if rec_type == HUB_PHASES:
        return [f"{phase} {name}" for phase, name in sorted(decode_phases(payload).items())]
    return None


def _link_text(link: GraphLink) -> str:
    flags = _flag_words(((link.present, "present"), (link.camera, "camera"),
                         (link.pod, "pod"), (link.host_csi, "host-csi"),
                         (link.frame_fixed, "frame-fixed")))
    mapped = " ".join(f"{a:#04x}->{t:#04x}" for a, t in link.entries) or "none"
    text = (f"{link.name} {flags}: serializer {link.ser_addr:#04x}, head {link.head_addr:#04x}, "
            f"expander {link.tca_addr:#04x}, {link.lanes} lanes, "
            f"{'RAW12' if link.data_type else 'RAW10'}, map {mapped}")
    if link.pod:
        text += (f", pod {link.pod_alias:#04x} (unit {link.pod_target:#04x}, camera door "
                 f"{link.pod_map:#04x}) {link.personality} {link.personality_crc:#010x}")
    if link.head_image:
        text += f", head image {link.head_image} {link.head_image_crc:#010x}"
    params = " ".join(f"{GraphParam._NAMES[i].lower()} {p}" for i, p in enumerate(link.params)
                      if p is not None)
    fast = "none" if link.trigger_fast is None else link.trigger_fast
    return (f"{text}, mode {link.mode}, trigger free {link.trigger_free} fast {fast}, "
            f"line {link.line_time_ns} ns, frame {link.frame_period_ns} ns, gain "
            f"{link.gain_mdb} mdB, exposure {link.exposure_us} us, params {params or 'none'}")


def _output_text(output: GraphOutput) -> str:
    streams = " ".join(word for bit, word in (
        (NxsMap.GRAPH_STREAM_Y_TO_A, "Y->A"), (NxsMap.GRAPH_STREAM_Z_TO_A, "Z->A"),
        (NxsMap.GRAPH_STREAM_Y_TO_B, "Y->B"), (NxsMap.GRAPH_STREAM_Z_TO_B, "Z->B"))
        if output.streams & bit) or "none"
    return (f"csi.{'AB'[output.output] if output.output < 2 else output.output} "
            f"{output.lanes} lanes {output.rate_mbps} Mbps, streams {streams}, "
            f"{OutputMode._NAMES.get(output.mode, output.mode).lower()}, "
            f"{Consumer._NAMES.get(output.consumer, output.consumer).lower()}, "
            f"vc A {output.vc_a} B {output.vc_b}")
