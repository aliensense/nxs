"""NXS personality-image binary format: serializer and deserializer. Layout:
13-byte header (magic, major, minor, kind, flags, name_len, num_params,
num_outputs, probe_len), name, bytecode section (plain, or nonce + ciphertext
when sealed), probe block, params, output fields, bus_config trailer,
descriptor trailer. Integers little-endian."""

import struct
from typing import List, Optional, Tuple

from nxs.compiler import (CompiledDriver, CompileError, ParamDescriptor,
                          PatchEntry, field_width, resolve_field_offsets)
from nxs.dsl.compiled import HOST_KINDS
from nxs.dsl.fields import FIELD_TYPES
from nxs._generated_constants import NxsMap, NxsDriverImage
from nxs.opcodes import Op, INSTRUCTION_SIZE

_ParamKind = NxsDriverImage.ParamKind
#: A parameter's kind by the name the DSL gives it: `ParamKind` in lower case.
_PARAM_KINDS = {name.lower(): number for number, name in _ParamKind._NAMES.items()}
_BusKind = NxsDriverImage.BusKind
ImageKind = NxsDriverImage.ImageKind
TrailerRecord = NxsDriverImage.TrailerRecord

NXS_MAGIC = b"NXS\x00"
NXS_MAJOR = NxsDriverImage.NXS_MAJOR
NXS_MINOR = NxsDriverImage.NXS_MINOR
NXS_MINOR_FIRST = NxsDriverImage.NXS_MINOR_FIRST

# Header: magic, major, format minor, kind, flags, name_len, num_params,
# num_outputs, probe_len (bytes of the probe program, the offset where
# configure begins).
_HEADER = struct.Struct("<4sBBBBBBBH")
HEADER_SIZE = _HEADER.size
#: The 9-byte header of a format 1.0 image (`NXS_MAJOR` below `NXS_MINOR_FIRST`), with no kind,
#: flags or probe length: refused as another major's, read only to name the rebuild.
V1_0_HEADER_SIZE = 9

IMAGE_FLAG_SEALED = NxsDriverImage.IMAGE_FLAG_SEALED
IMAGE_FLAG_AUTO = NxsDriverImage.IMAGE_FLAG_AUTO
SEAL_NONCE_SIZE = NxsDriverImage.SEAL_NONCE_SIZE
MAX_TRAILER_SIZE = NxsDriverImage.MAX_TRAILER_SIZE
_TRAILER_RECORD_HEADER = struct.Struct("<BH")

IMAGE_KIND_NAMES = {ImageKind.CLICK: 'click', ImageKind.CAMERA: 'cam',
                    ImageKind.HUB: 'hub', ImageKind.MODULE: 'module',
                    ImageKind.GRAPH: 'graph', ImageKind.HOST: 'host'}

# The hint every format refusal carries; a caller that knows the
# personality substitutes its name.
REBUILD_HINT = "rebuild it: nxs upload <name>"

# Field type codes; firmware stores and forwards them verbatim, so every decoder
# must map through FIELD_TYPE_NAMES.
_FIELD_TYPE_MAP = {name: code for code, name in enumerate(FIELD_TYPES)}
FIELD_TYPE_NAMES = dict(enumerate(FIELD_TYPES))

# Bus-kind encoding for the optional bus_config trailer.
BUS_KIND_I2C  = _BusKind.I2C
BUS_KIND_SPI  = _BusKind.SPI
BUS_KIND_UART = _BusKind.UART
MAX_BUS_PROFILES = NxsDriverImage.MAX_BUS_PROFILES
MAX_OUTPUTS = NxsDriverImage.MAX_OUTPUTS
MAX_PARAMS = NxsDriverImage.MAX_PARAMS
MAX_PARAM_VALUES = NxsDriverImage.MAX_PARAM_VALUES
MAX_PATCH_SITES = NxsDriverImage.MAX_PATCH_SITES
VM_MAX_PROGRAM_SIZE = NxsDriverImage.VM_MAX_PROGRAM_SIZE
VM_HOST_PROGRAM_SIZE = NxsDriverImage.VM_HOST_PROGRAM_SIZE
MAX_DRIVER_IMAGE_SIZE = NxsDriverImage.MAX_DRIVER_IMAGE_SIZE

_BUS_KIND_NAMES = {
    BUS_KIND_I2C:  'i2c',
    BUS_KIND_SPI:  'spi',
    BUS_KIND_UART: 'uart',
}
_BUS_KIND_NAME_TO_INT = {v: k for k, v in _BUS_KIND_NAMES.items()}

# Register-access switch codes shared with the firmware.
_AutoInc = NxsDriverImage.AutoInc
_Pec = NxsDriverImage.Pec
_AUTO_INC_TO_INT = {'implicit': _AutoInc.IMPLICIT, 'msb': _AutoInc.MSB,
                    'none': _AutoInc.NONE}
_AUTO_INC_NAMES = {v: k for k, v in _AUTO_INC_TO_INT.items()}
_PEC_TO_INT = {'none': _Pec.NONE, 'crc8': _Pec.CRC8}
_PEC_NAMES = {v: k for k, v in _PEC_TO_INT.items()}
_BYTE_ORDER_TO_INT = {'big': 0, 'little': 1}
_BYTE_ORDER_NAMES = {v: k for k, v in _BYTE_ORDER_TO_INT.items()}


def _u32(value, what: str) -> int:
    """`value` as an unsigned 32-bit cell, CompileError past one."""
    v = int(value)
    if not 0 <= v <= 0xFFFFFFFF:
        raise CompileError(f"{what} {v} does not fit the image's unsigned 32-bit cell")
    return v


def _code_name(names: dict, code: int, what: str) -> str:
    """Map a wire switch code to its name, failing on an unknown value so a
    corrupt image can't silently decode to a default the firmware rejects."""
    if code not in names:
        raise ValueError(f"invalid bus_config {what} code: 0x{code:02X}")
    return names[code]


def _code_int(mapping: dict, name: str, what: str) -> int:
    """Map a switch name to its wire code, failing on an unknown name with a
    clear message rather than a bare KeyError."""
    if name not in mapping:
        raise ValueError(f"invalid bus_config {what}: {name!r}")
    return mapping[name]


#: Characters a name keeps on the unit: its 16-byte field holds the NUL
#: (the firmware's `MAX_PARAM_NAME` and `MAX_DRIVER_NAME`).
NAME_CHARS_MAX = 15
#: Characters an output field's unit keeps: its row's `UNIT` field in the
#: OUTPUTS record, which the map serves.
UNIT_CHARS_MAX = NxsMap.ROW_UNIT_LEN
#: Characters a parameter's unit keeps: its row's `UNIT` field, which both
#: maps serve.
PARAM_UNIT_CHARS_MAX = NxsMap.ROW_UNIT_LEN


def _text(value: str, limit: int, what: str) -> bytes:
    """`value` as ASCII, refused past `limit` characters: the unit keeps no
    more, and a name it clamped would match nothing by name."""
    try:
        raw = str(value).encode("ascii")
    except UnicodeEncodeError:
        raise CompileError(f"{what} {value!r} is not ASCII") from None
    if len(raw) > limit:
        raise CompileError(f"{what} {value!r} takes at most {limit} characters, "
                           f"the unit keeps no more")
    return raw


def serialize(compiled: CompiledDriver) -> bytes:
    """Serialize a CompiledDriver into the NXS binary format. Raises CompileError
    when a descriptor cap (`MAX_PARAMS`, `MAX_PARAM_VALUES`, `MAX_PATCH_SITES`,
    `MAX_TRAILER_SIZE`) or the program / image size limit is exceeded. A sealed
    section passes through as it was read: this tool never encrypts."""
    if len(compiled.params) > MAX_PARAMS:
        raise CompileError(
            f"{compiled.name}: {len(compiled.params)} params exceeds the "
            f"{MAX_PARAMS}-param image limit")
    # A hub or graph image runs on the host executor and takes its program
    # budget; a pod's slot holds the rest.
    host = compiled.kind in HOST_KINDS
    program_cap = VM_HOST_PROGRAM_SIZE if host else VM_MAX_PROGRAM_SIZE
    if len(compiled.bytecode) > program_cap:
        raise CompileError(
            f"{compiled.name}: {len(compiled.bytecode)} B of bytecode "
            f"exceeds the {program_cap} B VM program limit")
    if compiled.kind not in IMAGE_KIND_NAMES:
        raise ValueError(f"{compiled.name}: unknown image kind {compiled.kind!r}")

    buf = bytearray()
    name_bytes = _text(compiled.name, NAME_CHARS_MAX, "the personality name")

    # The header's minor is the format minor of the wheel that built the
    # image; an image read back keeps the one it carries.
    required_minor = (NXS_MINOR if compiled.required_minor is None
                      else int(compiled.required_minor))
    if compiled.sealed and len(compiled.seal_nonce) != SEAL_NONCE_SIZE:
        raise ValueError(
            f"{compiled.name}: a sealed image carries a "
            f"{SEAL_NONCE_SIZE}-byte nonce, got {len(compiled.seal_nonce)}")

    probe_len = int(compiled.probe_len)
    if not 0 <= probe_len <= len(compiled.bytecode):
        raise CompileError(
            f"{compiled.name}: probe length {probe_len} lies outside the "
            f"{len(compiled.bytecode)} B program")

    buf += _HEADER.pack(NXS_MAGIC, NXS_MAJOR, required_minor,
                        int(compiled.kind), int(compiled.flags) & 0xFF,
                        len(name_bytes), len(compiled.params),
                        len(compiled.output_fields), probe_len)

    # Personality name
    buf += name_bytes

    # Bytecode section: the nonce precedes the length only when sealed.
    if compiled.sealed:
        buf += compiled.seal_nonce
    buf += struct.pack("<H", len(compiled.bytecode))
    buf += compiled.bytecode

    # Probe block: WHO_AM_I anchor + candidate I²C addresses; empty for a
    # stream personality.
    wai_reg = getattr(compiled, 'who_am_i_reg', 0) or 0
    wai_values = list(getattr(compiled, 'who_am_i_values', []) or [])[:16]
    i2c_addrs = list(getattr(compiled, 'i2c_addrs', []) or [])[:8]
    buf += struct.pack("<BB", wai_reg, len(wai_values))
    buf += bytes(v & 0xFF for v in wai_values)
    buf += struct.pack("<B", len(i2c_addrs))
    buf += bytes(a & 0x7F for a in i2c_addrs)

    # Parameter descriptors
    for param in compiled.params:
        _write_param(buf, param, compiled.patch_map)

    # Output fields. A `scale_param` name resolves to its param index here;
    # the firmware multiplies the base scale by that param's live value.
    param_index = {p.name: i for i, p in enumerate(compiled.params)}
    for field in resolve_field_offsets(compiled.output_fields):
        _write_output_field(buf, field, param_index)

    # The bus_config trailer's count byte is always present (0 = none, the
    # firmware keeps DTS defaults), so the descriptor trailer after it has a
    # fixed start.
    _write_bus_config(buf, getattr(compiled, 'bus_config', None) or [])

    # Descriptor trailer: host-owned records the firmware stores opaquely.
    buf += trailer_bytes(compiled.trailer, compiled.name)

    # The cap is the aggregate header + metadata + bytecode (store slots fill
    # the NVS partition exactly).
    if not host and len(buf) > MAX_DRIVER_IMAGE_SIZE:
        raise CompileError(
            f"{compiled.name}: {len(buf)} B image exceeds the "
            f"{MAX_DRIVER_IMAGE_SIZE} B image limit "
            f"({len(compiled.bytecode)} B bytecode + "
            f"{len(buf) - len(compiled.bytecode)} B header/metadata)")

    return bytes(buf)


def compose_graph(hub_asset: bytes, records, name: str) -> bytes:
    """A graph image named `name`: the hub asset's program under the kind
    GRAPH, its section as the asset stores it (a sealed one's nonce and
    ciphertext byte for byte), its parameters and its bus profile, and a
    trailer of the program's IDENTITY head, the GRAPH `records` and the
    program's HUB_PHASES. Nothing is sealed here, so a released hub program
    stays sealed under the release key. CompileError names a refusal: an
    asset that is no hub image, or records the program cannot run
    (`records_graph.graph_trailer`)."""
    from nxs.personality.records_fields import RecordError
    from nxs.personality.records_graph import graph_trailer

    program = deserialize(hub_asset)
    if program.kind != ImageKind.HUB:
        raise CompileError(f"{name}: a graph composes against a hub image, and {program.name} "
                           f"is a {IMAGE_KIND_NAMES[program.kind]} image")
    try:
        trailer = graph_trailer(program, records, name)
    except RecordError as exc:
        raise CompileError(str(exc)) from None
    program.kind = ImageKind.GRAPH
    program.name = name
    program.trailer = trailer
    return serialize(program)


def trailer_bytes(records, name: str = "image") -> bytes:
    """The descriptor trailer on the wire: a count byte, then one
    `type u8, len u16, bytes` record per `(type, bytes)` entry. Raises
    CompileError past `MAX_TRAILER_SIZE` (count byte included)."""
    records = list(records or [])
    if len(records) > 0xFF:
        raise CompileError(
            f"{name}: {len(records)} trailer records exceed the 255-record "
            f"count byte")
    buf = bytearray(struct.pack("<B", len(records)))
    for rec_type, payload in records:
        rec_type = int(rec_type)
        payload = bytes(payload)
        if not 1 <= rec_type <= 0xFF:
            raise ValueError(
                f"{name}: trailer record type {rec_type} is outside 1..255 "
                f"(0 is reserved)")
        if len(payload) > 0xFFFF:
            raise CompileError(
                f"{name}: trailer record type {rec_type} carries "
                f"{len(payload)} B, past the 16-bit length")
        buf += _TRAILER_RECORD_HEADER.pack(rec_type, len(payload))
        buf += payload
    if len(buf) > MAX_TRAILER_SIZE:
        raise CompileError(
            f"{name}: {len(buf)} B descriptor trailer exceeds the "
            f"{MAX_TRAILER_SIZE} B limit (count byte included)")
    return bytes(buf)


def trailer_size(data: bytes) -> Optional[int]:
    """Total byte count of the descriptor trailer whose first bytes are
    `data`, or None while `data` ends inside a record header or count byte.
    A prefix that already reads past `MAX_TRAILER_SIZE` raises ValueError."""
    if not data:
        return None
    count = data[0]
    pos = 1
    for _ in range(count):
        if pos + _TRAILER_RECORD_HEADER.size > len(data):
            return None
        _rec_type, length = _TRAILER_RECORD_HEADER.unpack_from(data, pos)
        pos += _TRAILER_RECORD_HEADER.size + length
        if pos > MAX_TRAILER_SIZE:
            raise ValueError(
                f"descriptor trailer declares {pos}+ B, past the "
                f"{MAX_TRAILER_SIZE} B limit")
    return pos


def parse_trailer(data: bytes) -> List[Tuple[int, bytes]]:
    """The `(type, bytes)` records of a complete descriptor trailer (the form
    `trailer_bytes` writes and the unit serves). Raises ValueError on a
    truncated trailer."""
    size = trailer_size(data)
    if size is None or size > len(data):
        raise ValueError("descriptor trailer truncated")
    records = []
    pos = 1
    for _ in range(data[0]):
        rec_type, length = _TRAILER_RECORD_HEADER.unpack_from(data, pos)
        pos += _TRAILER_RECORD_HEADER.size
        records.append((rec_type, bytes(data[pos:pos + length])))
        pos += length
    return records


def format_refusal(major: int, minor: int) -> Optional[str]:
    """The one-line reason this tool refuses an image of format
    `major.minor`, or None when it reads that format: its own major, from
    its first minor (`NXS_MINOR_FIRST`) to its own."""
    if major == NXS_MAJOR and NXS_MINOR_FIRST <= minor <= NXS_MINOR:
        return None
    layout = (f" (the v{major}.{minor} image layout)"
              if major == NXS_MAJOR and minor < NXS_MINOR_FIRST else "")
    return (f"image format {major}.{minor}{layout}, this tool builds "
            f"{NXS_MAJOR}.{NXS_MINOR} — {REBUILD_HINT}")


def peek_format(data: bytes) -> tuple:
    """`(major, minor, kind, flags)` from an NXS header. A v1.0 header
    answers kind CLICK and no flags, the only values that layout could
    carry; a truncated artifact is rejected here."""
    if (len(data) >= V1_0_HEADER_SIZE and data[:4] == NXS_MAGIC
            and data[4] == NXS_MAJOR and data[5] < NXS_MINOR_FIRST):
        return data[4], data[5], ImageKind.CLICK, 0
    if len(data) < HEADER_SIZE or data[:4] != NXS_MAGIC:
        raise ValueError(f"Not an NXS image header: {data[:HEADER_SIZE]!r}")
    return data[4], data[5], data[6], data[7]


def deserialize(data: bytes) -> CompiledDriver:
    """Deserialize an NXS binary image into a CompiledDriver. A sealed
    bytecode section stays opaque: `bytecode` holds the ciphertext and
    `sealed` is True. An image cut short anywhere is a ValueError."""
    try:
        return _deserialize(data)
    except (struct.error, IndexError) as exc:
        raise ValueError(f"image truncated ({exc})") from None


def _deserialize(data: bytes) -> CompiledDriver:
    pos = 0

    # Header
    if data[:4] != NXS_MAGIC:
        raise ValueError(f"Bad magic: {data[:4]!r}")
    # The v1.0 layout shares this major and has a header of its own.
    if len(data) >= 6 and data[4] == NXS_MAJOR and data[5] < NXS_MINOR_FIRST:
        raise ValueError(format_refusal(data[4], data[5]))
    if len(data) < HEADER_SIZE:
        raise ValueError("image header truncated")
    (_magic, major, minor, kind, flags, name_len, num_params,
     num_outputs, probe_len) = _HEADER.unpack_from(data, pos)
    pos += HEADER_SIZE
    # Major gates the wire layout. Minor, the building wheel's format minor, gates the load on the
    # firmware alone, so any same-major image is inspectable.
    if major != NXS_MAJOR:
        raise ValueError(format_refusal(major, minor))
    if kind not in IMAGE_KIND_NAMES:
        raise ValueError(f"image declares unknown kind {kind}")
    if num_outputs > MAX_OUTPUTS:
        raise ValueError(
            f"image declares {num_outputs} output fields (max {MAX_OUTPUTS})")

    # Name
    name = data[pos:pos + name_len].decode('ascii')
    pos += name_len

    # Bytecode: plain, or the CTR nonce then the ciphertext when sealed.
    seal_nonce = b""
    if flags & IMAGE_FLAG_SEALED:
        seal_nonce = bytes(data[pos:pos + SEAL_NONCE_SIZE])
        pos += SEAL_NONCE_SIZE
        if len(seal_nonce) != SEAL_NONCE_SIZE:
            raise ValueError("sealed image truncated inside its nonce")
    bytecode_len = struct.unpack_from("<H", data, pos)[0]
    pos += 2
    bytecode = data[pos:pos + bytecode_len]
    pos += bytecode_len
    if probe_len > bytecode_len:
        raise ValueError(
            f"image declares a {probe_len} B probe in a {bytecode_len} B program")

    # Probe block
    wai_reg = data[pos]; pos += 1
    num_wai = data[pos]; pos += 1
    who_am_i_values = list(data[pos:pos + num_wai]); pos += num_wai
    num_addrs = data[pos]; pos += 1
    i2c_addrs = list(data[pos:pos + num_addrs]); pos += num_addrs

    # Parameters
    params = []
    patch_map = []
    for _ in range(num_params):
        param, patches, pos = _read_param(data, pos)
        params.append(param)
        patch_map.extend(patches)

    # Output fields
    output_fields = []
    for _ in range(num_outputs):
        field, pos = _read_output_field(data, pos)
        output_fields.append(field)

    # bus_config trailer (count byte always present), then the descriptor
    # trailer.
    bus_config, pos = _read_bus_config(data, pos)
    trailer, pos = _read_trailer(data, pos)

    cd = CompiledDriver(
        bytecode=bytecode,
        # Fields carry explicit byte offsets (gaps are legal), so the
        # sample extent is the furthest field end, not a width sum.
        sample_size=max(
            (int(f['byte_off']) + field_width(f) for f in output_fields),
            default=0,
        ),
        name=name,
        config={p.name: p.current for p in params},
        output_fields=output_fields,
        params=params,
        patch_map=patch_map,
        kind=kind,
        flags=flags,
        seal_nonce=seal_nonce,
        required_minor=minor,
        trailer=trailer,
        probe_len=probe_len,
    )
    # Probe fields are set as attributes rather than ctor args.
    cd.who_am_i_reg = wai_reg
    cd.who_am_i_values = who_am_i_values
    cd.i2c_addrs = i2c_addrs
    cd.bus_config = bus_config
    return cd


def _read_trailer(data: bytes, pos: int):
    """Read the descriptor trailer at `pos`. Returns `(records, new_pos)`;
    a truncated trailer raises ValueError."""
    if pos >= len(data):
        raise ValueError("descriptor trailer missing (image truncated)")
    size = trailer_size(data[pos:])
    if size is None or pos + size > len(data):
        raise ValueError("descriptor trailer truncated")
    return parse_trailer(data[pos:pos + size]), pos + size


def _write_param(buf: bytearray, param: ParamDescriptor,
                 patch_map: List[PatchEntry]):
    """Write a parameter descriptor."""
    name_bytes = _text(param.name, NAME_CHARS_MAX, "parameter")
    buf += struct.pack("<B", len(name_bytes))
    buf += name_bytes

    if param.param_type == "enum":
        param_type = 0
    elif param.param_type == "range":
        param_type = 1
    else:
        raise ValueError(f"unknown param_type: {param.param_type!r}")
    buf += struct.pack("<B", param_type)
    kind = _PARAM_KINDS.get(param.kind, _ParamKind.RUN)
    buf += struct.pack("<B", kind)
    buf += struct.pack("<II", int(param.default), int(param.current))

    # This param's patch sites, ordered by offset for a deterministic wire
    # layout. Apply params carry no patch: the value lives in current_value.
    sites = ([] if param.kind == "apply"
             else sorted((p for p in patch_map if p.param_name == param.name),
                         key=lambda p: p.offset))
    if len(sites) > MAX_PATCH_SITES:
        raise CompileError(
            f"param {param.name!r}: {len(sites)} patch sites exceeds the "
            f"{MAX_PATCH_SITES}-site image limit")

    # Minimum bytes to hold every value in the map.
    def _bytes_needed(value_map):
        widest = max((int(v) for v in value_map.values()), default=0)
        if widest <= 0xFF:
            return 1
        if widest <= 0xFFFF:
            return 2
        return 4

    for patch in sites:
        # Patch operands are unsigned and at most 4 bytes; an out-of-range value
        # would otherwise crash in struct.pack.
        for v in patch.value_map.values():
            if not 0 <= int(v) <= 0xFFFFFFFF:
                raise ValueError(
                    f"param {param.name!r}: patch value {v} is outside the "
                    f"uint32 range")
        # The patch width is declared at the write site; the firmware writes
        # exactly that many bytes, so every value must fit.
        if patch.size not in (1, 2, 4):
            raise ValueError(
                f"param {param.name!r}: patch size {patch.size} must be 1, 2, or 4")
        if _bytes_needed(patch.value_map) > patch.size:
            raise ValueError(
                f"param {param.name!r}: a patched value needs "
                f"{_bytes_needed(patch.value_map)} bytes but a site is "
                f"{patch.size}")

    # num_sites (0 = no patch), then one (offset, size) header per site.
    buf += struct.pack("<B", len(sites))
    for patch in sites:
        buf += struct.pack("<HB", patch.offset, patch.size)

    # Enum params carry their allowed set; range params carry [min, max].
    # Only enum run params additionally carry per-value patch bytes.
    if param.param_type in ("enum", "range"):
        if len(param.values) > MAX_PARAM_VALUES:
            raise CompileError(
                f"param {param.name!r}: {len(param.values)} values exceeds "
                f"the {MAX_PARAM_VALUES}-value image limit")
        buf += struct.pack("<B", len(param.values))
        for v in param.values:
            buf += struct.pack("<I", _u32(v, f"param {param.name!r} value"))
        # Per-value patch bytes, site-major: all values for site 0, then site 1.
        if param.param_type == "enum":
            for patch in sites:
                for v in param.values:
                    pv = patch.value_map.get(v, 0)
                    if patch.size == 4:
                        buf += struct.pack("<I", _u32(pv, f"param {param.name!r} patch value"))
                    elif patch.size == 2:
                        buf += struct.pack("<H", int(pv) & 0xFFFF)
                    else:
                        buf += struct.pack("<B", int(pv) & 0xFF)

    unit_bytes = _text(param.unit, PARAM_UNIT_CHARS_MAX, f"parameter {param.name}'s unit")
    buf += struct.pack("<B", len(unit_bytes))
    buf += unit_bytes


def _read_param(data: bytes, pos: int):
    """Read a parameter descriptor. Returns (param, patches, new_pos)."""
    name_len = data[pos]; pos += 1
    name = data[pos:pos + name_len].decode('ascii'); pos += name_len

    param_type_byte = data[pos]; pos += 1
    if param_type_byte not in (0, 1):
        raise ValueError(f"invalid param type: {param_type_byte}")
    param_type = "enum" if param_type_byte == 0 else "range"
    kind_byte = data[pos]; pos += 1
    if kind_byte > _ParamKind.TRIGGER:
        raise ValueError(f"invalid param kind: {kind_byte}")
    kind = _ParamKind._NAMES[kind_byte].lower()

    default_val, current_val = struct.unpack_from("<II", data, pos); pos += 8
    num_sites = data[pos]; pos += 1
    if num_sites > MAX_PATCH_SITES:
        raise ValueError(
            f"param {name!r}: {num_sites} patch sites exceeds the "
            f"{MAX_PATCH_SITES}-site limit")
    site_offsets = []
    site_sizes = []
    for _ in range(num_sites):
        off = struct.unpack_from("<H", data, pos)[0]; pos += 2
        sz = data[pos]; pos += 1
        if sz not in (1, 2, 4):
            raise ValueError(
                f"param {name!r}: on-wire patch size {sz} must be 1, 2, or 4")
        site_offsets.append(off)
        site_sizes.append(sz)

    values = []
    site_value_maps = [dict() for _ in range(num_sites)]
    if param_type in ("enum", "range"):
        num_values = data[pos]; pos += 1
        if param_type == "range" and num_values != 2:
            raise ValueError(
                f"range param must carry [min, max], got {num_values} values")
        for _ in range(num_values):
            v = struct.unpack_from("<I", data, pos)[0]; pos += 4
            values.append(v)
        # Patch bytes: enum run params only; site-major, width per site.
        if param_type == "enum":
            for si in range(num_sites):
                for v in values:
                    if site_sizes[si] == 4:
                        pv = struct.unpack_from("<I", data, pos)[0]; pos += 4
                    elif site_sizes[si] == 2:
                        pv = struct.unpack_from("<H", data, pos)[0]; pos += 2
                    else:
                        pv = data[pos]; pos += 1
                    site_value_maps[si][v] = pv

    unit_len = data[pos]; pos += 1
    unit = data[pos:pos + unit_len].decode('ascii'); pos += unit_len

    param = ParamDescriptor(
        name=name, param_type=param_type, values=values,
        default=default_val, current=current_val, unit=unit, kind=kind,
    )

    patches = []
    for si in range(num_sites):
        if site_value_maps[si]:
            patches.append(PatchEntry(
                offset=site_offsets[si], param_name=name,
                value_map=site_value_maps[si], size=site_sizes[si],
            ))

    return param, patches, pos


def _write_output_field(buf: bytearray, field: dict, param_index=None):
    """Write an output field descriptor. After scale/offset comes a
    `scale_param_index` byte (0xFF = none), then `byte_off`; string fields
    carry a trailing `<H` count so the deserializer recovers `sample_size`."""
    name_bytes = _text(field['name'], NAME_CHARS_MAX, "output field")
    buf += struct.pack("<B", len(name_bytes))
    buf += name_bytes

    ftype = _FIELD_TYPE_MAP.get(field.get('type', 'int16'), 2)
    byte_order = 0 if field.get('byte_order', 'big') == 'big' else 1
    buf += struct.pack("<BBB", ftype, byte_order,
                       int(field.get('semantic', 0)))

    unit_bytes = _text(field.get('unit', ''), UNIT_CHARS_MAX, f"output field {field['name']}'s unit")
    buf += struct.pack("<B", len(unit_bytes))
    buf += unit_bytes

    buf += struct.pack("<dd",
                       float(field.get('scale', 1.0)),
                       float(field.get('offset', 0.0)))

    # scale_param_index: a `scale_param` name resolves through param_index, a
    # pre-resolved `scale_param_index` passes through, else 0xFF.
    scale_param = field.get('scale_param')
    if scale_param is not None and param_index is not None:
        scale_param_index = param_index.get(scale_param, 0xFF)
    else:
        scale_param_index = field.get('scale_param_index', 0xFF)
    buf += struct.pack("<B", scale_param_index & 0xFF)

    # Byte position of the field within the sample buffer.
    buf += struct.pack("<B", int(field['byte_off']) & 0xFF)

    if field.get('type') == 'string':
        buf += struct.pack("<H", int(field.get('count', 0)))


def _read_output_field(data: bytes, pos: int):
    """Read an output field descriptor. Returns (field_dict, new_pos).
    String fields carry a trailing `<H` count (see `_write_output_field`)."""
    name_len = data[pos]; pos += 1
    name = data[pos:pos + name_len].decode('ascii'); pos += name_len

    ftype_byte, byte_order_byte, semantic = (
        data[pos], data[pos + 1], data[pos + 2])
    pos += 3
    ftype = FIELD_TYPE_NAMES.get(ftype_byte, 'int16')
    byte_order = 'big' if byte_order_byte == 0 else 'little'

    unit_len = data[pos]; pos += 1
    unit = data[pos:pos + unit_len].decode('ascii'); pos += unit_len

    scale, offset = struct.unpack_from("<dd", data, pos); pos += 16
    scale_param_index = data[pos]; pos += 1
    byte_off = data[pos]; pos += 1

    field = {
        'name': name,
        'type': ftype,
        'byte_order': byte_order,
        'unit': unit,
        'scale': scale,
        'offset': offset,
        'semantic': semantic,
        'scale_param_index': scale_param_index,
        'byte_off': byte_off,
    }
    if ftype == 'string':
        field['count'] = struct.unpack_from("<H", data, pos)[0]
        pos += 2
    return field, pos


def _write_bus_config(buf: bytearray, bus_config) -> None:
    """Write the per-bus bus_config trailer: a count byte then one
    register-access / UART profile block per supported bus. A falsy
    bus_config writes a zero count (firmware keeps DTS defaults)."""
    if len(bus_config or []) > MAX_BUS_PROFILES:
        raise CompileError(
            f"bus_config has {len(bus_config)} profiles (max {MAX_BUS_PROFILES})")
    buf += struct.pack("<B", len(bus_config or []))
    for prof in bus_config or []:
        kind = prof['kind']
        if isinstance(kind, str):
            kind = _BUS_KIND_NAME_TO_INT.get(kind)
            if kind is None:
                raise ValueError(f"unknown bus kind: {prof['kind']!r}")
        buf += struct.pack("<BI", kind, int(prof.get('max_hz', 0)))
        if kind == BUS_KIND_SPI:
            buf += struct.pack(
                "<BBBBB",
                int(prof.get('spi_mode', 0)),
                int(prof.get('addr_bytes', 1)),
                int(prof.get('rw_read_level', 1)),
                int(prof.get('dummy_bytes', 0)),
                _code_int(_AUTO_INC_TO_INT, prof.get('auto_inc', 'implicit'), 'auto_inc'))
        elif kind == BUS_KIND_I2C:
            addr_bytes = int(prof.get('addr_bytes', 1))
            data_width = int(prof.get('data_width', 1))
            if addr_bytes not in (1, 2):
                raise ValueError(f"invalid bus_config addr_bytes: {addr_bytes}")
            if data_width not in (1, 2, 4):
                raise ValueError(f"invalid bus_config data_width: {data_width}")
            buf += struct.pack(
                "<BBBBB",
                _code_int(_AUTO_INC_TO_INT, prof.get('auto_inc', 'implicit'), 'auto_inc'),
                _code_int(_PEC_TO_INT, prof.get('pec', 'none'), 'pec'),
                addr_bytes,
                data_width,
                _code_int(_BYTE_ORDER_TO_INT, prof.get('byte_order', 'big'), 'byte_order'))
        elif kind == BUS_KIND_UART:
            buf += struct.pack("<BBB",
                               int(prof.get('uart_parity',    0)),
                               int(prof.get('uart_stop_bits', 1)),
                               int(prof.get('uart_data_bits', 8)))


def _read_bus_config(data: bytes, pos: int):
    """Read the per-bus bus_config trailer. Returns `(list_or_None, new_pos)`;
    a zero count reads as None (the firmware keeps DTS defaults)."""
    if pos >= len(data):
        raise ValueError("bus_config trailer missing (image truncated)")
    num = data[pos]
    pos += 1
    if num > MAX_BUS_PROFILES:
        raise ValueError(
            f"bus_config declares {num} profiles (max {MAX_BUS_PROFILES})")
    profiles = []
    for _ in range(num):
        if pos + 5 > len(data):
            raise ValueError("bus_config profile header truncated")
        kind = data[pos]
        pos += 1
        if kind not in _BUS_KIND_NAMES:
            raise ValueError(f"invalid bus_config kind: 0x{kind:02X}")
        max_hz = struct.unpack_from("<I", data, pos)[0]
        pos += 4
        bc = {'kind': _BUS_KIND_NAMES[kind], 'max_hz': max_hz}
        if kind == BUS_KIND_SPI:
            if pos + 5 > len(data):
                raise ValueError("bus_config spi body truncated")
            bc['spi_mode']      = data[pos]
            bc['addr_bytes']    = data[pos + 1]
            bc['rw_read_level'] = data[pos + 2]
            bc['dummy_bytes']   = data[pos + 3]
            bc['auto_inc']      = _code_name(_AUTO_INC_NAMES, data[pos + 4], 'auto_inc')
            pos += 5
        elif kind == BUS_KIND_I2C:
            if pos + 5 > len(data):
                raise ValueError("bus_config i2c body truncated")
            bc['auto_inc'] = _code_name(_AUTO_INC_NAMES, data[pos], 'auto_inc')
            bc['pec']      = _code_name(_PEC_NAMES, data[pos + 1], 'pec')
            if data[pos + 2] not in (1, 2):
                raise ValueError(f"invalid bus_config addr_bytes: {data[pos + 2]}")
            if data[pos + 3] not in (1, 2, 4):
                raise ValueError(f"invalid bus_config data_width: {data[pos + 3]}")
            bc['addr_bytes'] = data[pos + 2]
            bc['data_width'] = data[pos + 3]
            bc['byte_order'] = _code_name(_BYTE_ORDER_NAMES, data[pos + 4], 'byte_order')
            pos += 5
        elif kind == BUS_KIND_UART:
            if pos + 3 > len(data):
                raise ValueError("bus_config uart body truncated")
            bc['uart_parity']    = data[pos]
            bc['uart_stop_bits'] = data[pos + 1]
            bc['uart_data_bits'] = data[pos + 2]
            pos += 3
        profiles.append(bc)
    return (profiles or None), pos
