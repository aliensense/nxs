"""NXS personality-image binary format: serializer and deserializer. Layout:
13-byte header (magic, major, minor, kind, flags, name_len, num_params,
num_outputs, probe_len), name, bytecode section (plain, or nonce + ciphertext
when sealed), probe block, params, output fields, bus_config trailer,
descriptor trailer. Integers little-endian."""

import struct
from typing import List, Optional, Tuple

from nxs.compiler import (CompiledDriver, CompileError, ParamDescriptor,
                          PatchEntry, field_width, resolve_field_offsets)
from nxs._generated_constants import NxsDriverImage
from nxs.opcodes import Op, INSTRUCTION_SIZE, OPCODE_SINCE_MINOR

_ParamKind = NxsDriverImage.ParamKind
_BusKind = NxsDriverImage.BusKind
ImageKind = NxsDriverImage.ImageKind
TrailerRecord = NxsDriverImage.TrailerRecord

NXS_MAGIC = b"NXS\x00"
NXS_MAJOR = NxsDriverImage.NXS_MAJOR
NXS_MINOR = NxsDriverImage.NXS_MINOR

# Header: magic, major, required minor, kind, flags, name_len, num_params,
# num_outputs, probe_len (bytes of the probe program, the offset where
# configure begins).
_HEADER = struct.Struct("<4sBBBBBBBH")
HEADER_SIZE = _HEADER.size
# The v1.0 layout's header, recognised only to name the rebuild command.
_FORMAT1_HEADER_SIZE = 9

IMAGE_FLAG_SEALED = NxsDriverImage.IMAGE_FLAG_SEALED
IMAGE_FLAG_AUTO = NxsDriverImage.IMAGE_FLAG_AUTO
SEAL_NONCE_SIZE = NxsDriverImage.SEAL_NONCE_SIZE
MAX_TRAILER_SIZE = NxsDriverImage.MAX_TRAILER_SIZE
_TRAILER_RECORD_HEADER = struct.Struct("<BH")

IMAGE_KIND_NAMES = {ImageKind.DRIVER: 'driver', ImageKind.CAMERA: 'camera',
                    ImageKind.HUB: 'hub'}
#: The NXS minor that introduced a kind; an image of that kind never asks
#: for less, whatever its opcodes need.
IMAGE_KIND_SINCE_MINOR = {ImageKind.HUB: 3}

# The hint every format refusal carries; a caller that knows the
# personality substitutes its name.
REBUILD_HINT = "rebuild it: nxs upload <name>"

# Field type codes; firmware stores and forwards them verbatim, so every decoder
# must map through FIELD_TYPE_NAMES.
_FIELD_TYPE_MAP = {
    'int8': 0, 'uint8': 1, 'int16': 2, 'uint16': 3,
    'int32': 4, 'uint32': 5, 'float32': 6, 'float64': 7,
    'string': 8,
}
FIELD_TYPE_NAMES = {v: k for k, v in _FIELD_TYPE_MAP.items()}

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


def _required_minor(bytecode: bytes) -> int:
    """Minimum NXS minor an image needs: the max `since_minor` over its opcodes.
    Walks instruction by instruction (MEMCPY_IMM is variable-length)."""
    required = 0
    pc = 0
    n = len(bytecode)
    while pc < n:
        op = bytecode[pc]
        required = max(required, OPCODE_SINCE_MINOR.get(op, 0))
        size = INSTRUCTION_SIZE.get(op, 1) or 1
        if op == Op.MEMCPY_IMM and pc + 3 <= n:
            size = 3 + bytecode[pc + 2]
        pc += size
    return required


def serialize(compiled: CompiledDriver) -> bytes:
    """Serialize a CompiledDriver into the NXS binary format. Raises CompileError
    when a descriptor cap (`MAX_PARAMS`, `MAX_PARAM_VALUES`, `MAX_PATCH_SITES`,
    `MAX_TRAILER_SIZE`) or the program / image size limit is exceeded. A sealed
    section passes through as it was read: this tool never encrypts."""
    if len(compiled.params) > MAX_PARAMS:
        raise CompileError(
            f"{compiled.name}: {len(compiled.params)} params exceeds the "
            f"{MAX_PARAMS}-param image limit")
    # A hub image runs on the host executor and takes its program budget;
    # a pod's slot holds the rest.
    host = compiled.kind == ImageKind.HUB
    program_cap = VM_HOST_PROGRAM_SIZE if host else VM_MAX_PROGRAM_SIZE
    if len(compiled.bytecode) > program_cap:
        raise CompileError(
            f"{compiled.name}: {len(compiled.bytecode)} B of bytecode "
            f"exceeds the {program_cap} B VM program limit")
    if compiled.kind not in IMAGE_KIND_NAMES:
        raise ValueError(f"{compiled.name}: unknown image kind {compiled.kind!r}")

    buf = bytearray()
    name_bytes = compiled.name.encode('ascii')

    # A sealed section cannot be walked for its opcodes, so its required
    # minor is the one carried in from the read.
    if compiled.sealed:
        if compiled.required_minor is None:
            raise ValueError(
                f"{compiled.name}: a sealed image needs its required minor")
        if len(compiled.seal_nonce) != SEAL_NONCE_SIZE:
            raise ValueError(
                f"{compiled.name}: a sealed image carries a "
                f"{SEAL_NONCE_SIZE}-byte nonce, got {len(compiled.seal_nonce)}")
        required_minor = int(compiled.required_minor)
    else:
        # A reader older than the kind itself must refuse the image as
        # NEEDS_NEWER_MINOR rather than meet an unknown kind past the gate.
        required_minor = max(_required_minor(compiled.bytecode),
                             IMAGE_KIND_SINCE_MINOR.get(compiled.kind, 0))

    probe_len = int(compiled.probe_len)
    if not 0 <= probe_len <= len(compiled.bytecode):
        raise CompileError(
            f"{compiled.name}: probe length {probe_len} lies outside the "
            f"{len(compiled.bytecode)} B program")

    buf += _HEADER.pack(NXS_MAGIC, NXS_MAJOR, required_minor,
                        int(compiled.kind), int(compiled.flags) & 0xFF,
                        len(name_bytes), len(compiled.params),
                        len(compiled.output_fields), probe_len)

    # Driver name
    buf += name_bytes

    # Bytecode section: the nonce precedes the length only when sealed.
    if compiled.sealed:
        buf += compiled.seal_nonce
    buf += struct.pack("<H", len(compiled.bytecode))
    buf += compiled.bytecode

    # Probe block: WHO_AM_I anchor + candidate I²C addresses; empty for a
    # stream driver.
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
    `major.minor`, or None when it reads that format."""
    if major == NXS_MAJOR and minor <= NXS_MINOR:
        return None
    layout = " (the v1.0 image layout)" if major < NXS_MAJOR else ""
    return (f"image format {major}.{minor}{layout}, this tool builds "
            f"{NXS_MAJOR}.{NXS_MINOR} — {REBUILD_HINT}")


def peek_format(data: bytes) -> tuple:
    """`(major, minor, kind, flags)` from an NXS header. A v1.0 header (major
    below this tool's) answers kind DRIVER and no flags, the only values that
    layout could carry; a truncated artifact is rejected here."""
    if len(data) < _FORMAT1_HEADER_SIZE or data[:4] != NXS_MAGIC:
        raise ValueError(f"Not an NXS image header: {data[:HEADER_SIZE]!r}")
    major, minor = data[4], data[5]
    if major < NXS_MAJOR:
        return major, minor, ImageKind.DRIVER, 0
    if len(data) < HEADER_SIZE:
        raise ValueError(f"Not an NXS image header: {data[:HEADER_SIZE]!r}")
    return major, minor, data[6], data[7]


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
    if len(data) < HEADER_SIZE:
        raise ValueError("image header truncated")
    (_magic, major, minor, kind, flags, name_len, num_params,
     num_outputs, probe_len) = _HEADER.unpack_from(data, pos)
    pos += HEADER_SIZE
    # Major gates the wire layout. Minor is the image's required firmware minor,
    # enforced at load by the firmware, so any same-major image is inspectable.
    if major != NXS_MAJOR:
        refusal = format_refusal(major, minor)
        raise ValueError(refusal if major < NXS_MAJOR
                         else f"Unsupported major version: {major}")
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
    name_bytes = param.name.encode('ascii')
    buf += struct.pack("<B", len(name_bytes))
    buf += name_bytes

    if param.param_type == "enum":
        param_type = 0
    elif param.param_type == "range":
        param_type = 1
    else:
        raise ValueError(f"unknown param_type: {param.param_type!r}")
    buf += struct.pack("<B", param_type)
    kind = _ParamKind.LIVE if param.kind == "live" else _ParamKind.RELOAD
    buf += struct.pack("<B", kind)
    buf += struct.pack("<II", int(param.default), int(param.current))

    # This param's patch sites, ordered by offset for a deterministic wire
    # layout. Live params carry no patch: the value lives in current_value.
    sites = ([] if param.kind == "live"
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
    # Only enum reload params additionally carry per-value patch bytes.
    if param.param_type in ("enum", "range"):
        if len(param.values) > MAX_PARAM_VALUES:
            raise CompileError(
                f"param {param.name!r}: {len(param.values)} values exceeds "
                f"the {MAX_PARAM_VALUES}-value image limit")
        buf += struct.pack("<B", len(param.values))
        for v in param.values:
            buf += struct.pack("<I", int(v))
        # Per-value patch bytes, site-major: all values for site 0, then site 1.
        if param.param_type == "enum":
            for patch in sites:
                for v in param.values:
                    pv = patch.value_map.get(v, 0)
                    if patch.size == 4:
                        buf += struct.pack("<I", int(pv))
                    elif patch.size == 2:
                        buf += struct.pack("<H", int(pv) & 0xFFFF)
                    else:
                        buf += struct.pack("<B", int(pv) & 0xFF)

    unit_bytes = param.unit.encode('ascii')
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
    if kind_byte > _ParamKind.LIVE:
        raise ValueError(f"invalid param kind: {kind_byte}")
    kind = "live" if kind_byte == _ParamKind.LIVE else "reload"

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
        # Patch bytes: enum reload params only; site-major, width per site.
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
    name_bytes = field['name'].encode('ascii')
    buf += struct.pack("<B", len(name_bytes))
    buf += name_bytes

    ftype = _FIELD_TYPE_MAP.get(field.get('type', 'int16'), 2)
    byte_order = 0 if field.get('byte_order', 'big') == 'big' else 1
    buf += struct.pack("<BBB", ftype, byte_order,
                       int(field.get('semantic', 0)))

    unit_bytes = field.get('unit', '').encode('ascii')
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
