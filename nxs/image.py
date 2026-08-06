"""
NXS driver-image binary format — serializer and deserializer.

The NXS format stores everything needed to run a sensor driver on NXS:
bytecode, parameter descriptors (capabilities), patch map (for runtime
parameter changes), output field descriptors (for sample parsing), and
a probe block so NXS can auto-detect the sensor's I²C address against
its WHO_AM_I register instead of guessing.

Binary layout:
  Header (9B) → Name → Bytecode → Probe → Params → Output fields

The 9-byte header is magic(4) major(1) minor(1) name_len(1) num_params(1)
num_outputs(1). `major` is the incompatible-change counter; `minor` is the
minimum NXS minor this image needs — the max `since_minor` over the opcodes
it emits. Firmware runs the image iff major == NXS_MAJOR and minor <= the
firmware's NXS_MINOR.

All multi-byte integers are little-endian. Strings are length-prefixed
ASCII (no null terminator).
"""

import struct
from typing import List

from nxs.compiler import (CompiledDriver, CompileError, ParamDescriptor,
                          PatchEntry, field_width, resolve_field_offsets)
from nxs._generated_constants import NxsDriverImage
from nxs.opcodes import Op, INSTRUCTION_SIZE, OPCODE_SINCE_MINOR

_ParamKind = NxsDriverImage.ParamKind
_BusKind = NxsDriverImage.BusKind

NXS_MAGIC = b"NXS\x00"
NXS_MAJOR = NxsDriverImage.NXS_MAJOR
NXS_MINOR = NxsDriverImage.NXS_MINOR

# Field type encoding (matches descriptor.py _TYPE_INFO keys). This is
# the canonical numeric table: firmware stores and forwards these codes
# verbatim (DriverImage field_type, the firmware output-info record, the I2C register
# window), so every decoder must map through FIELD_TYPE_NAMES.
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
MAX_DRIVER_IMAGE_SIZE = NxsDriverImage.MAX_DRIVER_IMAGE_SIZE

_BUS_KIND_NAMES = {
    BUS_KIND_I2C:  'i2c',
    BUS_KIND_SPI:  'spi',
    BUS_KIND_UART: 'uart',
}
_BUS_KIND_NAME_TO_INT = {v: k for k, v in _BUS_KIND_NAMES.items()}

# Register-access switch codes — mirror the AutoInc / Pec enums in
# constants/driver_image.yaml (same SSOT the firmware reads), so the
# wire codes can't drift between host and device.
_AutoInc = NxsDriverImage.AutoInc
_Pec = NxsDriverImage.Pec
_AUTO_INC_TO_INT = {'implicit': _AutoInc.IMPLICIT, 'msb': _AutoInc.MSB,
                    'none': _AutoInc.NONE}
_AUTO_INC_NAMES = {v: k for k, v in _AUTO_INC_TO_INT.items()}
_PEC_TO_INT = {'none': _Pec.NONE, 'crc8': _Pec.CRC8}
_PEC_NAMES = {v: k for k, v in _PEC_TO_INT.items()}


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
    """Minimum NXS minor an image needs: the max `since_minor` over the
    opcodes it emits. Walks the bytecode instruction-by-instruction —
    mirroring the disassembler's MEMCPY_IMM variable-length step — so an
    opcode's operand or inline-data bytes are never misread as opcodes."""
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
    """Serialize a CompiledDriver into the NXS binary format.

    Enforces the shared descriptor caps (`MAX_PARAMS`, `MAX_PARAM_VALUES` —
    SSOT `constants/driver_image.yaml`) at build time, so an over-cap driver
    raises a `CompileError` here instead of compiling to an image the firmware
    rejects at load with `TOO_MANY_PARAMS` / `PARAM_TOO_MANY_VALUES`. The
    output-field count (`MAX_OUTPUTS`) is capped earlier, in the compiler's
    `validate_field_layout`.
    """
    if len(compiled.params) > MAX_PARAMS:
        raise CompileError(
            f"{compiled.name}: {len(compiled.params)} params exceeds the "
            f"{MAX_PARAMS}-param image limit")
    if len(compiled.bytecode) > VM_MAX_PROGRAM_SIZE:
        raise CompileError(
            f"{compiled.name}: {len(compiled.bytecode)} B of bytecode "
            f"exceeds the {VM_MAX_PROGRAM_SIZE} B VM program limit")

    buf = bytearray()
    name_bytes = compiled.name.encode('ascii')

    # Header (9 bytes): magic, major, required-minor, name_len, num_params,
    # num_outputs. The minor is computed from the emitted opcodes so an
    # image transparently demands the firmware that can run it.
    buf += NXS_MAGIC
    buf += struct.pack("<BBBBB",
                       NXS_MAJOR,
                       _required_minor(compiled.bytecode),
                       len(name_bytes),
                       len(compiled.params),
                       len(compiled.output_fields))

    # Driver name
    buf += name_bytes

    # Bytecode section
    buf += struct.pack("<H", len(compiled.bytecode))
    buf += compiled.bytecode

    # Probe block — WHO_AM_I anchor + candidate I²C addresses. Empty
    # when the driver doesn't expose these class attributes (e.g.,
    # StreamDriver, or authors using the legacy template).
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

    # Output field descriptors. A field's optional `scale_param` is
    # resolved to a param index here, where the final param order is
    # known — the firmware multiplies the base scale by that param's live
    # current_value to form the effective SI scale. Byte offsets are
    # resolved (sequential where not explicit) before writing, so every
    # image carries an explicit position per field.
    param_index = {p.name: i for i, p in enumerate(compiled.params)}
    for field in resolve_field_offsets(compiled.output_fields):
        _write_output_field(buf, field, param_index)

    # Optional bus_config trailer. Driver omits this entirely when
    # `bus_config` is None / absent — firmware then keeps DTS defaults.
    bus_config = getattr(compiled, 'bus_config', None)
    if bus_config is not None:
        _write_bus_config(buf, bus_config)

    # The image cap is a storage contract (store slots fill the NVS
    # partition exactly); the enforced invariant is the aggregate
    # header + metadata + bytecode, not a per-part split.
    if len(buf) > MAX_DRIVER_IMAGE_SIZE:
        raise CompileError(
            f"{compiled.name}: {len(buf)} B image exceeds the "
            f"{MAX_DRIVER_IMAGE_SIZE} B image limit "
            f"({len(compiled.bytecode)} B bytecode + "
            f"{len(buf) - len(compiled.bytecode)} B header/metadata)")

    return bytes(buf)


def peek_format(data: bytes) -> tuple:
    """Image-format (major, required-minor) from an NXS header — cheap
    staleness check before an upload. Requires the full 9-byte header
    (magic, major, minor, name/param/output counts), so a truncated
    artifact is rejected here instead of on-device."""
    if len(data) < 9 or data[:4] != NXS_MAGIC:
        raise ValueError(f"Not an NXS image header: {data[:9]!r}")

    return data[4], data[5]


def deserialize(data: bytes) -> CompiledDriver:
    """Deserialize an NXS binary image into a CompiledDriver."""
    pos = 0

    # Header
    magic = data[pos:pos + 4]
    if magic != NXS_MAGIC:
        raise ValueError(f"Bad magic: {magic!r}")
    pos += 4

    major, _minor, name_len, num_params, num_outputs = struct.unpack_from(
        "<BBBBB", data, pos)
    pos += 5
    # Major gates wire-layout compatibility — a foreign major can't be
    # parsed at all. Minor is the image's required firmware minor; the
    # host can inspect any same-major image, so it isn't gated here (the
    # firmware parser enforces required-minor <= its NXS_MINOR at load).
    if major != NXS_MAJOR:
        raise ValueError(f"Unsupported major version: {major}")
    if num_outputs > MAX_OUTPUTS:
        raise ValueError(
            f"image declares {num_outputs} output fields (max {MAX_OUTPUTS})")

    # Name
    name = data[pos:pos + name_len].decode('ascii')
    pos += name_len

    # Bytecode
    bytecode_len = struct.unpack_from("<H", data, pos)[0]
    pos += 2
    bytecode = data[pos:pos + bytecode_len]
    pos += bytecode_len

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

    # Optional bus_config trailer.
    bus_config, pos = _read_bus_config(data, pos)

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
    )
    # Probe fields — set as attributes rather than ctor args so older
    # CompiledDriver shapes (and fresh compile() outputs) coexist.
    cd.who_am_i_reg = wai_reg
    cd.who_am_i_values = who_am_i_values
    cd.i2c_addrs = i2c_addrs
    cd.bus_config = bus_config
    return cd


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

    # Collect this param's patch sites, ordered by offset for a deterministic
    # wire layout. A param may patch more than one register (e.g. a filter
    # cutoff written to an X register and a Y/Z register), each its own site.
    # Live params apply without a reload — the value lives in current_value,
    # never a bytecode operand — so they never carry a patch.
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
        # would otherwise crash in struct.pack rather than report cleanly.
        for v in patch.value_map.values():
            if not 0 <= int(v) <= 0xFFFFFFFF:
                raise ValueError(
                    f"param {param.name!r}: patch value {v} is outside the "
                    f"uint32 range")
        # The patch width is declared at the write site (write=1, set_baud=4,
        # stage=explicit); the firmware writes exactly that many bytes. Every
        # value must fit, and the width must match a supported store.
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
        # Patch bytes — enum reload params only; site-major, width per site.
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
    `scale_param_index` byte (0xFF = none): the declared-param index whose
    live value multiplies the base scale on-device. String fields then
    carry a trailing `<H` count so the deserializer can recover
    `sample_size` — numeric types' size is implied by `ftype`, but a
    string's payload width is driver-defined (`MAX_READ_SIZE`-style)."""
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

    # scale_param_index: resolve a `scale_param` name through param_index
    # (the compile path), fall back to a pre-resolved `scale_param_index`
    # (the deserialize→re-serialize path), else 0xFF for "no live param".
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
    bus_config writes nothing (firmware keeps DTS defaults)."""
    if not bus_config:
        return
    if len(bus_config) > MAX_BUS_PROFILES:
        raise CompileError(
            f"bus_config has {len(bus_config)} profiles (max {MAX_BUS_PROFILES})")
    buf += struct.pack("<B", len(bus_config))
    for prof in bus_config:
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
            buf += struct.pack(
                "<BB",
                _code_int(_AUTO_INC_TO_INT, prof.get('auto_inc', 'implicit'), 'auto_inc'),
                _code_int(_PEC_TO_INT, prof.get('pec', 'none'), 'pec'))
        elif kind == BUS_KIND_UART:
            buf += struct.pack("<BBB",
                               int(prof.get('uart_parity',    0)),
                               int(prof.get('uart_stop_bits', 1)),
                               int(prof.get('uart_data_bits', 8)))


def _read_bus_config(data: bytes, pos: int):
    """Read the per-bus bus_config trailer. Returns `(list_or_None, new_pos)`.
    Absent trailer → (None, pos)."""
    if pos >= len(data):
        return None, pos
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
            if pos + 2 > len(data):
                raise ValueError("bus_config i2c body truncated")
            bc['auto_inc'] = _code_name(_AUTO_INC_NAMES, data[pos], 'auto_inc')
            bc['pec']      = _code_name(_PEC_NAMES, data[pos + 1], 'pec')
            pos += 2
        elif kind == BUS_KIND_UART:
            if pos + 3 > len(data):
                raise ValueError("bus_config uart body truncated")
            bc['uart_parity']    = data[pos]
            bc['uart_stop_bits'] = data[pos + 1]
            bc['uart_data_bits'] = data[pos + 2]
            pos += 3
        profiles.append(bc)
    return (profiles or None), pos
