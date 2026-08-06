"""NXS serializer/deserializer roundtrip tests.

`test_iim20670_nxs_roundtrip` covers numeric-output drivers via the
iim20670 fixture; this file pins the image header, per-field semantics,
param round-trip, and the stale-artifact refusal paths."""

import pytest

from nxs.image import (
    serialize, deserialize, _required_minor, NXS_MAGIC, NXS_MAJOR, NXS_MINOR)
from nxs.opcodes import Op


def test_nxs_roundtrip_preserves_semantics():
    """Per-field semantic codes survive serialize → deserialize, and
    the compiler infers them from field names without driver edits."""
    from nxs.drivers.iam20680 import Iam20680
    cd = Iam20680().compile({
        'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000,
        'trigger': 'drdy'})
    cd2 = deserialize(serialize(cd))

    by_name = {f['name']: f['semantic'] for f in cd2.output_fields}
    assert by_name['accel_x'] == 1
    assert by_name['accel_z'] == 3
    assert by_name['gyro_y'] == 5
    assert by_name['temp'] == 10  # alias → temperature


def test_nxs_roundtrip_preserves_string_field_width():
    """A string output field's declared count and the driver's sample_size
    survive serialize → deserialize — the width the host needs to slice the
    record. The numeric case is covered above; this pins the string half."""
    from nxs.drivers.neo_m9n import NeoM9n
    cd = NeoM9n().compile({'protocol': 'nmea'})
    cd2 = deserialize(serialize(cd))
    assert cd2.sample_size == cd.sample_size
    orig = {f['name']: f['count'] for f in cd.output_fields
            if f.get('type') == 'string'}
    assert orig, "neo_m9n nmea should declare a string output field"
    for f in cd2.output_fields:
        if f.get('type') == 'string':
            assert f['count'] == orig[f['name']] > 0


# ── NXS major/minor header + required-minor stamping ──────────────────

def test_header_carries_major_and_required_minor():
    """The 9-byte header is magic(4) major(1) minor(1) ... — a base-opcode
    driver stamps required-minor 0 (the firmware floor)."""
    from nxs.drivers.iam20680 import Iam20680
    img = serialize(Iam20680().compile({
        'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'}))
    assert img[:4] == NXS_MAGIC
    assert img[4] == NXS_MAJOR
    assert img[5] == 0  # Iam20680 uses only since_minor-0 opcodes


def test_required_minor_is_max_over_emitted_opcodes():
    """An image needs the highest `since_minor` of any opcode it emits. Every
    opcode the 1.0 release ships is `since_minor` 0, so any image it can
    produce needs minor 0; the walk regains a non-zero gradient once a later
    release adds a higher-minor opcode."""
    assert _required_minor(bytes([Op.HALT])) == 0
    assert _required_minor(bytes([Op.CMP_LT, 0, 0, 0, 0, 0, 1, Op.HALT])) == 0
    assert _required_minor(bytes([Op.MUL64, 0, 8, 16, Op.HALT])) == 0


def test_required_minor_skips_memcpy_imm_inline_payload():
    """MEMCPY_IMM carries arbitrary inline data after its 3-byte header; the
    walk must treat those bytes as data, not opcodes, and resume decoding at
    the right boundary. With every 1.0 opcode at `since_minor` 0 the floor is
    0 either way — this pins the walk parsing the variable payload and
    re-syncing without choking. The false-inflation guard (a payload byte
    equal to a higher-minor opcode) re-arms with the first post-1.0 opcode."""
    trap = bytes([Op.MEMCPY_IMM, 0x00, 0x01, int(Op.CMP_LT), Op.HALT])
    assert _required_minor(trap) == 0
    resync = bytes([Op.MEMCPY_IMM, 0x00, 0x01, 0x00,
                    Op.CMP_LT, 0, 0, 0, 0, 0, 1, Op.HALT])
    assert _required_minor(resync) == 0


def test_deserialize_rejects_foreign_major():
    """A foreign major byte means an unparseable wire layout — reject it."""
    from nxs.drivers.iam20680 import Iam20680
    img = bytearray(serialize(Iam20680().compile({
        'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000, 'trigger': 'drdy'})))
    img[4] = NXS_MAJOR + 1  # bump the major
    with pytest.raises(ValueError, match="major"):
        deserialize(bytes(img))


def test_drive_pwm_range_params_round_trip():
    """Live range params carry [min, max] through NXS — before the range
    extension the deserializer dropped the bounds (values came back empty),
    so the firmware had no safety bound to validate a `set pwm_freq` against."""
    from nxs.compiler import RegisterDriver, SensorDriver, Sample

    class PwmDriver(RegisterDriver):
        def configure(self, config):
            self.set_sample_size(2)
            self.drive_pwm(freq=2000, duty=25)

        @SensorDriver.measure_loop(trigger="poll", sample_rate=10)
        def measure(self):
            raw = self.read_analog(0)
            return Sample(raw)

    cd2 = deserialize(serialize(PwmDriver().compile()))
    by_name = {p.name: p for p in cd2.params}

    freq = by_name['pwm_freq']
    assert freq.param_type == 'range'
    assert freq.kind == 'live'
    assert freq.values == [500, 25000]   # [min, max] preserved
    assert freq.default == 2000
    assert by_name['pwm_duty'].values == [0, 100]


def test_read_param_rejects_invalid_param_type():
    from nxs.image import _read_param
    # name_len=0, then param_type byte = 2 (only 0=enum / 1=range are valid).
    try:
        _read_param(bytes([0x00, 0x02]), 0)
        assert False, "expected ValueError for invalid param_type byte"
    except ValueError:
        pass


def test_read_param_rejects_malformed_range():
    from nxs.image import _read_param
    # A range param (type=1) must carry exactly [min, max]; here num_values=3.
    body = bytes([0x00,             # name_len
                  0x01,             # param_type = range
                  0x01,             # kind = live
                  0, 0, 0, 0,       # default
                  0, 0, 0, 0,       # current
                  0xFF, 0xFF,       # patch_offset = none
                  0x00,             # patch_size
                  0x03])            # num_values = 3 (invalid for range)
    try:
        _read_param(body, 0)
        assert False, "expected ValueError for malformed range"
    except ValueError:
        pass


def test_write_param_rejects_unknown_param_type():
    from nxs.image import _write_param
    from nxs.compiler import ParamDescriptor
    bad = ParamDescriptor(name="x", param_type="bogus", values=[],
                          default=0, current=0, unit="", kind="reload")
    try:
        _write_param(bytearray(), bad, [])
        assert False, "expected ValueError for unknown param_type"
    except ValueError:
        pass


def test_write_param_rejects_value_wider_than_patch_size():
    from nxs.image import _write_param
    from nxs.compiler import ParamDescriptor, PatchEntry
    param = ParamDescriptor(name="x", param_type="enum", values=[1, 2],
                            default=1, current=1, unit="", kind="reload")
    # size=1 site, but a mapped value needs two bytes.
    patch = PatchEntry(offset=0, param_name="x",
                       value_map={1: 0x1234, 2: 0x5678}, size=1)
    try:
        _write_param(bytearray(), param, [patch])
        assert False, "expected ValueError for a value wider than the patch site"
    except ValueError:
        pass


def test_patch_size_survives_deserialize_roundtrip():
    # serialize -> deserialize -> serialize must preserve the patch width;
    # a deserialize that dropped size collapsed a 2-byte patch to 1 byte.
    from nxs.image import _write_param, _read_param
    from nxs.compiler import ParamDescriptor, PatchEntry
    param = ParamDescriptor(name="rate", param_type="enum", values=[1, 2],
                            default=1, current=1, unit="", kind="reload")
    patch = PatchEntry(offset=4, param_name="rate",
                       value_map={1: 0x0111, 2: 0x0222}, size=2)
    buf = bytearray()
    _write_param(buf, param, [patch])
    param2, patches2, _ = _read_param(bytes(buf), 0)
    assert patches2 and patches2[0].size == 2
    buf2 = bytearray()
    _write_param(buf2, param2, patches2)
    assert bytes(buf2) == bytes(buf)


def test_multi_site_patch_roundtrip():
    # One param patching two registers: both sites survive serialize →
    # deserialize with their own offset, size, and per-value bytes.
    from nxs.image import _write_param, _read_param
    from nxs.compiler import ParamDescriptor, PatchEntry
    param = ParamDescriptor(name="filter", param_type="enum", values=[10, 60],
                            default=60, current=60, unit="Hz", kind="reload")
    sites = [
        PatchEntry(offset=8, param_name="filter",
                   value_map={10: 0x0041, 60: 0x0820}, size=4),
        PatchEntry(offset=20, param_name="filter",
                   value_map={10: 0x0100, 60: 0x2000}, size=4),
    ]
    buf = bytearray()
    _write_param(buf, param, sites)
    _param2, patches2, _ = _read_param(bytes(buf), 0)
    assert len(patches2) == 2
    by_off = {p.offset: p for p in patches2}
    assert by_off[8].value_map == {10: 0x0041, 60: 0x0820}
    assert by_off[20].value_map == {10: 0x0100, 60: 0x2000}
    buf2 = bytearray()
    _write_param(buf2, param, patches2)
    assert bytes(buf2) == bytes(buf)


def test_write_param_rejects_over_max_patch_sites():
    # Defense in depth: even if a hand-built patch_map bypasses the compiler's
    # cap, the serializer refuses more than MAX_PATCH_SITES sites per param.
    from nxs.image import _write_param, MAX_PATCH_SITES
    from nxs.compiler import ParamDescriptor, PatchEntry, CompileError
    param = ParamDescriptor(name="x", param_type="enum", values=[1, 2],
                            default=1, current=1, unit="", kind="reload")
    sites = [PatchEntry(offset=4 * i, param_name="x",
                        value_map={1: 1, 2: 2}, size=1)
             for i in range(MAX_PATCH_SITES + 1)]
    try:
        _write_param(bytearray(), param, sites)
        assert False, "expected CompileError for over-cap patch sites"
    except CompileError as e:
        assert "site image limit" in str(e)


def test_write_param_rejects_value_outside_uint32():
    from nxs.image import _write_param
    from nxs.compiler import ParamDescriptor, PatchEntry
    param = ParamDescriptor(name="x", param_type="enum", values=[1, 2],
                            default=1, current=1, unit="", kind="reload")
    patch = PatchEntry(offset=0, param_name="x",
                       value_map={1: 1, 2: 0x1_0000_0000}, size=4)  # > uint32
    try:
        _write_param(bytearray(), param, [patch])
        assert False, "expected ValueError for a value outside uint32"
    except ValueError:
        pass


def test_write_param_rejects_unsupported_patch_size():
    from nxs.image import _write_param
    from nxs.compiler import ParamDescriptor, PatchEntry
    param = ParamDescriptor(name="x", param_type="enum", values=[1, 2],
                            default=1, current=1, unit="", kind="reload")
    patch = PatchEntry(offset=0, param_name="x", value_map={1: 1, 2: 2}, size=3)
    try:
        _write_param(bytearray(), param, [patch])
        assert False, "expected ValueError for patch size 3"
    except ValueError:
        pass


# ── Stale-artifact refusal at upload ───────────────────────────────────

def _compiled_image() -> bytearray:
    from nxs.drivers.iam20680 import Iam20680
    return bytearray(serialize(Iam20680().compile({
        'sample_rate': 250, 'accel_fs': 8, 'gyro_fs': 2000,
        'trigger': 'drdy'})))


def test_peek_format_reads_header():
    from nxs.image import peek_format
    img = _compiled_image()
    assert peek_format(bytes(img)) == (NXS_MAJOR, 0)
    with pytest.raises(ValueError, match="NXS image"):
        peek_format(b"#!py not an image")
    # Truncated below the 9-byte header — including a valid magic with
    # major/minor but missing counts — must reject, not crash or pass.
    for n in range(9):
        with pytest.raises(ValueError, match="NXS image"):
            peek_format(bytes(img[:n]))


def _run_upload(tmp_path, img: bytes, capsys):
    """Drive cmd_upload's file branch with a dummy transport; the refusal
    paths return before the transport is touched."""
    import argparse
    from nxs.cli import cmd_upload
    path = tmp_path / "stale.nxs"
    path.write_bytes(img)
    args = argparse.Namespace(driver=str(path), config=None, output=None)
    rc = cmd_upload(None, args)

    return rc, capsys.readouterr().err


def test_upload_refuses_stale_major(tmp_path, capsys):
    img = _compiled_image()
    img[4] = NXS_MAJOR + 1
    rc, err = _run_upload(tmp_path, bytes(img), capsys)
    assert rc == 1
    assert f"image format {NXS_MAJOR + 1}.0, this tool builds" in err
    assert "rebuild it: nxs upload stale" in err


def test_upload_refuses_needs_newer_minor(tmp_path, capsys):
    img = _compiled_image()
    img[5] = NXS_MINOR + 1
    rc, err = _run_upload(tmp_path, bytes(img), capsys)
    assert rc == 1
    assert f"image format {NXS_MAJOR}.{NXS_MINOR + 1}" in err


def test_upload_refuses_non_image_file(tmp_path, capsys):
    rc, err = _run_upload(tmp_path, b"#!py not an image", capsys)
    assert rc == 1
    assert "not an NXS image" in err
    assert "nxs upload stale" in err


# ── Descriptor-cap enforcement (host mirror of the firmware parser) ──

def _driver_with_param_values(n: int):
    """A minimal CompiledDriver carrying one enum param with `n` values —
    the shape that trips MAX_PARAM_VALUES."""
    from nxs.compiler import CompiledDriver, ParamDescriptor
    return CompiledDriver(
        bytecode=b"", sample_size=0, name="capfix",
        params=[ParamDescriptor(
            name="rate", param_type="enum", values=list(range(n)),
            default=0, current=0)])


def test_serialize_rejects_too_many_params():
    """More than MAX_PARAMS params raises at build, not at device parse."""
    from nxs.compiler import CompiledDriver, ParamDescriptor, CompileError
    from nxs.image import MAX_PARAMS
    over = CompiledDriver(
        bytecode=b"", sample_size=0, name="capfix",
        params=[ParamDescriptor(name=f"p{i}", param_type="enum",
                                values=[0], default=0, current=0)
                for i in range(MAX_PARAMS + 1)])
    with pytest.raises(CompileError, match="params exceeds"):
        serialize(over)


def test_serialize_enforces_param_values_cap_exactly():
    """The cap is inclusive at MAX_PARAM_VALUES and rejects one past it —
    the boundary is what pins the exact limit (mc6470's 9-value rate must
    pass, a hypothetical 17-value param must not)."""
    from nxs.compiler import CompileError
    from nxs.image import MAX_PARAM_VALUES

    # At the cap: must serialize (this is the assertion mc6470's 9-value
    # rate relies on — the at-cap success is what pins the exact limit).
    serialize(_driver_with_param_values(MAX_PARAM_VALUES))

    # One past the cap: must reject at build.
    with pytest.raises(CompileError, match="values exceeds"):
        serialize(_driver_with_param_values(MAX_PARAM_VALUES + 1))


def test_serialize_rejects_oversized_bytecode():
    """The bytecode cap is enforced at build (SSOT constant), so an
    over-cap program raises CompileError instead of compiling to an
    image the firmware rejects at load with PROGRAM_TOO_LARGE."""
    import dataclasses
    import pytest
    from nxs.compiler import CompileError
    from nxs.drivers.iam20680 import Iam20680
    from nxs.image import serialize, VM_MAX_PROGRAM_SIZE

    cd = Iam20680().compile({})
    fat = dataclasses.replace(cd, bytecode=bytes(VM_MAX_PROGRAM_SIZE + 1))
    with pytest.raises(CompileError, match="VM program limit"):
        serialize(fat)


def test_serialize_rejects_oversized_image(monkeypatch):
    """The image cap is the aggregate header+metadata+bytecode invariant:
    a cap-size program plus a maximal descriptor set can exceed it even
    when both individual caps hold. Real drivers sit far below the
    corner, so the check is pinned directly with a lowered cap rather
    than by constructing a maximal-descriptor fixture."""
    import dataclasses
    import pytest
    from nxs import image
    from nxs.compiler import CompileError
    from nxs.drivers.iim20670 import Iim20670
    from nxs.image import serialize, VM_MAX_PROGRAM_SIZE

    cd = Iim20670().compile({})   # ~625 B of metadata
    fat = dataclasses.replace(cd, bytecode=bytes(VM_MAX_PROGRAM_SIZE))
    monkeypatch.setattr(image, 'MAX_DRIVER_IMAGE_SIZE',
                        VM_MAX_PROGRAM_SIZE + 100)
    with pytest.raises(CompileError, match="image limit"):
        serialize(fat)
