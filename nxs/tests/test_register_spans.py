"""The register-map generator rejects overlapping spans and reserved-range
grabs — the check that turns an address collision into a build error."""
import importlib.util
import os
import tempfile

import pytest

# The generator lives in the firmware repo beside the SDK tree; a standalone
# SDK checkout has no register YAML to generate from, so the suite skips.
_GEN = os.path.join(os.path.dirname(__file__), "..", "..", "..",
                    "scripts", "generate-constants.py")
if not os.path.exists(_GEN):
    pytest.skip("register-map generator lives in the firmware repo",
                allow_module_level=True)
_spec = importlib.util.spec_from_file_location("genconst", _GEN)
genconst = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(genconst)

_HEADER = """\
cpp_namespace: rb::test
cpp_header:    vendor/constants/test.h
python_class:  TestRegs
enums:
  - name: Reg
    underlying: uint8_t
    address_space: true
"""


def _parse(body):
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "regs.yaml")
        with open(path, "w") as f:
            f.write(_HEADER + body)
        return genconst.parse_category(genconst.Path(path))


def test_real_register_map_validates():
    root = os.path.join(os.path.dirname(_GEN), "..")
    genconst.parse_category(
        genconst.Path(os.path.join(root, "constants", "axon_registers.yaml")))


def test_wide_register_neighbor_collides():
    """The regression: a u16 register's high byte is not a free cell."""
    with pytest.raises(ValueError, match="overlaps"):
        _parse("""\
    values:
      - { name: VALUE, value: 0xEB, width: 2 }
      - { name: TERM,  value: 0xEC }
""")


def test_disjoint_views_may_overlay():
    cat = _parse("""\
    values:
      - { name: A, value: 0xD2, width: 4, view: param }
      - { name: B, value: 0xD2, width: 4, view: output }
""")
    assert cat is not None


def test_shared_view_overlap_rejected():
    with pytest.raises(ValueError, match="overlaps"):
        _parse("""\
    values:
      - { name: A, value: 0xD3, view: "driver,peek" }
      - { name: B, value: 0xD3, view: peek }
""")


def test_reserved_range_grab_rejected():
    with pytest.raises(ValueError, match="reserved"):
        _parse("""\
    reserved:
      - { from: 0xFE, to: 0xFF, why: kept }
    values:
      - { name: A, value: 0xFE }
""")
