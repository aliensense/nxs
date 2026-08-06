"""Compile the driver-code snippets in the docs against the real nxs compiler.

A `measure()` body compiles to VM bytecode that resolves only integer *literals*
— the FIFO recipe once shipped with named constants (`FIFO_COUNT`, `RST | EN`)
that raise `CompileError`. This gate makes teaching material that can't compile a
test failure, not a bring-up surprise.

Scope: self-contained fences only — a complete module (its own imports + a
driver subclass, e.g. the released spec's worked example), a full driver body
(both `def configure` and `def measure`), or a bare `measure()` body that
commits `Sample(raw)`. Partial fragments (a lone `declare_param`, a measure that
references coefficients defined in an omitted `configure`) and `<placeholder>`
templates are skipped; they are illustrative, not copy-paste drivers.
"""
import importlib.util
import re
import sys
from pathlib import Path

import pytest

_SDK_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SDK_ROOT))

# The skill ships with the SDK tree; the other two docs exist only when the
# tree sits inside the firmware repo, and drop out of the parametrization in
# a standalone checkout.
_FIRMWARE_REPO = _SDK_ROOT.parent
_DOCS = [p for p in (
    _SDK_ROOT / "skills" / "nxs-generate-sensor-driver" / "SKILL.md",
    _FIRMWARE_REPO / "docs" / "zephyr" / "axon" / "driver-authors-guide.md",
    _FIRMWARE_REPO / "docs" / "specs" / "nxs-driver-development.md",
) if p.exists()]

_FENCE = re.compile(r"^```python\n(.*?)^```", re.DOTALL | re.MULTILINE)

_SHELL_HEAD = (
    "from nxs.compiler import RegisterDriver, StreamDriver, Sample, SensorDriver\n"
    "class _Snippet(RegisterDriver):\n"
    "    BUSES = ('i2c', 'spi')\n"
    "    WHO_AM_I_VALUES = []\n"
    "    WHO_AM_I_SKIP_REASON = 'doc snippet'\n"
)
_STUB_CONFIGURE = (
    "    def configure(self, config):\n"
    "        self.set_output([{'name': 'raw', 'type': 'uint16',\n"
    "                          'scale': 1.0, 'unit': ''}])\n"
    "        self.set_sample_size(14)\n"
)
_MEASURE_DECO = "    @RegisterDriver.measure_loop(trigger='drdy')\n    def measure(self):\n"


def _indent(src: str, n: int) -> str:
    pad = " " * n
    return "\n".join(pad + ln if ln.strip() else ln for ln in src.splitlines())


def _snippets():
    for doc in _DOCS:
        text = doc.read_text()
        for m in _FENCE.finditer(text):
            body = m.group(1)
            line = text[: m.start()].count("\n") + 2
            yield f"{doc.name}:{line}", body


_PLACEHOLDER = re.compile(r"<[A-Za-z_][\w ]*>")   # <SensorName>, <start_reg>, …


def _module_source(body: str):
    """Wrap a self-contained snippet into a driver module, or None to skip.

    Self-contained = a full driver (configure + measure), or a `measure` that
    commits `Sample(raw)` without configure-defined coefficients (`self.c…`).
    A measure that needs coefficients or named `Sample(field=…)` outputs from an
    omitted configure is not standalone and is skipped."""
    if _PLACEHOLDER.search(body):
        return None                       # fill-in placeholder template
    # A self-contained module — its own imports + a driver subclass — runs
    # as-is (the released spec's complete-driver example). Checked before the
    # wrapped-body cases so it isn't nested inside the _Snippet shell.
    if re.search(r"^(from|import) ", body, re.M) and \
            re.search(r"^class \w+\(", body, re.M):
        return body
    has_cfg = "def configure(" in body
    has_meas = "def measure(" in body
    standalone_measure = "return Sample(raw)" in body and "self.c" not in body
    if has_meas and has_cfg:
        return _SHELL_HEAD + _indent(body, 4) + "\n"
    if has_meas and standalone_measure:
        return _SHELL_HEAD + _STUB_CONFIGURE + _indent(body, 4) + "\n"
    if "def " not in body and standalone_measure:
        return _SHELL_HEAD + _STUB_CONFIGURE + _MEASURE_DECO + _indent(body, 8) + "\n"
    return None                           # not a self-contained driver snippet


def _compile_module(src: str, tmp_path):
    # A real file is required: the compiler reads the measure() body via
    # inspect.getsource. tmp_path is a per-test dir pytest cleans up.
    path = tmp_path / "doc_snippet.py"
    path.write_text(src)
    spec = importlib.util.spec_from_file_location("doc_snippet", str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    from nxs.compiler import SensorDriver
    drivers = [v for v in vars(mod).values()
               if isinstance(v, type) and issubclass(v, SensorDriver)
               and v.__module__ == mod.__name__]
    assert drivers, "snippet defines no driver class"
    for d in drivers:
        d().compile({})


_CASES = [(tag, body) for tag, body in _snippets() if _module_source(body)]


@pytest.mark.parametrize("tag,body", _CASES, ids=[t for t, _ in _CASES])
def test_doc_snippet_compiles(tag, body, tmp_path):
    _compile_module(_module_source(body), tmp_path)


def test_gate_covers_the_fifo_recipe():
    # Guard the guard: the FIFO recipe (the snippet class that broke) must be
    # among the compiled cases, so a future edit to it is actually checked.
    assert any("read_burst(0x46, 14)" in body for _, body in _CASES), \
        "the FIFO recipe snippet is no longer being compiled by the gate"
