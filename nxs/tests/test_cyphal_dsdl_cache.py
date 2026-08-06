"""The DSDL-cache staleness key: it must change when a source .dsdl is added,
removed, or its mtime/size changes, and stay stable otherwise — that gate is
what lets _ensure_dsdl() skip pycyphal's ~2 s compile_all re-validation."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from nxs.transports.cyphal_source import _dsdl_fingerprint


def test_fingerprint_stable_when_unchanged(tmp_path):
    ns = tmp_path / "uavcan"
    ns.mkdir()
    (ns / "Foo.1.0.dsdl").write_text("uint8 x\n")
    fp = _dsdl_fingerprint([str(ns)])
    assert _dsdl_fingerprint([str(ns)]) == fp


def test_fingerprint_changes_on_mtime(tmp_path):
    ns = tmp_path / "uavcan"
    ns.mkdir()
    f = ns / "Foo.1.0.dsdl"
    f.write_text("uint8 x\n")
    fp1 = _dsdl_fingerprint([str(ns)])
    st = f.stat()
    os.utime(f, (st.st_atime, st.st_mtime + 10))
    assert _dsdl_fingerprint([str(ns)]) != fp1


def test_fingerprint_changes_on_added_file(tmp_path):
    ns = tmp_path / "uavcan"
    ns.mkdir()
    (ns / "Foo.1.0.dsdl").write_text("uint8 x\n")
    fp1 = _dsdl_fingerprint([str(ns)])
    (ns / "Bar.1.0.dsdl").write_text("uint8 y\n")
    assert _dsdl_fingerprint([str(ns)]) != fp1


def test_fingerprint_tolerates_vanished_file(tmp_path, monkeypatch):
    """A .dsdl that disappears between os.walk and os.stat (concurrent
    checkout/build) must not crash the fingerprint."""
    ns = tmp_path / "uavcan"
    ns.mkdir()
    (ns / "Foo.1.0.dsdl").write_text("uint8 x\n")
    real_stat = os.stat

    def flaky_stat(path, *a, **k):
        if str(path).endswith(".dsdl"):
            raise FileNotFoundError(path)
        return real_stat(path, *a, **k)

    monkeypatch.setattr(os, "stat", flaky_stat)
    assert isinstance(_dsdl_fingerprint([str(ns)]), str)


def test_toolchain_fingerprint_changes_on_version_bump(monkeypatch):
    """A pycyphal/nunavut/Python upgrade must shift the cache key even when the
    .dsdl sources are unchanged, so the old compiled tree is not reused."""
    pycyphal = pytest.importorskip("pycyphal")
    from nxs.transports.cyphal_source import _toolchain_fingerprint
    base = _toolchain_fingerprint()
    assert base.startswith(f"py{sys.version_info.major}.{sys.version_info.minor}")
    monkeypatch.setattr(pycyphal, "__version__", "999.999.999", raising=False)
    assert _toolchain_fingerprint() != base


def test_ensure_dsdl_raises_when_no_sources(monkeypatch):
    """No DSDL sources -> _ensure_dsdl() raises instead of caching an empty tree."""
    pytest.importorskip("pycyphal")
    import nxs.transports.cyphal_source as cs
    monkeypatch.setattr(cs, "_dsdl_roots", lambda: [])
    monkeypatch.setattr(cs, "_dsdl_ready", False)
    with pytest.raises(RuntimeError, match="no DSDL sources"):
        cs._ensure_dsdl()
