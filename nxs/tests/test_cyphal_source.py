"""Offline tests for CyphalSampleSource.

The pycyphal layer is bypassed (`autostart=False`) and GetOutputInfo responses
plus raw samples are injected directly, so the descriptor walk and the
`parse_sample` decode run without a bus or pycyphal installed.
"""

import os
import struct
import types

import pytest

from nxs.transports.cyphal_source import CyphalSampleSource, _dsdl_roots


def _resp(num_outputs, name, field_type, byte_order, scale, offset, unit,
          count=0, semantic=0):
    """A stand-in GetOutputInfo response carrying the DSDL field surface."""
    return types.SimpleNamespace(
        num_outputs=num_outputs, field_type=field_type, byte_order=byte_order,
        semantic=semantic, count=count, scale=scale, offset=offset,
        name=name.encode(), unit=unit.encode(),
    )


class _FakeSource(CyphalSampleSource):
    """CyphalSampleSource with the bus replaced by canned descriptors."""

    def __init__(self, descriptors):
        super().__init__(port="(fake)", autostart=False)
        self._descs = descriptors

    def _fetch_descriptor(self, index):
        if not self._descs:
            return None  # no device present (probe / timeout path)
        if index < len(self._descs):
            return self._descs[index]
        # A present device answers an out-of-range index with num_outputs and
        # an empty name (per the GetOutputInfo contract), never None.
        return _resp(self._descs[0].num_outputs, "", 0, 0, 0.0, 0.0, "")


def test_descriptors_built_from_rpc():
    s = _FakeSource([
        _resp(2, "accel_x", 2, 1, 0.5, -1.0, "m/s^2"),
        _resp(2, "label", 8, 0, 1.0, 0.0, "", count=4),
    ])
    d = s.descriptors()
    assert d[0] == {
        "idx": 0, "name": "accel_x", "type": "int16", "byte_order": "little",
        "semantic": 0, "count": 0, "scale": 0.5, "offset": -1.0, "unit": "m/s^2",
    }
    assert d[1]["type"] == "string"
    assert d[1]["count"] == 4
    assert s.descriptors() is d  # cached on the second call


def test_descriptors_stop_at_num_outputs():
    # num_outputs is 1, but the fake would serve a second — the walk must
    # honor the count and stop at one.
    s = _FakeSource([
        _resp(1, "only", 6, 1, 1.0, 0.0, "g"),
        _resp(1, "extra", 2, 1, 1.0, 0.0, ""),
    ])
    assert len(s.descriptors()) == 1


def test_descriptors_not_cached_on_rpc_failure():
    # A timed-out walk (fetch returns None) must not be cached, or one dropped
    # startup RPC would wedge decoding for the process lifetime.
    s = _FakeSource([_resp(1, "accel_x", 2, 1, 0.5, 0.0, "m/s^2")])
    s._fetch_descriptor = lambda index: None  # device unresponsive
    assert s.descriptors() == []
    assert s._descriptors is None  # transient failure left uncached
    del s._fetch_descriptor  # device recovers; restore the faithful fake
    assert [d["name"] for d in s.descriptors()] == ["accel_x"]
    assert s._descriptors is not None


def test_next_sample_decodes():
    s = _FakeSource([_resp(1, "accel_x", 2, 1, 0.5, 0.0, "m/s^2")])  # int16 LE, x0.5
    s._queue.put((7, struct.pack("<h", 123), 9000))
    sample = s.next_sample(timeout=0.1)
    assert sample.count == 7
    assert sample.values["accel_x"] == pytest.approx(123 * 0.5)
    assert sample.timestamp_us == 9000


def test_iter_samples_decodes_stream():
    s = _FakeSource([_resp(1, "accel_x", 2, 1, 0.5, 0.0, "m/s^2")])
    for seq in range(1, 4):
        s._queue.put((seq, struct.pack("<h", 100 + seq), seq * 1000))

    out = []
    for sample in s.iter_samples(timeout=0.1):
        out.append(sample)
        if len(out) >= 3:
            break

    assert len(out) == 3
    for i, sample in enumerate(out, 1):
        assert sample.values["accel_x"] == pytest.approx((100 + i) * 0.5)
        assert sample.timestamp_us == i * 1000


def test_next_sample_none_on_empty():
    s = _FakeSource([_resp(1, "x", 2, 1, 1.0, 0.0, "")])
    assert s.next_sample(timeout=0.01) is None


def test_probe_false_without_device():
    assert _FakeSource([]).probe() is False


def test_ensure_dsdl_compiles_vendored(tmp_path, monkeypatch):
    """The vendored DSDL compiles self-contained — no repo, no PRDT, no
    CYPHAL_PATH — including the fixed-port vendor service. Guards the
    `allow_unregulated_fixed_port_id` flag the offline tests can't reach."""
    pytest.importorskip("pycyphal")
    import os
    import nxs.transports.cyphal_source as cs
    if not os.path.isdir(os.path.join(_dsdl_roots()[-1], "aliensense")):
        pytest.skip("vendored DSDL absent — run scripts/vendor-dsdl.sh")
    monkeypatch.setattr(cs, "_dsdl_ready", False)
    monkeypatch.delenv("CYPHAL_PATH", raising=False)  # force the in-wheel copy
    monkeypatch.setenv("HOME", str(tmp_path))          # compile into a temp cache
    cs._ensure_dsdl()
    import aliensense.nxs.GetOutputInfo_1_0  # noqa: F401 — fixed-port vendor service
    import aliensense.nxs.RawSample_0_1      # noqa: F401
    import uavcan.register.Access_1_0         # noqa: F401 — the arming service


def test_dsdl_roots_bundled_only_when_cyphal_path_unset(monkeypatch):
    monkeypatch.delenv("CYPHAL_PATH", raising=False)
    roots = _dsdl_roots()
    assert len(roots) == 1
    assert roots[0].endswith("dsdl")


def test_dsdl_roots_appends_bundled_fallback_after_cyphal_path(monkeypatch):
    # A CYPHAL_PATH set for yakut takes precedence but must not shadow the
    # bundled aliensense: the bundled dir is always a trailing fallback.
    monkeypatch.setenv("CYPHAL_PATH", "/x" + os.pathsep + "/y")
    roots = _dsdl_roots()
    assert roots[:2] == ["/x", "/y"]
    assert roots[-1].endswith("dsdl")
