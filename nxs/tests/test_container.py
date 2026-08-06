"""Tests for the container entry point — the wire-blind run loop and sink."""

import argparse

import pytest

from nxs import container
from nxs.client import Sample


class _FakeSource:
    def __init__(self, samples):
        self._samples = samples
        self.closed = False

    def iter_samples(self, timeout=1.0):
        yield from self._samples

    def close(self):
        self.closed = True


def test_run_streams_every_sample_and_closes():
    samples = [Sample(count=i, raw=b"", values={"x": float(i)}) for i in range(5)]
    src = _FakeSource(samples)
    got = []
    container.run(src, got.append)
    assert [s.count for s in got] == [0, 1, 2, 3, 4]
    assert src.closed  # run() closes the source on exit


def test_run_respects_count():
    src = _FakeSource([Sample(count=i, raw=b"", values={}) for i in range(10)])
    got = []
    container.run(src, got.append, count=3)
    assert len(got) == 3
    assert src.closed


def test_print_sink(capsys):
    container.print_sink(Sample(count=7, raw=b"", values={"accel_x": 1.5, "label": "ok"}))
    out = capsys.readouterr().out
    assert "[7]" in out
    assert "accel_x=1.5000" in out  # floats fixed-width
    assert "label=ok" in out        # strings verbatim


def test_json_sink(capsys):
    import json
    container.json_sink(Sample(count=42, raw=b"", values={"accel_x": 1.5, "label": "ok"},
                               timestamp_us=1000))
    obj = json.loads(capsys.readouterr().out.strip())
    assert obj == {"seq": 42, "t_us": 1000, "accel_x": 1.5, "label": "ok"}


def test_cyphal_can_is_guarded():
    with pytest.raises(SystemExit):
        container.make_source(argparse.Namespace(transport="cyphal-can"))
