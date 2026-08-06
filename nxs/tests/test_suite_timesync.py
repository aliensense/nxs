"""`suite timesync` — the resident pusher's contract:

- every selected unit gets a push per round, with per-unit isolation;
- --unit filters to a subset and rejects unknown names;
- a dead unit reports without blocking the rest and is reopened once
  its link answers again;
- a transport without the time surface is noted, never crashed on.
"""
import pytest

from nxs.suite.schema import parse_suite_config
from nxs.suite.timesync import SuitePusher
from nxs.transports.mock import MockTransport


def _cfg(names):
    return parse_suite_config({'units': [
        {'name': name, 'links': [{'transport': 'mock'}]} for name in names]})


class _DeadThenAlive(MockTransport):
    """Refuses the probe for a set number of attempts, then answers."""

    def __init__(self, dead_probes):
        super().__init__()
        self._dead_probes = dead_probes

    def probe(self):
        if self._dead_probes > 0:
            self._dead_probes -= 1
            return False
        return True


class _NoTimeSurface:
    def probe(self):
        return True

    def close(self):
        pass


def test_round_pushes_every_declared_unit():
    fakes = {'u1': MockTransport(), 'u2': MockTransport()}
    pusher = SuitePusher(_cfg(['u1', 'u2']),
                         opener=lambda kind, **kw: fakes.popitem()[1])
    reports = pusher.round()
    assert [r.name for r in reports] == ['u1', 'u2']
    assert all(r.bound_us is not None for r in reports)


def test_unit_filter_selects_subset():
    pusher = SuitePusher(_cfg(['u1', 'u2', 'u3']), only_units=['u2', 'u3'],
                         opener=lambda kind, **kw: MockTransport())
    assert [r.name for r in pusher.round()] == ['u2', 'u3']


def test_unknown_unit_name_is_refused():
    with pytest.raises(ValueError):
        SuitePusher(_cfg(['u1']), only_units=['ghost'])


def test_dead_unit_reports_and_recovers_next_round():
    fake = _DeadThenAlive(dead_probes=1)
    pusher = SuitePusher(_cfg(['u1']), opener=lambda kind, **kw: fake)
    first = pusher.round()[0]
    assert first.bound_us is None and first.note == "no link answered"
    second = pusher.round()[0]
    assert second.bound_us is not None


def test_transport_without_time_surface_is_noted():
    pusher = SuitePusher(_cfg(['u1']),
                         opener=lambda kind, **kw: _NoTimeSurface())
    report = pusher.round()[0]
    assert report.bound_us is None and report.note == "no time surface"


def test_nxs_path_prefers_the_path_resolution():
    from nxs.suite.timesync import resolve_nxs_path

    assert resolve_nxs_path('nxs', which=lambda n: '/opt/bin/nxs') \
        == '/opt/bin/nxs'


def test_nxs_path_falls_back_to_the_script_argument():
    from nxs.suite.timesync import resolve_nxs_path
    import os

    got = resolve_nxs_path('bin/nxs', which=lambda n: None)
    assert os.path.isabs(got) and got.endswith('/bin/nxs')


def test_systemd_unit_carries_path_user_and_defaults():
    from nxs.suite.timesync import render_systemd_unit

    unit = render_systemd_unit('/home/rig/.local/bin/nxs', 'rig')
    assert "ExecStart=/home/rig/.local/bin/nxs suite timesync\n" in unit
    assert "User=rig\n" in unit
    assert "WantedBy=multi-user.target" in unit


def test_systemd_unit_carries_subset_and_interval():
    from nxs.suite.timesync import render_systemd_unit

    unit = render_systemd_unit('/usr/bin/nxs', 'rig',
                               only_units=['u1', 'u2'], interval=5.0)
    assert ("ExecStart=/usr/bin/nxs suite timesync "
            "--unit u1 --unit u2 --interval 5\n") in unit


def test_failed_push_drops_the_transport_for_reopen():
    opened = []

    def opener(kind, **kw):
        opened.append(MockTransport())
        return opened[-1]

    def flaky_pusher(transport):
        if len(opened) == 1:
            raise OSError("bus glitch")
        return 150

    pusher = SuitePusher(_cfg(['u1']), opener=opener, pusher=flaky_pusher)
    first = pusher.round()[0]
    assert first.bound_us is None and "push failed" in first.note
    second = pusher.round()[0]
    assert second.bound_us == 150
    assert len(opened) == 2


def test_reopened_transport_inherits_the_estimator():
    """A link flap must not reset the drift fit: the reopened transport
    carries the unit's estimator, not a fresh one pushing rate 0."""
    opened = []
    estimators = []

    def opener(kind, **kw):
        opened.append(MockTransport())
        return opened[-1]

    def flaky_pusher(transport):
        estimators.append(transport.get_time_sync())
        if len(estimators) == 1:
            raise OSError("bus glitch")
        return 150

    pusher = SuitePusher(_cfg(['u1']), opener=opener, pusher=flaky_pusher)
    pusher.round()
    pusher.round()
    assert len(opened) == 2
    assert estimators[0] is estimators[1]
