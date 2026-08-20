"""Suite integration of calibration: declared orientation flows through
schema → apply → drift → freeze; solved calibration surfaces as the CAL
status verdict and never enters the manifest."""
import os
import tempfile

import pytest

from nxs.client import CalibrationRecord, fnv1a32, rotation_code
from nxs.suite.drift import orientation_drift
from nxs.suite.freeze import freeze_suite
from nxs.suite.reconcile import switch_suite, load_unit_driver
from nxs.suite.schema import ManifestError, parse_suite_config
from nxs.suite.state import SuiteState
from nxs.suite.status import UnitStatus, _calib_verdict, render_status
from nxs.transports.mock import MockTransport


class FakeCalUnit(MockTransport):
    """`MockTransport` — the full reconciler surface — with an injected
    orientation, driver identity, and capability list so the panel and the
    orientation push run against a known device state."""

    def __init__(self, orientation=0, driver_name="", caps=None):
        super().__init__()
        self._calibration = CalibrationRecord(orientation=orientation)
        self._fake_driver_name = driver_name
        self._fake_caps = caps or []

    def read_driver_name(self) -> str:
        return self._fake_driver_name

    def read_capabilities(self) -> list:
        return [dict(p) for p in self._fake_caps]

    def read_store_count(self) -> int:
        return 1


def _cfg(**unit_overrides):
    unit = {'name': 'u1', 'links': [{'transport': 'mock'}], 'sensors': []}
    unit.update(unit_overrides)
    return parse_suite_config({'units': [unit]})


def test_orientation_key_parses_and_validates():
    cfg = _cfg(orientation='yaw_90')
    assert cfg.units[0].orientation == 'YAW_90'
    with pytest.raises(ManifestError):
        _cfg(orientation='YAW_45')


def test_apply_pushes_declared_orientation():
    fake = FakeCalUnit(orientation=0)
    with tempfile.TemporaryDirectory() as tmp:
        state = SuiteState.load(os.path.join(tmp, 'state.yaml'))
        reports = switch_suite(_cfg(orientation='YAW_90'), state,
                              opener=lambda k, **kw: fake)
        assert reports[0].ok
        assert any('set orientation YAW_90' in a for a in reports[0].actions)
        assert fake.read_calibration().orientation == rotation_code('YAW_90')

        # Converged: a second apply pushes nothing.
        again = switch_suite(_cfg(orientation='YAW_90'),
                            SuiteState.load(state.path),
                            opener=lambda k, **kw: fake)
        assert not any('orientation' in a for a in again[0].actions)


def test_apply_leaves_undeclared_orientation_alone():
    fake = FakeCalUnit(orientation=rotation_code('ROLL_180'))
    with tempfile.TemporaryDirectory() as tmp:
        state = SuiteState.load(os.path.join(tmp, 'state.yaml'))
        reports = switch_suite(_cfg(), state, opener=lambda k, **kw: fake)
        assert reports[0].ok
        assert fake.read_calibration().orientation == rotation_code('ROLL_180')


def test_orientation_drift_axis():
    cfg = _cfg(orientation='YAW_90')
    fake = FakeCalUnit(orientation=0)
    assert orientation_drift(cfg.units[0], fake)
    fake.set_orientation(rotation_code('YAW_90'))
    assert not orientation_drift(cfg.units[0], fake)
    assert not orientation_drift(_cfg().units[0], fake)  # undeclared


def test_freeze_adopts_hand_set_orientation():
    driver = load_unit_driver('iam20680')().compile({})
    caps = [{'name': p.name, 'current': p.current} for p in driver.params]
    fake = FakeCalUnit(orientation=rotation_code('PITCH_180'),
                       driver_name=driver.name, caps=caps)
    manifest = {'units': [{'name': 'u1', 'links': [{'transport': 'mock'}],
                           'sensors': [{'driver': 'iam20680'}]}]}
    cfg = parse_suite_config(manifest)
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'suite.yaml')
        import yaml as pyyaml
        with open(path, 'w') as f:
            pyyaml.safe_dump(manifest, f)
        reports = freeze_suite(cfg, path, opener=lambda k, **kw: fake)
        assert reports[0].ok and reports[0].changed
        assert reports[0].orientation == 'PITCH_180'
        frozen = parse_suite_config(pyyaml.safe_load(open(path)))
        assert frozen.units[0].orientation == 'PITCH_180'

def test_calib_verdict_states():
    from nxs.client import active_driver_tag
    from nxs.descriptor import driver_tag

    t = FakeCalUnit(driver_name='Iam20680')
    assert _calib_verdict(t) == '-'

    # Tagged for whatever sensor this transport actually reports.
    solved = t.read_calibration().replace_vector(
            1, (1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0), (0.01, 0.0, 0.0),
            active_driver_tag(t))
    t.write_calibration(solved)
    assert _calib_verdict(t) == 'ok'

    # A record solved for a different sensor stays guarded off.
    other = t.read_calibration().replace_vector(
            1, (1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0), (0.01, 0.0, 0.0),
            driver_tag('Someone else', 0, 0x7F))
    t.write_calibration(other)
    assert _calib_verdict(t) == 'STALE'


def test_calib_verdict_separates_unguarded_from_bound():
    """An applied record carrying no identity is not a verified one. Folding
    both into `ok` hid the difference the tag exists to express."""
    t = FakeCalUnit(driver_name='Iam20680')
    untagged = t.read_calibration().replace_vector(
            1, (1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0), (0.01, 0.0, 0.0), 0)
    t.write_calibration(untagged)
    assert _calib_verdict(t) == 'unguarded'


def test_status_renders_cal_column():
    row = UnitStatus(name='u1', link='mock', up=True, driver='Iam20680',
                     vm_state='running', fw_version='1.0', serial_ok='ok',
                     drift='-', cal='STALE', samples=42)
    table = render_status([row])
    assert 'CAL' in table.splitlines()[0]
    assert 'STALE' in table


def test_a_parked_runner_is_driver_drift_and_shows_in_the_vm_column():
    """The runner parks in PROBE_FAILED until a host command intervenes, so
    the driver name keeps matching the manifest and every later switch
    reported `converged` over a sensor that never answered — while the table
    showed VM `running`, because the VM does keep executing."""
    from nxs._generated_constants import RunnerStates
    from nxs.suite.status import _vm_verdict

    class Running(MockTransport):
        def read_vm_state(self):
            return 1

    class Parked(Running):
        def read_runner_state(self):
            return RunnerStates.RunnerState.PROBE_FAILED

    assert _vm_verdict(Parked()) == 'no-probe'
    assert _vm_verdict(Running()) == 'running'
    # An idle VM keeps its own verdict — the runner never overrides it.
    assert _vm_verdict(MockTransport()) == 'idle'
