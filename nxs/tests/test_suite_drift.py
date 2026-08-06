"""Tests for drift detection — the one comparison apply repairs with,
freeze adopts with, and status displays."""
import os
import tempfile

from nxs.suite.drift import detect_unit_drift
from nxs.suite.reconcile import switch_suite, load_unit_driver, panel_hash
from nxs.suite.schema import parse_suite_config
from nxs.suite.state import SuiteState
from nxs.tests.test_suite_reconcile import FakeUnit, _cfg


def _deployed_fake(tmp):
    """A fake with the manifest panel applied — the converged baseline."""
    fake = FakeUnit()
    state = SuiteState.load(os.path.join(tmp, 'state.yaml'))
    reports = switch_suite(_cfg(), state, opener=lambda k, **kw: fake)
    assert reports[0].ok
    return fake, SuiteState.load(state.path)


def _panel(cfg):
    unit = cfg.units[0]
    return unit, [(s, load_unit_driver(s.driver)().compile(s.config))
                  for s in unit.sensors], panel_hash(unit)


def test_converged_unit_has_no_drift():
    with tempfile.TemporaryDirectory() as tmp:
        fake, state = _deployed_fake(tmp)
        unit, panel, digest = _panel(_cfg())
        drift = detect_unit_drift(unit, panel, fake, state, digest)
        assert not drift.any()


def test_manual_set_param_is_config_drift():
    with tempfile.TemporaryDirectory() as tmp:
        fake, state = _deployed_fake(tmp)
        fake.set_param('sample_rate', 500)
        unit, panel, digest = _panel(_cfg())
        drift = detect_unit_drift(unit, panel, fake, state, digest)
        assert drift.params == {'sample_rate': (250, 500)}
        assert drift.kinds() == ['config']


def test_wrong_active_driver_is_driver_drift():
    with tempfile.TemporaryDirectory() as tmp:
        fake, state = _deployed_fake(tmp)
        cfg = parse_suite_config({'units': [
            {'name': 'u1', 'links': [{'transport': 'mock'}],
             'sensors': [{'driver': 'ms5611'}]}]})
        unit, panel, digest = _panel(cfg)
        drift = detect_unit_drift(unit, panel, fake, state, digest)
        assert drift.driver and drift.kinds() == ['driver']


def test_store_shape_change_is_shape_drift():
    with tempfile.TemporaryDirectory() as tmp:
        fake, state = _deployed_fake(tmp)
        fake.save_slot(1)  # a manually appended slot
        unit, panel, digest = _panel(_cfg())
        drift = detect_unit_drift(unit, panel, fake, state, digest)
        assert drift.shape


def test_firmware_pin_drift_uses_wire_then_state():
    with tempfile.TemporaryDirectory() as tmp:
        fake, state = _deployed_fake(tmp)
        unit, panel, digest = _panel(parse_suite_config({'units': [
            {'name': 'u1', 'links': [{'transport': 'mock'}], 'firmware': '1.1.0',
             'sensors': [{'driver': 'iam20680',
                          'config': {'sample_rate': 250}}]}]}))
        # Provable wire mismatch (fake reports 1.0).
        assert detect_unit_drift(unit, panel, fake, state, digest).fw
        # A proven major.minor match with no record is converged — no
        # disruptive reflash to "confirm" an unprovable patch level.
        assert not detect_unit_drift(unit, panel, FakeUnit(fw='1.1'),
                                     state, digest).fw
        # A record of a different patch is a bump — drift, even though the
        # wire's major.minor still matches.
        state.record('u1', fw_version='1.1.5')
        assert detect_unit_drift(unit, panel, FakeUnit(fw='1.1'),
                                 state, digest).fw
        # The record matching the pin is converged.
        state.record('u1', fw_version='1.1.0')
        assert not detect_unit_drift(unit, panel, FakeUnit(fw='1.1'),
                                     state, digest).fw
        # A re-spelled pin (1.1.0 vs 1.1) is the same version, not drift.
        respelled = parse_suite_config({'units': [
            {'name': 'u1', 'links': [{'transport': 'mock'}], 'firmware': '1.1',
             'sensors': [{'driver': 'iam20680',
                          'config': {'sample_rate': 250}}]}]})
        ru, rpanel, rdig = _panel(respelled)
        assert not detect_unit_drift(ru, rpanel, FakeUnit(fw='1.1'),
                                     state, rdig).fw
        # No wire version and no record (I2C first switch): flash once.
        fresh = SuiteState('/nonexistent/state.yaml')
        assert detect_unit_drift(unit, panel, FakeUnit(fw=None),
                                 fresh, digest).fw
