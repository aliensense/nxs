"""The manifest's `egress:` section declares decimation intent: apply
converges and persists it, drift names it, freeze adopts live factors
back into a declared section."""
import os

from nxs.suite.drift import egress_drift
from nxs.suite.reconcile import switch_suite
from nxs.suite.schema import ManifestError, parse_suite_config
from nxs.suite.state import SuiteState
from nxs.tests.test_suite_reconcile import FakeUnit

import pytest


def _cfg(egress):
    return parse_suite_config({'units': [
        {'name': 'u1', 'links': [{'transport': 'mock'}],
         'egress': egress}]})


def _state(tmp):
    return SuiteState.load(os.path.join(str(tmp), 'state.yaml'))


def test_declared_factors_converge_and_persist(tmp_path):
    board = FakeUnit()
    board.write_decimation(4)
    board.write_decimation(1, subject='temperature')

    cfg = _cfg({'decimation': 1, 'subjects': {'temperature': 25}})
    reports = switch_suite(cfg, _state(tmp_path),
                          opener=lambda k, **kw: board)
    assert reports[0].ok
    assert any('retune decimation 4→1' in a for a in reports[0].actions)
    assert any('retune decimation[temperature] 1→25' in a
               for a in reports[0].actions)
    assert board.read_decimation() == 1
    assert board.read_decimation(subject='temperature') == 25
    # The converge ends with the persist Save (bare commission()).
    assert board.commissioned() == [None]


def test_matching_factors_are_not_rewritten(tmp_path):
    board = FakeUnit()
    board.write_decimation(1)
    board.write_decimation(25, subject='temperature')

    cfg = _cfg({'decimation': 1, 'subjects': {'temperature': 25}})
    reports = switch_suite(cfg, _state(tmp_path),
                          opener=lambda k, **kw: board)
    assert reports[0].ok
    assert not any('retune' in a for a in reports[0].actions)
    assert board.commissioned() == []


def test_absent_section_is_unmanaged(tmp_path):
    board = FakeUnit()
    board.write_decimation(7)
    cfg = parse_suite_config({'units': [
        {'name': 'u1', 'links': [{'transport': 'mock'}]}]})
    reports = switch_suite(cfg, _state(tmp_path),
                          opener=lambda k, **kw: board)
    assert reports[0].ok
    assert board.read_decimation() == 7


def test_egress_drift_names_the_factors():
    board = FakeUnit()
    board.write_decimation(4)
    cfg = _cfg({'decimation': 1, 'subjects': {'temperature': 25}})
    drift = egress_drift(cfg.units[0], board)
    assert drift['device'] == (1, 4)
    assert drift['temperature'][0] == 25


def test_unknown_subject_is_rejected():
    with pytest.raises(ManifestError, match='unknown subject'):
        _cfg({'subjects': {'gravity': 2}})


def test_freeze_adopts_live_factors_into_a_declared_section(tmp_path):
    from nxs.suite.freeze import freeze_suite
    from nxs.suite.schema import load_suite_config

    path = os.path.join(str(tmp_path), 'suite.yaml')
    with open(path, 'w') as f:
        f.write("units:\n"
                "  - name: u1\n"
                "    module: nxs\n"
                "    links: [{transport: mock}]\n"
                "    sensors:\n"
                "      - driver: iam20680\n"
                "        config: {sample_rate: 250}\n"
                "    egress:\n"
                "      decimation: 1\n"
                "      subjects: {temperature: 25}\n")
    cfg = load_suite_config(path)
    board = FakeUnit()
    switch_suite(cfg, _state(tmp_path), opener=lambda k, **kw: board)
    # Field tuning moves the factors past the manifest.
    board.write_decimation(5)
    board.write_decimation(40, subject='temperature')

    reports = freeze_suite(cfg, path, only_unit='u1',
                           opener=lambda k, **kw: board)
    assert reports[0].ok and reports[0].changed
    assert any('egress decimation: 1→5' in a for a in reports[0].actions)
    assert any('egress[temperature]: 25→40' in a for a in reports[0].actions)
    frozen = load_suite_config(path)
    assert frozen.units[0].egress.decimation == 5
    assert frozen.units[0].egress.subjects['temperature'] == 40
