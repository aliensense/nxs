"""Generations: a fully-converged switch records the manifest in the
tool-managed repo; snapshot tags, rollback restores, and neither ever
touches state.yaml (trust is not intent)."""
import os

import pytest

from nxs.suite.generations import (
    GenerationsError, list_generations, record_generation, rollback,
    snapshot)


def _paths(tmp):
    config = os.path.join(str(tmp), 'config', 'suite.yaml')
    state = os.path.join(str(tmp), 'state', 'state.yaml')
    os.makedirs(os.path.dirname(config))
    os.makedirs(os.path.dirname(state))
    return config, state


def _write(config, text):
    with open(config, 'w') as f:
        f.write(text)


def test_record_snapshot_list_rollback_round_trip(tmp_path):
    config, state = _paths(tmp_path)
    _write(config, "units: []\n# generation one\n")
    sha1 = record_generation(config, state, "1/1 converged")
    assert sha1

    snapshot(state, 'good')

    _write(config, "units: []\n# generation two\n")
    sha2 = record_generation(config, state, "1/1 converged")
    assert sha2 and sha2 != sha1

    rows = list_generations(state)
    assert [r[0] for r in rows] == [sha2, sha1]
    assert rows[1][2] == 'good'
    assert rows[0][3] == '1/1 converged'

    rollback(state, 'good', config)
    assert open(config).read() == "units: []\n# generation one\n"


def test_identical_manifest_records_no_generation(tmp_path):
    config, state = _paths(tmp_path)
    _write(config, "units: []\n")
    assert record_generation(config, state, "1/1 converged")
    assert record_generation(config, state, "1/1 converged") is None
    assert len(list_generations(state)) == 1


def test_rollback_unknown_ref_is_a_clean_error(tmp_path):
    config, state = _paths(tmp_path)
    _write(config, "units: []\n")
    record_generation(config, state, "1/1 converged")
    with pytest.raises(GenerationsError, match='no generation'):
        rollback(state, 'ghost', config)


def test_verbs_before_any_generation_explain_themselves(tmp_path):
    config, state = _paths(tmp_path)
    with pytest.raises(GenerationsError, match='no generations recorded'):
        snapshot(state, 'good')


def test_missing_git_degrades_with_a_clear_error(tmp_path, monkeypatch):
    import subprocess

    config, state = _paths(tmp_path)
    _write(config, "units: []\n")

    def _no_git(*args, **kwargs):
        raise FileNotFoundError('git')

    monkeypatch.setattr(subprocess, 'run', _no_git)
    with pytest.raises(GenerationsError, match='git is not installed'):
        record_generation(config, state, "1/1 converged")


def test_converged_apply_records_and_partial_does_not(tmp_path, monkeypatch, capsys):
    """CLI-level: the auto-generation fires only when every unit
    converged."""
    import argparse

    from nxs.suite import cli as suite_cli
    from nxs.suite.reconcile import UnitReport

    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'xdg'))
    monkeypatch.setenv('XDG_DATA_HOME', str(tmp_path / 'xdg-data'))
    config_dir = tmp_path / 'xdg' / 'aliensense'
    config_dir.mkdir(parents=True)
    (config_dir / 'suite.yaml').write_text(
        "units:\n"
        "  - name: u1\n"
        "    module: nxs\n"
        "    links: [{transport: mock}]\n")

    def _args(**overrides):
        defaults = {'config': None, 'suite_cmd': 'switch', 'init': False,
                    'diff': False, 'dry_run': False, 'only_unit': None,
                    'accept_new_serial': False}
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    ok = [UnitReport(name='u1', link='mock', ok=True, actions=['converged'])]
    monkeypatch.setattr('nxs.suite.reconcile.switch_suite',
                        lambda *a, **kw: ok)
    rc = suite_cli.cmd_suite(_args())
    out = capsys.readouterr().out
    assert rc == 0
    assert 'generation' in out and 'recorded' in out

    bad = [UnitReport(name='u1', link='mock', ok=False, error='boom')]
    monkeypatch.setattr('nxs.suite.reconcile.switch_suite',
                        lambda *a, **kw: bad)
    (config_dir / 'suite.yaml').write_text("units:\n  - name: u1\n"
                                           "    module: nxs\n"
                                           "    links: [{transport: mock}]\n"
                                           "    # edited\n")
    rc = suite_cli.cmd_suite(_args())
    assert rc == 1
    from nxs.suite import default_state_path
    assert len(list_generations(default_state_path())) == 1


def test_snapshot_duplicate_label_names_the_reason(tmp_path):
    """git reports tag collisions on stderr; the error must carry it."""
    config, state = _paths(tmp_path)
    _write(config, "units: []\n")
    record_generation(config, state, "1/1 converged")
    snapshot(state, 'good')
    with pytest.raises(GenerationsError, match='already exists'):
        snapshot(state, 'good')
