"""Tests for the suite state file — the tool-owned side of the
intent/state split."""
import os

from nxs.suite.state import SuiteState


def test_load_missing_file_is_empty():
    state = SuiteState.load('/nonexistent/state.yaml')
    assert state.unit('anything') == {}


def test_save_and_reload_round_trip(tmp_path):
    path = str(tmp_path / 'nested' / 'state.yaml')
    state = SuiteState.load(path)
    state.record('imu-mast', serial='2f004b0032510f0011223344')
    state.save()
    reloaded = SuiteState.load(path)
    assert reloaded.unit('imu-mast')['serial'] == '2f004b0032510f0011223344'
    assert 'applied_at' in reloaded.unit('imu-mast')


def test_save_with_bare_filename(tmp_path, monkeypatch):
    """A path with no directory component must not crash makedirs."""
    monkeypatch.chdir(tmp_path)
    state = SuiteState.load('state.yaml')
    state.record('u1', serial='2f004b0032510f0011223344')
    state.save()
    assert os.path.exists(str(tmp_path / 'state.yaml'))


def test_save_is_a_noop_until_something_is_recorded(tmp_path):
    """A converged switch must leave the state file untouched."""
    path = str(tmp_path / 'state.yaml')
    SuiteState.load(path).save()
    assert not os.path.exists(path)

    state = SuiteState.load(path)
    state.record('u1', panel_hash='abc')
    state.save()
    before = open(path, 'rb').read()
    reloaded = SuiteState.load(path)
    reloaded.save()  # nothing recorded on this instance
    assert open(path, 'rb').read() == before


def test_load_survives_unparseable_yaml(tmp_path):
    path = str(tmp_path / 'state.yaml')
    with open(path, 'w') as f:
        f.write("units: {u1: [unclosed\n")
    state = SuiteState.load(path)
    assert state.unit('u1') == {}


def test_load_survives_a_non_mapping_file(tmp_path):
    path = str(tmp_path / 'state.yaml')
    with open(path, 'w') as f:
        f.write("- just\n- a\n- list\n")
    state = SuiteState.load(path)
    assert state.unit('u1') == {}


def test_load_survives_an_unreadable_file(tmp_path, monkeypatch):
    path = str(tmp_path / 'state.yaml')
    open(path, 'w').close()

    def _denied(*a, **k):
        raise PermissionError("permission denied")

    monkeypatch.setattr('builtins.open', _denied)
    assert SuiteState.load(path).unit('u1') == {}


def test_load_survives_a_binary_corrupt_file(tmp_path):
    path = str(tmp_path / 'state.yaml')
    with open(path, 'wb') as f:
        f.write(b'\xff\xfe\x00\x01 not utf-8')
    state = SuiteState.load(path)
    assert state.unit('u1') == {}


def test_load_drops_malformed_unit_entries(tmp_path):
    path = str(tmp_path / 'state.yaml')
    with open(path, 'w') as f:
        f.write("units:\n  u1: oops\n  u2: {serial: 2f004b0032510f0011223344}\n")
    state = SuiteState.load(path)
    assert state.unit('u1') == {}
    assert state.unit('u2')['serial'] == '2f004b0032510f0011223344'


def test_unit_returns_a_copy_not_the_record(tmp_path):
    """Mutating the returned dict must not alter state — writes go
    through record(), which is what sets the dirty flag that gates
    save()."""
    path = str(tmp_path / 'state.yaml')
    state = SuiteState(path)
    state.record('u1', serial='2f004b0032510f0011223344')

    state.unit('u1')['serial'] = 'tampered'
    assert state.unit('u1')['serial'] == '2f004b0032510f0011223344'
