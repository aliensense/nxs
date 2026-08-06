"""`suite reset` blanks declared units; `collect-garbage` drops state
entries the manifest no longer roots."""
import os

from nxs.suite.reset import collect_garbage, reset_suite, NODE_ADDR_UNSET
from nxs.suite.schema import parse_suite_config
from nxs.suite.state import SuiteState
from nxs.tests.test_suite_reconcile import FakeUnit


def _cfg(names=('u1',)):
    return parse_suite_config({'units': [
        {'name': n, 'links': [{'transport': 'mock'}]} for n in names]})


def _state(tmp):
    return SuiteState.load(os.path.join(str(tmp), 'state.yaml'))


def test_blank_reset_stops_clears_and_forgets(tmp_path):
    state = _state(tmp_path)
    state.record('u1', serial='aa' * 12, fw_version='1.0')
    board = FakeUnit()
    board.upload_image  # noqa: B018 — presence documented by FakeUnit
    reports = reset_suite(_cfg(), state, opener=lambda k, **kw: board)
    assert reports[0].ok
    assert board.slots() == []
    assert board.read_vm_state() == 0
    assert state.unit('u1') == {}
    # Blank reset preserves identity: nothing commissioned.
    assert board.commissioned() == []


def test_factory_reset_reverts_identity_and_decimation(tmp_path):
    board = FakeUnit()
    reports = reset_suite(_cfg(), _state(tmp_path), factory=True,
                          opener=lambda k, **kw: board)
    assert reports[0].ok
    assert board.commissioned() == [NODE_ADDR_UNSET]
    assert board.read_decimation() == 1
    assert board.read_decimation(subject='temperature') == 25
    assert board.read_decimation(subject='acceleration') == 1


def test_reset_only_touches_the_selected_unit(tmp_path):
    state = _state(tmp_path)
    state.record('u1', serial='aa' * 12)
    state.record('u2', serial='bb' * 12)
    boards = {'u1': FakeUnit(), 'u2': FakeUnit()}
    calls = []

    def opener(kind, **kw):
        calls.append(kind)
        return boards['u1']

    reports = reset_suite(_cfg(('u1', 'u2')), state, only_unit='u1',
                          opener=opener)
    assert len(reports) == 1
    assert state.unit('u1') == {}
    assert state.unit('u2') != {}


def test_collect_garbage_drops_only_orphans(tmp_path):
    state = _state(tmp_path)
    state.record('u1', serial='aa' * 12)
    state.record('ghost', serial='cc' * 12)
    dropped = collect_garbage(_cfg(), state)
    assert dropped == ['ghost']
    assert state.unit('u1') != {}
    reloaded = SuiteState.load(state.path)
    assert reloaded.unit('ghost') == {}


def test_declared_empty_panel_clears_a_running_driver(tmp_path):
    from nxs.suite.reconcile import switch_suite
    from nxs.image import serialize
    from nxs.suite.reconcile import load_unit_driver

    board = FakeUnit()
    compiled = load_unit_driver('iam20680')().compile({'sample_rate': 250})
    board.upload_image(serialize(compiled))
    board.save_slot(0)
    board.vm_run()

    cfg = parse_suite_config({'units': [
        {'name': 'u1', 'links': [{'transport': 'mock'}], 'sensors': []}]})
    reports = switch_suite(cfg, _state(tmp_path),
                          opener=lambda k, **kw: board)
    assert reports[0].ok
    assert any('clear panel' in a for a in reports[0].actions)
    assert board.slots() == []
    assert board.read_driver_name() == ""


def test_unmanaged_panel_is_left_alone(tmp_path):
    from nxs.suite.reconcile import switch_suite
    from nxs.image import serialize
    from nxs.suite.reconcile import load_unit_driver

    board = FakeUnit()
    compiled = load_unit_driver('iam20680')().compile({'sample_rate': 250})
    board.upload_image(serialize(compiled))
    board.save_slot(0)

    reports = switch_suite(_cfg(), _state(tmp_path),
                          opener=lambda k, **kw: board)
    assert reports[0].ok
    assert board.slots() != []
