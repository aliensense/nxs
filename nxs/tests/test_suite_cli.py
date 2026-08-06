"""Tests for the `nxs suite` command dispatch — scan must bootstrap and
repair manifests, so a broken suite.yaml blocks apply/status/diff but
never a plain scan."""
import argparse
import os

from nxs.suite import cli as suite_cli


def _args(tmp_path, suite_cmd, **extra):
    defaults = {'config': str(tmp_path / 'suite.yaml'), 'suite_cmd': suite_cmd,
                'init': False, 'diff': False, 'dry_run': False, 'only_unit': None,
                'pin_firmware': False, 'accept_new_serial': False}
    defaults.update(extra)
    return argparse.Namespace(**defaults)


def _write_broken_manifest(tmp_path):
    with open(tmp_path / 'suite.yaml', 'w') as f:
        f.write("units: []\n")  # valid YAML, fails validation (empty units)


def test_plain_scan_proceeds_past_an_invalid_manifest(tmp_path, monkeypatch, capsys):
    _write_broken_manifest(tmp_path)
    monkeypatch.setattr('nxs.suite.scan.scan_suite', lambda cfg=None, **kw: [])
    rc = suite_cli.cmd_suite(_args(tmp_path, 'scan'))
    captured = capsys.readouterr()
    assert rc == 0
    assert 'ignoring invalid manifest' in captured.err


def test_init_refuses_an_invalid_manifest(tmp_path, monkeypatch, capsys):
    """--init edits the file in place; merging over a manifest that
    cannot be parsed would risk hand-written content."""
    _write_broken_manifest(tmp_path)
    monkeypatch.setattr('nxs.suite.scan.scan_suite', lambda cfg=None, **kw: [])
    before = (tmp_path / 'suite.yaml').read_text()
    rc = suite_cli.cmd_suite(_args(tmp_path, 'scan', init=True))
    assert rc == 1
    assert 'cannot merge' in capsys.readouterr().err
    assert (tmp_path / 'suite.yaml').read_text() == before


def test_scan_stays_silent_on_an_empty_manifest(tmp_path, monkeypatch, capsys):
    """A zero-byte manifest (a crashed editor, an interrupted
    redirect) is "no manifest yet", not corruption worth a warning."""
    (tmp_path / 'suite.yaml').touch()
    monkeypatch.setattr('nxs.suite.scan.scan_suite', lambda cfg=None, **kw: [])
    rc = suite_cli.cmd_suite(_args(tmp_path, 'scan', init=True))
    captured = capsys.readouterr()
    assert rc == 0
    assert 'ignoring' not in captured.err


def _found(serial='2f004b0032510f0011223344', bus='/dev/i2c-9', addr=0x30):
    from nxs.suite.scan import Found
    from nxs.suite.schema import LinkSpec

    return Found(link=LinkSpec(transport='i2c', bus=bus, address=addr),
                 serial=serial)


def test_init_creates_the_manifest_in_place(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr('nxs.suite.scan.scan_suite',
                        lambda cfg=None, **kw: [_found()])
    rc = suite_cli.cmd_suite(_args(tmp_path, 'scan', init=True))
    assert rc == 0
    assert 'created' in capsys.readouterr().out
    text = (tmp_path / 'suite.yaml').read_text()
    assert 'serial: "2f004b0032510f0011223344"' in text


def test_init_rerun_is_idempotent_and_preserves_hand_edits(
        tmp_path, monkeypatch, capsys):
    """A re-scan on a covered bench changes nothing: hand-written
    names, sensors, and comments survive byte-for-byte."""
    (tmp_path / 'suite.yaml').write_text(
        "# bench notes survive\n"
        "units:\n"
        "  - name: imu-mast   # renamed by hand\n"
        "    module: nxs\n"
        "    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x30}]\n"
        "    serial: \"2f004b0032510f0011223344\"\n"
        "    sensors: [{driver: iam20680, config: {sample_rate: 250}}]\n")
    monkeypatch.setattr('nxs.suite.scan.scan_suite',
                        lambda cfg=None, **kw: [_found()])
    before = (tmp_path / 'suite.yaml').read_text()
    rc = suite_cli.cmd_suite(_args(tmp_path, 'scan', init=True))
    out = capsys.readouterr().out
    assert rc == 0
    assert 'kept imu-mast' in out
    assert '1 unchanged' in out
    assert (tmp_path / 'suite.yaml').read_text() == before


def test_init_appends_a_new_board_below_hand_edits(
        tmp_path, monkeypatch, capsys):
    (tmp_path / 'suite.yaml').write_text(
        "units:\n"
        "  - name: imu-mast\n"
        "    module: nxs\n"
        "    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x30}]\n"
        "    serial: \"2f004b0032510f0011223344\"\n")
    hits = [_found(),
            _found(serial='aa004b0032510f0011223344', bus='/dev/i2c-11')]
    monkeypatch.setattr('nxs.suite.scan.scan_suite',
                        lambda cfg=None, **kw: hits)
    rc = suite_cli.cmd_suite(_args(tmp_path, 'scan', init=True))
    out = capsys.readouterr().out
    assert rc == 0
    assert 'added unit-i2c-11-30' in out
    text = (tmp_path / 'suite.yaml').read_text()
    assert text.index('imu-mast') < text.index('unit-i2c-11-30')
    from nxs.suite.schema import load_suite_config
    cfg = load_suite_config(str(tmp_path / 'suite.yaml'))
    assert [u.name for u in cfg.units] == ['imu-mast', 'unit-i2c-11-30']


def test_init_appends_a_new_link_at_the_end_of_links(
        tmp_path, monkeypatch, capsys):
    """A recognized board on a new route grows its links list at the
    END — hand-ordered management priority survives."""
    from nxs.suite.scan import Found
    from nxs.suite.schema import LinkSpec, load_suite_config

    (tmp_path / 'suite.yaml').write_text(
        "units:\n"
        "  - name: imu-mast   # comment survives\n"
        "    module: nxs\n"
        "    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x30}]\n"
        "    serial: \"2f004b0032510f0011223344\"\n")
    hits = [_found(),
            Found(link=LinkSpec(transport='cyphal-can', iface='can1',
                                node_id=125),
                  serial='2f004b0032510f0011223344')]
    monkeypatch.setattr('nxs.suite.scan.scan_suite',
                        lambda cfg=None, **kw: hits)
    rc = suite_cli.cmd_suite(_args(tmp_path, 'scan', init=True))
    out = capsys.readouterr().out
    assert rc == 0
    assert 'kept imu-mast (new link: can can1 node 125)' in out
    text = (tmp_path / 'suite.yaml').read_text()
    assert '# comment survives' in text
    cfg = load_suite_config(str(tmp_path / 'suite.yaml'))
    assert [l.transport for l in cfg.units[0].links] == ['i2c', 'cyphal-can']


def test_apply_refuses_an_invalid_manifest(tmp_path, capsys):
    _write_broken_manifest(tmp_path)
    rc = suite_cli.cmd_suite(_args(tmp_path, 'switch'))
    assert rc == 1
    assert "non-empty 'units'" in capsys.readouterr().err


def test_unreadable_manifest_fails_cleanly(tmp_path, monkeypatch, capsys):
    """An existing-but-unreadable manifest is a clean error, not a
    traceback (both suite commands and --unit)."""
    (tmp_path / 'suite.yaml').write_text("units: []\n")

    def _denied(*a, **k):
        raise PermissionError("permission denied")

    monkeypatch.setattr('nxs.suite.cli.load_suite_config', _denied)
    rc = suite_cli.cmd_suite(_args(tmp_path, 'switch'))
    assert rc == 1
    assert 'permission denied' in capsys.readouterr().err


def test_unit_flag_unreadable_manifest_exits_clean(tmp_path, monkeypatch):
    import pytest

    from nxs.cli import _manifest_unit

    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    (tmp_path / 'aliensense').mkdir()
    (tmp_path / 'aliensense' / 'suite.yaml').write_text("units: []\n")
    monkeypatch.setattr('nxs.suite.schema.load_suite_config',
                        lambda p: (_ for _ in ()).throw(PermissionError("denied")))
    with pytest.raises(SystemExit, match='--unit'):
        _manifest_unit('anything')


def test_apply_names_the_bootstrap_when_no_manifest(tmp_path, capsys):
    rc = suite_cli.cmd_suite(_args(tmp_path, 'switch'))
    assert rc == 1
    assert 'scan --init' in capsys.readouterr().err


MANIFEST = """\
units:
  - name: imu-mast
    module: nxs
    links: [{transport: i2c, bus: /dev/i2c-9, address: 0x30}]
    sensors: []
"""


def _main_args(**overrides):
    defaults = {'transport': 'i2c', 'bus': '/dev/i2c-9', 'addr': 0x30,
                'port': None, 'remote_node_id': None, 'unit': None,
                'command': 'set'}
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def test_manifest_unit_resolves_a_declared_unit(tmp_path, monkeypatch):
    from nxs.cli import _manifest_unit

    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    (tmp_path / 'aliensense').mkdir()
    (tmp_path / 'aliensense' / 'suite.yaml').write_text(MANIFEST)
    unit = _manifest_unit('imu-mast')
    assert unit.links[0].client_kwargs() == {'bus': '/dev/i2c-9',
                                             'address': 0x30}

    import pytest
    with pytest.raises(SystemExit, match='ghost'):
        _manifest_unit('ghost')


def test_pick_unit_link_fails_over_in_declared_order(monkeypatch):
    """`--unit` management follows apply's rule: the first link whose
    device answers wins; when nothing answers, the first link is
    returned blind so recovery verbs still reach non-probing devices."""
    import nxs.cli as cli_mod
    from nxs.suite.schema import parse_suite_config

    cfg = parse_suite_config({'units': [
        {'name': 'imu', 'links': [
            {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x30},
            {'transport': 'cyphal-can', 'iface': 'can1', 'node_id': 125}]}]})

    class _Probe:
        def __init__(self, ok):
            self._ok = ok
            self._closed = False

        def probe(self):
            return self._ok

        def close(self):
            self._closed = True

        def closed(self):
            return self._closed

    dead, alive = _Probe(False), _Probe(True)
    monkeypatch.setattr(cli_mod, 'open_client',
                        lambda kind, **kw: {'i2c': dead,
                                            'cyphal-can': alive}[kind])
    link, transport = cli_mod._pick_unit_link(cfg.units[0])
    assert link.transport == 'cyphal-can'
    assert transport is alive
    assert dead.closed()

    monkeypatch.setattr(cli_mod, 'open_client',
                        lambda kind, **kw: _Probe(False))
    link, transport = cli_mod._pick_unit_link(cfg.units[0])
    assert link.transport == 'i2c'
    assert transport is None


def test_apply_unit_link_syncs_display_args():
    """`--unit` must update the args a command later prints/decides on,
    not just the opened transport."""
    from argparse import Namespace

    from nxs.cli import _apply_unit_link
    from nxs.suite.schema import LinkSpec

    args = Namespace(transport='i2c', bus='/dev/i2c-2', addr=0x30,
                     port=None, remote_node_id=None, baud=460800)
    _apply_unit_link(args, LinkSpec(transport='i2c', bus='/dev/i2c-9',
                                    address=0x31))
    assert (args.transport, args.bus, args.addr) == ('i2c', '/dev/i2c-9', 0x31)

    _apply_unit_link(args, LinkSpec(transport='cyphal-can', iface='can0',
                                    node_id=10))
    assert (args.transport, args.port, args.remote_node_id) == \
        ('cyphal-can', 'can0', 10)

    _apply_unit_link(args, LinkSpec(transport='cyphal-serial',
                                    port='/dev/ttyUSB0', baud=115200))
    assert (args.transport, args.port, args.baud) == \
        ('cyphal-serial', '/dev/ttyUSB0', 115200)


def test_flag_addressed_mutation_warns_once(tmp_path, monkeypatch, capsys):
    from nxs.cli import _warn_if_suite_managed

    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    (tmp_path / 'aliensense').mkdir()
    (tmp_path / 'aliensense' / 'suite.yaml').write_text(MANIFEST)
    _warn_if_suite_managed(_main_args())
    err = capsys.readouterr().err
    assert "imu-mast" in err and "suite freeze" in err

    # A different address is not a declared unit — silence.
    _warn_if_suite_managed(_main_args(addr=0x31))
    assert capsys.readouterr().err == ''


def test_warning_is_best_effort_without_a_manifest(tmp_path, monkeypatch, capsys):
    from nxs.cli import _warn_if_suite_managed

    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    _warn_if_suite_managed(_main_args())
    assert capsys.readouterr().err == ''


def test_global_unit_and_suite_unit_do_not_collide():
    """The top-level `nxs --unit` (manual addressing) and the suite
    subcommand `--unit` (unit filter) parse into distinct dests, so one
    never drives the other."""
    from nxs.cli import build_parser

    parser = build_parser()
    ns = parser.parse_args(['--unit', 'mast', 'suite', 'switch', '--unit', 'knee'])
    assert ns.unit == 'mast'         # top-level: manual transport addressing
    assert ns.only_unit == 'knee'    # suite: which unit to reconcile

    ns = parser.parse_args(['suite', 'freeze', '--all'])
    assert ns.all and ns.only_unit is None


def test_cmd_freeze_labels_a_failed_unit_failed(monkeypatch, capsys):
    from argparse import Namespace

    from nxs.suite.freeze import FreezeReport

    rep = FreezeReport(name='u1', ok=False, error='no active driver to freeze')
    monkeypatch.setattr('nxs.suite.freeze.freeze_suite', lambda *a, **k: [rep])
    rc = suite_cli._cmd_freeze(Namespace(only_unit='u1', dry_run=False,
                                         pin_firmware=False), None, 'x')
    out = capsys.readouterr().out
    assert '✗ u1: failed' in out and 'no active driver' in out
    assert rc == 1


def test_cmd_freeze_reports_a_manifest_write_failure(monkeypatch, capsys):
    from argparse import Namespace

    def _boom(*a, **k):
        raise OSError("read-only file system")

    monkeypatch.setattr('nxs.suite.freeze.freeze_suite', _boom)
    rc = suite_cli._cmd_freeze(Namespace(only_unit='u1', dry_run=False,
                                         pin_firmware=False), None,
                               '/ro/suite.yaml')
    assert rc == 1 and 'cannot rewrite' in capsys.readouterr().err


def test_is_mutating_spares_the_read_forms():
    from nxs.cli import _is_mutating

    assert _is_mutating(_main_args(command='set'))
    assert _is_mutating(_main_args(command='push-fw'))
    assert _is_mutating(_main_args(command='store', store_cmd='clear'))
    assert not _is_mutating(_main_args(command='store', store_cmd='ls'))
    assert not _is_mutating(_main_args(command='decimation', value=None))
    assert _is_mutating(_main_args(command='decimation', value=4))
    assert not _is_mutating(_main_args(command='commission', show=True,
                                       node_id=None, subject=[], save=False))
    assert _is_mutating(_main_args(command='commission', show=False,
                                   node_id=10, subject=[], save=False))
    assert not _is_mutating(_main_args(command='probe'))


def test_collect_garbage_reports_an_unwritable_state_file(
        tmp_path, capsys, monkeypatch):
    """A state.save() failure means nothing was dropped durably; the verb
    reports that cleanly instead of a traceback."""
    from nxs.suite.cli import _cmd_collect_garbage
    from nxs.suite.schema import load_suite_config
    from nxs.suite.state import SuiteState

    manifest = tmp_path / 'suite.yaml'
    manifest.write_text("suite: {name: bench}\n"
                        "units:\n"
                        "  - name: u1\n"
                        "    module: nxs\n"
                        "    links: [{transport: mock}]\n")
    cfg = load_suite_config(str(manifest))
    state = SuiteState.load(str(tmp_path / 'state.yaml'))
    monkeypatch.setattr(state, 'unit_names', lambda: ['ghost'])
    monkeypatch.setattr(state, 'forget', lambda name: None)

    def refuse():
        raise OSError('read-only file system')

    monkeypatch.setattr(state, 'save', refuse)
    rc = _cmd_collect_garbage(cfg, state)
    captured = capsys.readouterr()
    assert rc == 1
    assert 'cannot write the state file' in captured.err
    assert 'dropped' not in captured.out
