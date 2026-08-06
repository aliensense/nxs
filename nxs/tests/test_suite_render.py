"""Tests for the suite render surfaces — scan --init emits a loadable
manifest skeleton, scan --diff names drift, status renders one row per
declared unit."""
from nxs.suite.scan import Found, _inspect, render_diff, render_init
from nxs.suite.schema import LinkSpec, parse_suite_config
from nxs.suite.state import SuiteState
from nxs.suite.status import collect_status, render_status

import yaml


def _found_i2c():
    return Found(link=LinkSpec(transport='i2c', bus='/dev/i2c-9', address=0x31),
                 serial='2f004b0032510f0011223344', fw_version='1.1')


def _found_can():
    return Found(link=LinkSpec(transport='cyphal-can', iface='can0', node_id=125),
                 serial='aa004b0032510f0011223344')


def test_scan_reports_a_probed_unit_even_if_metadata_reads_raise():
    """A present unit whose serial/fw read throws (I²C mid-transaction)
    is still reported — blank fields, not dropped."""
    class _MetaRaises:
        def probe(self):
            return True

        def read_serial(self):
            raise OSError("bus glitch")

        def close(self):
            pass

    found = _inspect(_MetaRaises(),
                     LinkSpec(transport='i2c', bus='/dev/i2c-9', address=0x30))
    assert found is not None and found.serial == '' and found.fw_version == ''


def test_scan_drops_a_unit_whose_probe_raises():
    class _ProbeRaises:
        def probe(self):
            raise OSError("no device")

        def close(self):
            pass

    assert _inspect(_ProbeRaises(),
                    LinkSpec(transport='i2c', bus='/dev/i2c-9', address=0x30)) is None


def test_scan_i2c_probes_declared_addresses(monkeypatch):
    """A unit declared at a non-default address (a SerDes alias) is part
    of the sweep, so `scan --diff` cannot report an answering unit as
    missing just because its address isn't a well-known one."""
    from nxs.suite import scan as scan_mod

    monkeypatch.setattr(scan_mod.glob, 'glob', lambda pattern: ['/dev/i2c-9'])

    class _OnlyAt35:
        def __init__(self, address):
            self._address = address

        def probe(self):
            return self._address == 0x35

        def read_serial(self):
            return bytes.fromhex('2f004b0032510f0011223344')

        def read_fw_version(self):
            return '1.0'

        def read_driver_name(self):
            return ''

        def close(self):
            pass

    declared = [LinkSpec(transport='i2c', bus='/dev/i2c-9', address=0x35)]
    found = scan_mod._scan_i2c(lambda kind, **kw: _OnlyAt35(kw['address']),
                               declared)
    assert [hit.link.address for hit in found] == [0x35]


def test_scan_i2c_skips_mux_parent_buses(monkeypatch):
    """A GPIO/i2c-mux parent reaches whichever child channel the mux
    currently selects — probing it duplicates a child-bus unit under a
    nondeterministic address, so only the leaf buses are swept."""
    from nxs.suite import scan as scan_mod

    monkeypatch.setattr(scan_mod.glob, 'glob',
                        lambda pattern: ['/dev/i2c-2', '/dev/i2c-9',
                                         '/dev/i2c-10'])
    names = {'/dev/i2c-2': '3180000.i2c',
             '/dev/i2c-9': 'i2c-2-mux (chan_id 0)',
             '/dev/i2c-10': 'i2c-2-mux (chan_id 1)'}
    monkeypatch.setattr(scan_mod, '_adapter_name',
                        lambda bus: names.get(bus, ''))

    assert scan_mod._i2c_buses() == ['/dev/i2c-10', '/dev/i2c-9']


def test_scan_i2c_dedupes_bus_aliases_and_prefers_them(monkeypatch, tmp_path):
    """A udev alias matches the same /dev/i2c-* glob as its node; the
    bus must be probed once, and reported under the stable alias so a
    transcribed manifest carries connector names."""
    from nxs.suite import scan as scan_mod

    node = tmp_path / 'i2c-9'
    node.write_text('')
    alias = tmp_path / 'i2c-cam1'
    alias.symlink_to(node)
    monkeypatch.setattr(scan_mod.glob, 'glob',
                        lambda pattern: [str(node), str(alias)])

    probed = []

    class _At30:
        def __init__(self, bus, address):
            self._address = address
            probed.append((bus, address))

        def probe(self):
            return self._address == 0x30

        def read_serial(self):
            return b''

        def read_fw_version(self):
            return ''

        def read_driver_name(self):
            return ''

        def close(self):
            pass

    found = scan_mod._scan_i2c(
        lambda kind, **kw: _At30(kw['bus'], kw['address']), [])
    assert [hit.link.bus for hit in found] == [str(alias)]
    assert len(probed) == len(scan_mod.I2C_ADDRESSES)


def test_scan_serial_probes_declared_ports_only_beyond_usb(monkeypatch):
    """A declared cyphal-serial unit on a non-USB port (a SoC UART) is
    part of the sweep; an undeclared non-USB port is not — a GetInfo
    probe writes to the port, which is rude on a console UART."""
    from nxs.suite import scan as scan_mod

    class _Port:
        def __init__(self, device, vid):
            self.device = device
            self.vid = vid

    class _Ports:
        @staticmethod
        def comports():
            return [_Port('/dev/ttyTHS0', None)]   # console UART, undeclared

    import serial.tools.list_ports  # noqa: F401 — bind the submodule attribute
    import serial.tools
    monkeypatch.setattr(serial.tools, 'list_ports', _Ports)
    monkeypatch.setattr(scan_mod.glob, 'glob', lambda pattern: [])

    probed = []

    class _Answers:
        def __init__(self, port):
            probed.append(port)

        def probe(self):
            return True

        def read_serial(self):
            return b''

        def read_fw_version(self):
            return ''

        def read_driver_name(self):
            return ''

        def close(self):
            pass

    declared = [LinkSpec(transport='cyphal-serial', port='/dev/ttyTHS1',
                         baud=115200)]
    opened = []
    found = scan_mod._scan_serial(
        lambda kind, **kw: (opened.append(kw), _Answers(kw['port']))[1],
        declared)
    assert probed == ['/dev/ttyTHS1']
    assert [hit.link.port for hit in found] == ['/dev/ttyTHS1']
    # The declared baud rides into the probe — a unit wired at a
    # non-default rate must not scan as "missing".
    assert opened == [{'port': '/dev/ttyTHS1', 'baud': 115200}]
    assert found[0].link.baud == 115200


def test_serial_alias_prefers_by_id_name(monkeypatch, tmp_path):
    """A port with a /dev/serial/by-id alias is reported under it, so
    the transcribed manifest survives replug reordering; a port without
    one stays as enumerated."""
    from nxs.suite import scan as scan_mod

    node = tmp_path / 'ttyUSB0'
    node.write_text('')
    alias = tmp_path / 'usb-FTDI_A50285BI-if00-port0'
    alias.symlink_to(node)
    monkeypatch.setattr(scan_mod.glob, 'glob', lambda pattern: [str(alias)])

    assert scan_mod._serial_alias(str(node)) == str(alias)
    assert scan_mod._serial_alias('/dev/ttyACM7') == '/dev/ttyACM7'


def test_render_diff_matches_across_bus_alias(tmp_path):
    """An alias-declared unit and a scan hit on the raw node are the
    same link — no false missing/undeclared pair."""
    node = tmp_path / 'i2c-9'
    node.write_text('')
    alias = tmp_path / 'i2c-cam1'
    alias.symlink_to(node)

    cfg = parse_suite_config({'units': [
        {'name': 'imu-a',
         'links': [{'transport': 'i2c', 'bus': str(alias), 'address': 0x30}]},
    ]})
    hit = Found(link=LinkSpec(transport='i2c', bus=str(node), address=0x30))
    assert 'manifest matches reality' in render_diff(cfg, [hit])


def test_render_init_output_is_a_loadable_manifest():
    text = render_init([_found_i2c(), _found_can()])
    cfg = parse_suite_config(yaml.safe_load(text))
    assert [u.links[0].transport for u in cfg.units] == ['i2c', 'cyphal-can']
    assert cfg.units[0].serial == '2f004b0032510f0011223344'
    # Serials are quoted — a bare <digits>e<digits> UID would resolve
    # as YAML scientific notation in round-trip parsers.
    assert 'serial: "2f004b0032510f0011223344"' in text
    # The running version transcribes as a comment, never as an active
    # pin that apply would then demand an image for.
    assert cfg.units[0].firmware is None
    assert '# firmware: "1.1"' in text


def test_render_init_groups_same_serial_into_one_unit():
    """A dual-homed board (one silicon, two routes) answers on both
    paths; the skeleton emits one unit carrying both links — and stays
    a loadable manifest."""
    dup_can = Found(link=LinkSpec(transport='cyphal-can', iface='can0',
                                  node_id=125),
                    serial='2f004b0032510f0011223344')
    text = render_init([_found_i2c(), dup_can])
    cfg = parse_suite_config(yaml.safe_load(text))
    assert len(cfg.units) == 1
    assert sorted(l.transport for l in cfg.units[0].links) == \
        ['cyphal-can', 'i2c']
    assert cfg.units[0].serial == '2f004b0032510f0011223344'


def test_render_init_orders_grouped_links_i2c_can_serial():
    """links[0] is the management default: wired-local I2C leads, CAN
    next, the serial debug cable last — independent of discovery
    order."""
    same = '2f004b0032510f0011223344'
    hits = [Found(link=LinkSpec(transport='cyphal-serial',
                                port='/dev/ttyUSB0'), serial=same),
            Found(link=LinkSpec(transport='cyphal-can', iface='can0',
                                node_id=125), serial=same),
            _found_i2c()]
    cfg = parse_suite_config(yaml.safe_load(render_init(hits)))
    assert [l.transport for l in cfg.units[0].links] == \
        ['i2c', 'cyphal-can', 'cyphal-serial']


def test_render_init_keeps_serial_less_hits_apart():
    """Grouping is evidence-based: without a serial two hits cannot be
    proven one board, so they stay separate units, flagged for the
    operator."""
    blank_a = Found(link=LinkSpec(transport='i2c', bus='/dev/i2c-9',
                                  address=0x30))
    blank_b = Found(link=LinkSpec(transport='cyphal-can', iface='can0',
                                  node_id=125))
    text = render_init([blank_a, blank_b])
    assert len(parse_suite_config(yaml.safe_load(text)).units) == 2
    assert text.count('# no serial read') == 2


def test_render_init_empty_scan_stays_valid_yaml():
    text = render_init([])
    assert yaml.safe_load(text) == {'units': []}


def test_render_diff_names_missing_and_undeclared():
    cfg = parse_suite_config({'units': [
        {'name': 'imu-mast',
         'links': [{'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x31}]},
        {'name': 'ghost',
         'links': [{'transport': 'i2c', 'bus': '/dev/i2c-10', 'address': 0x30}]},
    ]})
    text = render_diff(cfg, [_found_i2c(), _found_can()])
    assert 'missing   ghost' in text
    assert 'undeclared can can0 node 125' in text


def test_render_diff_labels_an_undeclared_edge_of_a_declared_unit():
    """An answering route to silicon the manifest already pins belongs
    to that unit — the diff names it and suggests the links: addition
    instead of reporting a stranger."""
    cfg = parse_suite_config({'units': [
        {'name': 'imu-mast', 'serial': '2f004b0032510f0011223344',
         'links': [{'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x31}]},
    ]})
    second = Found(link=LinkSpec(transport='cyphal-can', iface='can0',
                                 node_id=125),
                   serial='2f004b0032510f0011223344')
    text = render_diff(cfg, [_found_i2c(), second])
    assert 'undeclared edge of imu-mast: can can0 node 125' in text
    assert 'undeclared can can0' not in text


def test_render_diff_reports_a_silent_declared_edge():
    """A declared route that stops answering while another still does
    is edge degradation, not a missing unit."""
    cfg = parse_suite_config({'units': [
        {'name': 'imu-mast', 'links': [
            {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x31},
            {'transport': 'cyphal-can', 'iface': 'can0', 'node_id': 125}]},
    ]})
    text = render_diff(cfg, [_found_i2c()])
    assert 'edge down imu-mast: can can0 node 125 silent' in text
    assert 'missing' not in text


def test_render_diff_flags_a_pinned_serial_mismatch():
    cfg = parse_suite_config({'units': [
        {'name': 'imu-mast', 'serial': 'bb004b0032510f0011223344',
         'links': [{'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x31}]},
    ]})
    text = render_diff(cfg, [_found_i2c()])
    assert 'serial    imu-mast' in text


class _StatusFake:
    def __init__(self, alive=True):
        self._alive = alive

    def probe(self):
        return self._alive

    def read_driver_name(self):
        return 'Iam20680'

    def read_vm_state(self):
        return 1

    def read_fw_version(self):
        return '1.1'

    def read_sample_count(self):
        return 42

    def read_serial(self):
        return bytes.fromhex('2f004b0032510f0011223344')

    def close(self):
        pass


class _ProbeOnlyFake(_StatusFake):
    """Probes fine, then every metadata read raises — a degraded link."""

    def read_driver_name(self):
        raise TimeoutError("register window timed out")  # one field fails


def test_status_reads_each_field_independently():
    """One failing metadata read (driver) blanks only its column; the
    unit stays up and the other fields still populate."""
    cfg = parse_suite_config({'units': [
        {'name': 'degraded', 'links': [{'transport': 'mock'}]}]})
    state = SuiteState('/nonexistent/state.yaml')
    rows = collect_status(cfg, state, opener=lambda k, **kw: _ProbeOnlyFake())
    assert rows[0].up
    assert rows[0].driver == '-'          # the failed read, blank
    assert rows[0].fw_version == '1.1'    # independent reads still populate
    assert rows[0].samples == 42


def test_status_drift_column_names_config_drift():
    from nxs.tests.test_suite_reconcile import FakeUnit, _cfg
    from nxs.suite.reconcile import switch_suite
    import os
    import tempfile

    fake = FakeUnit()
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'state.yaml')
        switch_suite(_cfg(), SuiteState.load(path), opener=lambda k, **kw: fake)
        fake.set_param('sample_rate', 500)
        rows = collect_status(_cfg(), SuiteState.load(path),
                              opener=lambda k, **kw: fake)
        assert rows[0].drift == 'config'
        assert 'DRIFT' in render_status(rows)

        # Repair, then the column reads clean.
        switch_suite(_cfg(), SuiteState.load(path), opener=lambda k, **kw: fake)
        rows = collect_status(_cfg(), SuiteState.load(path),
                              opener=lambda k, **kw: fake)
        assert rows[0].drift == '-'


def test_status_renders_up_and_down_rows():
    cfg = parse_suite_config({'units': [
        {'name': 'up-unit', 'links': [{'transport': 'mock'}]},
        {'name': 'down-unit', 'links': [{'transport': 'mock'}]},
    ]})
    fakes = iter([_StatusFake(), _StatusFake(alive=False)])
    state = SuiteState('/nonexistent/state.yaml')
    rows = collect_status(cfg, state, opener=lambda k, **kw: next(fakes))
    text = render_status(rows)
    assert 'up-unit' in text and 'Iam20680' in text and 'running' in text
    assert 'down' in text
    # No pin and no TOFU record yet — the serial column says so.
    assert 'new' in text
