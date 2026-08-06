"""Tests for the suite.yaml schema — the manifest is the customer
contract, so every malformed shape must fail with the YAML path named,
and defaults must inherit exactly one way (unit overrides suite
default)."""
import pytest

from nxs.suite.schema import ManifestError, parse_suite_config, parse_version


def _minimal(unit_overrides=None, **top):
    unit = {'name': 'u1', 'links': [{'transport': 'mock'}]}
    unit.update(unit_overrides or {})
    cfg = {'units': [unit]}
    cfg.update(top)
    return cfg


def test_parse_minimal_unit_defaults_module_nxs():
    cfg = parse_suite_config(_minimal())
    assert cfg.units[0].module == 'nxs'
    # Absent sensors key = panel unmanaged (None), not enforced-empty.
    assert cfg.units[0].sensors is None
    assert cfg.units[0].firmware is None


def test_sensors_tri_state():
    """Absent key = unmanaged (None); explicit [] = enforce empty."""
    assert parse_suite_config(_minimal()).units[0].sensors is None
    cfg = parse_suite_config(_minimal({'sensors': []}))
    assert cfg.units[0].sensors == []


def test_links_map_to_open_client_kwargs():
    cfg = parse_suite_config({'units': [
        {'name': 'a', 'links': [{'transport': 'i2c', 'bus': '/dev/i2c-9',
                                 'address': '0x31'}]},
        {'name': 'b', 'links': [{'transport': 'cyphal-can', 'iface': 'can0',
                                 'node_id': 10}]},
        {'name': 'c', 'links': [{'transport': 'cyphal-serial',
                                 'port': '/dev/ttyUSB0', 'baud': 460800}]},
    ]})
    assert cfg.units[0].links[0].client_kwargs() == {'bus': '/dev/i2c-9',
                                                     'address': 0x31}
    assert cfg.units[1].links[0].client_kwargs() == {'can_iface': 'can0',
                                                     'remote_node_id': 10}
    assert cfg.units[2].links[0].client_kwargs() == {'port': '/dev/ttyUSB0',
                                                     'baud': 460800}


def test_multi_link_unit_keeps_declared_order():
    """links[] order is the management priority — apply tries the first
    that answers, so parsing must not reorder."""
    cfg = parse_suite_config({'units': [
        {'name': 'imu', 'links': [
            {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x30},
            {'transport': 'cyphal-can', 'iface': 'can1', 'node_id': 125}]}]})
    assert [l.transport for l in cfg.units[0].links] == ['i2c', 'cyphal-can']


def test_defaults_firmware_inherits_and_unit_overrides():
    cfg = parse_suite_config({
        'defaults': {'firmware': '1.0.0'},
        'units': [
            {'name': 'a', 'links': [{'transport': 'mock'}]},
            {'name': 'b', 'links': [{'transport': 'mock'}], 'firmware': '1.1.0'},
        ]})
    assert cfg.units[0].firmware == '1.0.0'
    assert cfg.units[1].firmware == '1.1.0'


def test_serial_pin_normalized():
    cfg = parse_suite_config(_minimal(
        {'serial': '2F:00:4B:00:32:51:0F:00:11:22:33:44'}))
    assert cfg.units[0].serial == '2f004b0032510f0011223344'


def test_numeric_serial_is_rejected_with_a_quote_hint():
    """An unquoted <digits>e<digits> UID resolves as a YAML float in
    some parsers and silently loses digits; a non-string here must
    fail loudly, naming the fix."""
    import pytest

    with pytest.raises(ManifestError, match='quote'):
        parse_suite_config(_minimal({'serial': 2.030355845315e+46}))


def test_driver_names_accept_marketing_dashes():
    """`neo-m9n` is the product spelling; the module file is
    `neo_m9n.py` — dashes normalize instead of erroring."""
    cfg = parse_suite_config(_minimal(
        {'sensors': [{'driver': 'neo-m9n'}]}))
    assert cfg.units[0].sensors[0].driver == 'neo_m9n'


@pytest.mark.parametrize("bad, fragment", [
    ({'units': []}, "non-empty 'units'"),
    ({'units': [{'name': 'x'}]}, "needs a 'links'"),
    ({'units': [{'links': [{'transport': 'mock'}]}]}, "needs a 'name'"),
    (_minimal({'links': [{'transport': 'spi'}]}), "unknown transport"),
    (_minimal({'links': [{'transport': 'i2c', 'bus': '/dev/i2c-9'}]}),
     "needs ['address']"),
    (_minimal({'links': [{'transport': 'cyphal-can', 'iface': 'can0',
                          'node_id': 200}]}), "out of range"),
    (_minimal({'serial': 'zz'}), "24 hex digits"),
    (_minimal({'sensors': [{'driver': '../../tmp/evil'}]}), "not a module name"),
    ({'units': [{'name': 'x', 'links': [{'transport': 'i2c',
                                         'bus': '/dev/i2c-9',
                                         'address': 0x80}]}]},
     "not a usable 7-bit I2C address"),
    (_minimal({'firmware': 'v1'}), "bad version"),
    (_minimal({'oops': 1}), "unknown key"),
    (_minimal(defaults={'oops': 1}), "unknown key"),
    ({'units': [{'name': 'x', 'links': [{'transport': 'mock'}]},
                {'name': 'x', 'links': [{'transport': 'mock'}]}]}, "duplicate"),
    (_minimal({'sensors': [{'driver': 'iam20680'}, {'driver': 'iam20680'}]}),
     "duplicate driver"),
    (_minimal({'links': []}), "non-empty list"),
    (_minimal({'links': [
        {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x30},
        {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x30}]}),
     "declared twice"),
    ({'units': [
        {'name': 'a', 'links': [{'transport': 'i2c', 'bus': '/dev/i2c-9',
                                 'address': 0x30}]},
        {'name': 'b', 'links': [{'transport': 'i2c', 'bus': '/dev/i2c-9',
                                 'address': 0x30}]}]},
     "one route reaches one board"),
    ({'units': [
        {'name': 'a', 'links': [{'transport': 'mock'}],
         'serial': '2f004b0032510f0011223344'},
        {'name': 'b', 'links': [{'transport': 'mock'}],
         'serial': '2f004b0032510f0011223344'}]},
     "pin the same serial"),
])
def test_malformed_manifest_names_the_problem(bad, fragment):
    with pytest.raises(ManifestError) as e:
        parse_suite_config(bad)
    assert fragment in str(e.value)


def test_load_binary_manifest_raises_manifest_error(tmp_path):
    from nxs.suite.schema import load_suite_config

    path = str(tmp_path / 'suite.yaml')
    with open(path, 'wb') as f:
        f.write(b'\xff\xfe\x00 not utf-8')
    with pytest.raises(ManifestError, match='not valid YAML'):
        load_suite_config(path)


def test_parse_version_pads_patch():
    assert parse_version('1.2') == (1, 2, 0)
    assert parse_version('1.2.3') == (1, 2, 3)
    with pytest.raises(ValueError):
        parse_version('1')


def test_identity_resolves_stable_device_aliases(tmp_path):
    """A udev alias and the kernel's enumerated node are one link:
    `bus: /dev/i2c-cam1` in the manifest must match a scan or a
    `-b /dev/i2c-9` invocation that used the raw node."""
    from nxs.suite.schema import LinkSpec

    node = tmp_path / 'i2c-9'
    node.write_text('')
    alias = tmp_path / 'i2c-cam1'
    alias.symlink_to(node)

    by_alias = LinkSpec(transport='i2c', bus=str(alias), address=0x30)
    by_node = LinkSpec(transport='i2c', bus=str(node), address=0x30)
    assert by_alias.identity() == by_node.identity()

    port_node = tmp_path / 'ttyUSB0'
    port_node.write_text('')
    port_alias = tmp_path / 'usb-FTDI-if00-port0'
    port_alias.symlink_to(port_node)
    assert (LinkSpec(transport='cyphal-serial', port=str(port_alias)).identity()
            == LinkSpec(transport='cyphal-serial', port=str(port_node)).identity())
