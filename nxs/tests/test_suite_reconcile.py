"""Tests for `suite switch` — the reconciler's contract:

- idempotence: a second apply against unchanged reality does nothing;
- dry-run: reports every action, writes neither device nor state;
- TOFU: first contact records the serial, later mismatches fail unless
  --accept-new-serial, and an explicit manifest pin is never overridden;
- firmware pin: flashes on a provable mismatch or an unknown version,
  skips when the state file already records the pin;
- isolation: one failing unit does not block the rest.
"""
import os
import struct
import tempfile

import pytest

from nxs.suite.firmware import IMAGE_MAGIC
from nxs.suite.reconcile import switch_suite, load_unit_driver
from nxs.client import SupportsCommissioning
from nxs.suite.schema import parse_suite_config
from nxs.suite.state import SuiteState


class FakeUnit(SupportsCommissioning):
    """Records every write the reconciler makes, like a persisted device."""

    def __init__(self, serial="2f004b0032510f0011223344", fw="1.0",
                 node_addr=10, alive=True, raise_on_probe=False):
        self._serial = bytes.fromhex(serial) if serial else None
        self._fw = fw
        self._node_addr = node_addr
        self._alive = alive
        self._raise_on_probe = raise_on_probe
        self._closed = False
        self._slots = []
        self._driver_name = ""
        self._caps = []
        self._running = False
        self._pushed = []
        self._commissioned = []
        self._decimation = {}

    # ── NxsClient surface the reconciler touches ──────────
    def probe(self):
        if self._raise_on_probe:
            raise OSError("probe write failed")
        return self._alive

    def read_serial(self):
        return self._serial

    def read_fw_version(self):
        return self._fw

    def read_identity(self):
        return {"node_addr": self._node_addr, "topics": {}}

    def commission(self, node_addr=None, topics=None):
        self._commissioned.append(node_addr)

    def push_image(self, path, chunk_size=32, progress_cb=None):
        self._pushed.append(path)
        # A real device reboots into the pushed image; transports that
        # serve a version then report the new one (None stays None, as
        # on I2C).
        if self._fw is not None:
            from nxs.suite.firmware import read_image_version
            major, minor, _ = read_image_version(path)
            self._fw = f"{major}.{minor}"
        return os.path.getsize(path)

    def clear_store(self):
        self._slots.clear()

    def upload_image(self, image):
        from nxs.image import deserialize
        compiled = deserialize(image)
        self._driver_name = compiled.name
        self._caps = [{"name": p.name, "current": p.current,
                       "default": p.default} for p in compiled.params]

    def save_slot(self, slot):
        self._slots.append(self._driver_name)

    def vm_run(self):
        self._running = True

    def vm_stop(self):
        self._running = False

    def vm_reset(self):
        self._running = False
        self._driver_name = ""
        self._caps = []

    def read_decimation(self, subject=None):
        return self._decimation.get(subject, 1)

    def write_decimation(self, value, subject=None):
        self._decimation[subject] = value

    def read_store_count(self):
        return len(self._slots)

    def read_active_slot(self):
        return 0 if self._slots else 0xFF

    def read_driver_name(self):
        return self._driver_name

    def read_vm_state(self):
        return 1 if self._running else 0

    def read_sample_count(self):
        return 0

    def read_capabilities(self):
        return [dict(p) for p in self._caps]

    def set_param(self, name, value):
        for p in self._caps:
            if p["name"] == name:
                p["current"] = value

    def close(self):
        self._closed = True

    # ── Test accessors ─────────────────────────────────────
    def closed(self):
        return self._closed

    def slots(self):
        return list(self._slots)

    def pushed(self):
        return list(self._pushed)

    def commissioned(self):
        return list(self._commissioned)


def _cfg(**unit_overrides):
    unit = {'name': 'u1', 'links': [{'transport': 'mock'}],
            'sensors': [{'driver': 'iam20680', 'config': {'sample_rate': 250}}]}
    unit.update(unit_overrides)
    return parse_suite_config({'units': [unit]})


def _state(tmp):
    return SuiteState.load(os.path.join(tmp, 'state.yaml'))


def _mcuboot_image(path, version=(1, 1, 0)):
    header = bytearray(32)
    struct.pack_into('<I', header, 0, IMAGE_MAGIC)
    struct.pack_into('<BBH', header, 20, *version)
    with open(path, 'wb') as f:
        f.write(bytes(header) + b'\xFF' * 32)


def test_apply_deploys_then_converges():
    fake = FakeUnit()
    with tempfile.TemporaryDirectory() as tmp:
        state = _state(tmp)
        r1 = switch_suite(_cfg(), state, opener=lambda k, **kw: fake)
        assert r1[0].ok and fake.slots() == ['Iam20680']
        r2 = switch_suite(_cfg(), SuiteState.load(state.path),
                         opener=lambda k, **kw: fake)
        assert r2[0].actions == ['converged']


def test_dry_run_reports_and_writes_nothing():
    fake = FakeUnit()
    with tempfile.TemporaryDirectory() as tmp:
        state = _state(tmp)
        reports = switch_suite(_cfg(), state, dry_run=True,
                              opener=lambda k, **kw: fake)
        assert any(a.startswith('would deploy') for a in reports[0].actions)
        assert fake.slots() == []
        assert not os.path.exists(state.path)


def test_tofu_records_then_rejects_a_swapped_board():
    with tempfile.TemporaryDirectory() as tmp:
        state = _state(tmp)
        r1 = switch_suite(_cfg(), state, opener=lambda k, **kw: FakeUnit())
        assert any('recorded serial' in a for a in r1[0].actions)

        swapped = FakeUnit(serial='aa004b0032510f0011223344')
        r2 = switch_suite(_cfg(), SuiteState.load(state.path),
                         opener=lambda k, **kw: swapped)
        assert not r2[0].ok and 'accept-new-serial' in r2[0].error
        assert swapped.slots() == []  # refused before any device write

        r3 = switch_suite(_cfg(), SuiteState.load(state.path),
                         accept_new_serial=True,
                         opener=lambda k, **kw: swapped)
        assert r3[0].ok and swapped.slots() == ['Iam20680']


def test_manifest_serial_pin_is_never_overridden():
    pinned = _cfg(serial='2f004b0032510f0011223344')
    wrong = FakeUnit(serial='aa004b0032510f0011223344')
    with tempfile.TemporaryDirectory() as tmp:
        reports = switch_suite(pinned, _state(tmp), accept_new_serial=True,
                              opener=lambda k, **kw: wrong)
        assert not reports[0].ok and 'serial pin in the manifest' in reports[0].error


def test_firmware_pin_flashes_on_mismatch_and_skips_when_recorded():
    with tempfile.TemporaryDirectory() as tmp:
        store = os.path.join(tmp, 'fw')
        os.makedirs(store)
        _mcuboot_image(os.path.join(store, 'nxs.bin'), (1, 1, 0))

        fake = FakeUnit(fw='1.0')  # provable major.minor mismatch
        state = _state(tmp)
        r1 = switch_suite(_cfg(firmware='1.1.0'), state,
                         opener=lambda k, **kw: fake, firmware_dir=store)
        assert len(fake.pushed()) == 1
        assert any('flash firmware 1.1.0' in a for a in r1[0].actions)
        # The fake now reports the new version, as a real reboot would.
        assert r1[0].ok

        converged = FakeUnit(fw='1.1')
        r2 = switch_suite(_cfg(firmware='1.1.0'), SuiteState.load(state.path),
                         opener=lambda k, **kw: converged, firmware_dir=store)
        assert converged.pushed() == []
        assert r2[0].ok


def test_firmware_unknown_version_flashes_once():
    """A transport that serves no version (I2C today) converges via the
    state record: first switch flashes, the second trusts the record."""
    with tempfile.TemporaryDirectory() as tmp:
        store = os.path.join(tmp, 'fw')
        os.makedirs(store)
        _mcuboot_image(os.path.join(store, 'nxs.bin'), (1, 1, 0))

        state = _state(tmp)
        first = FakeUnit(fw=None)
        switch_suite(_cfg(firmware='1.1.0'), state,
                    opener=lambda k, **kw: first, firmware_dir=store)
        assert len(first.pushed()) == 1

        second = FakeUnit(fw=None)
        reports = switch_suite(_cfg(firmware='1.1.0'),
                              SuiteState.load(state.path),
                              opener=lambda k, **kw: second, firmware_dir=store)
        assert second.pushed() == [] and reports[0].ok


def test_missing_firmware_image_fails_before_touching_the_device():
    fake = FakeUnit(fw='1.0')
    with tempfile.TemporaryDirectory() as tmp:
        store = os.path.join(tmp, 'fw')
        os.makedirs(store)
        reports = switch_suite(_cfg(firmware='9.9.9'), _state(tmp),
                              opener=lambda k, **kw: fake, firmware_dir=store)
        assert not reports[0].ok and 'no image for firmware 9.9.9' in reports[0].error
        assert fake.slots() == [] and fake.pushed() == []


def test_commission_stages_the_declared_node_id():
    cfg = parse_suite_config({'units': [
        {'name': 'knee', 'links': [{'transport': 'cyphal-can', 'iface': 'can0',
                                    'node_id': 10}], 'sensors': []}]})
    factory = FakeUnit(node_addr=125)
    with tempfile.TemporaryDirectory() as tmp:
        reports = switch_suite(cfg, _state(tmp), opener=lambda k, **kw: factory)
        assert factory.commissioned() == [10]
        assert any('commission node-id 10' in a for a in reports[0].actions)


def test_a_raising_probe_fails_cleanly_and_closes_the_transport():
    """A serial probe can throw OSError; the unit fails without leaking
    the transport handle."""
    fake = FakeUnit(raise_on_probe=True)
    with tempfile.TemporaryDirectory() as tmp:
        reports = switch_suite(_cfg(), _state(tmp), opener=lambda k, **kw: fake)
        assert not reports[0].ok and 'no response' in reports[0].error
        assert fake.closed()


def test_commission_runs_after_firmware_for_a_factory_unit():
    """A factory CAN unit (answers only at 125) with a firmware pin must
    flash before it commissions — else the DFU reboot moves the node to
    the new id while the transport is still at 125."""
    cfg = parse_suite_config({'units': [
        {'name': 'knee', 'links': [{'transport': 'cyphal-can', 'iface': 'can0',
                                    'node_id': 10}], 'firmware': '1.1.0',
         'sensors': []}]})
    unit = FakeUnit(node_addr=125, fw='1.0')

    def opener(kind, **kwargs):
        unit._alive = kwargs.get('remote_node_id') == 125  # factory only
        return unit

    with tempfile.TemporaryDirectory() as tmp:
        store = os.path.join(tmp, 'fw')
        os.makedirs(store)
        _mcuboot_image(os.path.join(store, 'nxs.bin'), (1, 1, 0))
        reports = switch_suite(cfg, _state(tmp), opener=opener, firmware_dir=store)
        acts = reports[0].actions
        fw_i = next(i for i, a in enumerate(acts) if 'flash firmware' in a)
        cm_i = next(i for i, a in enumerate(acts) if 'commission node-id' in a)
        assert fw_i < cm_i and unit.pushed() and unit.commissioned() == [10]


def test_one_dead_unit_does_not_block_the_rest():
    cfg = parse_suite_config({'units': [
        {'name': 'dead', 'links': [{'transport': 'mock'}],
         'sensors': [{'driver': 'iam20680'}]},
        {'name': 'alive', 'links': [{'transport': 'mock'}],
         'sensors': [{'driver': 'iam20680'}]},
    ]})
    units = {'dead': FakeUnit(alive=False), 'alive': FakeUnit()}
    calls = iter(['dead', 'alive'])
    with tempfile.TemporaryDirectory() as tmp:
        reports = switch_suite(cfg, _state(tmp),
                              opener=lambda k, **kw: units[next(calls)])
        assert not reports[0].ok and 'no response' in reports[0].error
        assert reports[1].ok and units['alive'].slots() == ['Iam20680']


def test_broken_host_driver_fails_its_unit_only():
    """A syntax error in a host-dropped driver file is a per-unit
    failure, never a crash of the whole run."""
    cfg = parse_suite_config({'units': [
        {'name': 'broken', 'links': [{'transport': 'mock'}],
         'sensors': [{'driver': 'busted'}]},
        {'name': 'fine', 'links': [{'transport': 'mock'}],
         'sensors': [{'driver': 'iam20680'}]},
    ]})
    fine = FakeUnit()
    opens = []

    def opener(kind, **kwargs):
        opens.append(kind)
        return fine

    with tempfile.TemporaryDirectory() as tmp:
        drivers = os.path.join(tmp, 'drivers')
        os.makedirs(drivers)
        with open(os.path.join(drivers, 'busted.py'), 'w') as f:
            f.write("def broken(:\n")
        reports = switch_suite(cfg, _state(tmp), opener=opener,
                              drivers_dir=drivers)
        assert not reports[0].ok and 'import failed' in reports[0].error
        assert reports[1].ok and fine.slots() == ['Iam20680']
        # Staging fails before any hardware: only the healthy unit opened.
        assert opens == ['mock']


def test_no_state_write_when_nothing_was_reconciled():
    """`--unit` matching nothing must not create or touch state.yaml."""
    with tempfile.TemporaryDirectory() as tmp:
        state = _state(tmp)
        reports = switch_suite(_cfg(), state, only_unit='no-such-unit',
                              opener=lambda k, **kw: FakeUnit())
        assert reports == []
        assert not os.path.exists(state.path)


def test_param_drift_is_retuned_without_a_redeploy():
    """A teammate's `nxs set` is repaired with set_param — the store is
    never wiped, so there is no sample gap."""
    fake = FakeUnit()
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'state.yaml')
        switch_suite(_cfg(), SuiteState.load(path), opener=lambda k, **kw: fake)
        fake.set_param('sample_rate', 500)
        deploys_before = fake.slots()

        reports = switch_suite(_cfg(), SuiteState.load(path),
                              opener=lambda k, **kw: fake)
        assert reports[0].actions == ['retune sample_rate 500→250']
        assert fake.slots() == deploys_before  # store untouched
        assert any(p['name'] == 'sample_rate' and p['current'] == 250
                   for p in fake.read_capabilities())

        again = switch_suite(_cfg(), SuiteState.load(path),
                            opener=lambda k, **kw: fake)
        assert again[0].actions == ['converged']


def test_param_drift_dry_run_reports_the_retune():
    fake = FakeUnit()
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'state.yaml')
        switch_suite(_cfg(), SuiteState.load(path), opener=lambda k, **kw: fake)
        fake.set_param('sample_rate', 500)
        reports = switch_suite(_cfg(), SuiteState.load(path), dry_run=True,
                              opener=lambda k, **kw: fake)
        assert reports[0].actions == ['would retune sample_rate 500→250']
        assert any(p['name'] == 'sample_rate' and p['current'] == 500
                   for p in fake.read_capabilities())


def test_apply_survives_an_unwritable_state_file(monkeypatch):
    """A failed state.save() warns but doesn't crash apply — the units
    are already converged on the hardware."""
    fake = FakeUnit()
    with tempfile.TemporaryDirectory() as tmp:
        state = _state(tmp)

        def _boom():
            raise OSError("read-only file system")

        monkeypatch.setattr(state, 'save', _boom)
        reports = switch_suite(_cfg(), state, opener=lambda k, **kw: fake)
        assert reports[0].ok


def test_missing_driver_names_paths_and_generation_route():
    cfg = parse_suite_config({'units': [
        {'name': 'u1', 'links': [{'transport': 'mock'}],
         'sensors': [{'driver': 'does_not_exist'}]}]})
    with tempfile.TemporaryDirectory() as tmp:
        reports = switch_suite(cfg, _state(tmp),
                              opener=lambda k, **kw: FakeUnit(),
                              drivers_dir=os.path.join(tmp, 'drivers'))
        assert not reports[0].ok
        assert 'generate-sensor-driver' in reports[0].error


def test_host_drivers_dir_takes_precedence_over_builtins():
    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, 'blinky.py'), 'w') as f:
            f.write("from nxs import RegisterDriver\n\n\n"
                    "class Blinky(RegisterDriver):\n"
                    "    WHO_AM_I_REG = 0x00\n"
                    "    WHO_AM_I_VALUES = [0x01]\n")
        cls = load_unit_driver('blinky', drivers_dir=tmp)
        assert cls.__name__ == 'Blinky'
    with pytest.raises(Exception):
        load_unit_driver('blinky', drivers_dir='/nonexistent')


def test_host_driver_file_must_define_exactly_one_driver():
    with tempfile.TemporaryDirectory() as tmp:
        with open(os.path.join(tmp, 'twins.py'), 'w') as f:
            f.write("from nxs import RegisterDriver\n\n\n"
                    "class TwinA(RegisterDriver):\n    pass\n\n\n"
                    "class TwinB(RegisterDriver):\n    pass\n")
        with pytest.raises(Exception, match='one driver per file'):
            load_unit_driver('twins', drivers_dir=tmp)


def test_apply_fails_over_to_the_second_link(tmp_path):
    """A unit whose first route is dead is converged over the next one,
    with the dead edge reported — a dropped link needs no manifest
    edit."""
    cfg = parse_suite_config({'units': [
        {'name': 'imu', 'links': [
            {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x30},
            {'transport': 'cyphal-can', 'iface': 'can1', 'node_id': 10}],
         'sensors': []}]})
    device = FakeUnit(node_addr=10)

    def opener(kind, **kwargs):
        if kind == 'i2c':
            raise OSError('bus gone')
        return device

    reports = switch_suite(cfg, _state(tmp_path), opener=opener)
    assert reports[0].ok
    assert reports[0].link == 'can can1 node 10'
    assert any('edge i2c /dev/i2c-9@0x30: down' in a
               for a in reports[0].actions)


def test_a_link_reaching_different_silicon_fails_before_provisioning(tmp_path):
    """Two routes answering with two serials is miswiring (or a
    merged-bus ghost): the unit fails on the cross-link identity check
    before anything is written."""
    cfg = parse_suite_config({'units': [
        {'name': 'imu', 'links': [
            {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x30},
            {'transport': 'cyphal-can', 'iface': 'can1', 'node_id': 10}],
         'sensors': [{'driver': 'iam20680', 'config': {'sample_rate': 250}}]}]})
    board_a = FakeUnit(serial='aa004b0032510f0011223344')
    board_b = FakeUnit(serial='bb004b0032510f0011223344', node_addr=10)

    reports = switch_suite(cfg, _state(tmp_path),
                          opener=lambda kind, **kw:
                          board_a if kind == 'i2c' else board_b)
    assert not reports[0].ok
    assert 'cannot verify' in reports[0].error
    assert board_a.slots() == [] and board_b.slots() == []
    assert board_b.commissioned() == []


def test_two_units_resolving_to_one_silicon_fail_the_second(tmp_path):
    """A hand-written manifest can still declare one board twice as two
    single-link units; the second observation fails loudly instead of
    double-converging the store."""
    cfg = parse_suite_config({'units': [
        {'name': 'left-leg', 'links': [{'transport': 'mock'}], 'sensors': []},
        {'name': 'right-leg', 'links': [{'transport': 'mock'}], 'sensors': []},
    ]})
    board = FakeUnit()
    reports = switch_suite(cfg, _state(tmp_path),
                          opener=lambda kind, **kw: board)
    assert reports[0].ok
    assert not reports[1].ok
    assert 'same silicon' in reports[1].error
    assert 'left-leg' in reports[1].error


def test_secondary_can_link_is_commissioned(tmp_path):
    """The node-id is CAN-edge config: it converges even when
    management runs over I2C."""
    cfg = parse_suite_config({'units': [
        {'name': 'imu', 'links': [
            {'transport': 'i2c', 'bus': '/dev/i2c-9', 'address': 0x30},
            {'transport': 'cyphal-can', 'iface': 'can1', 'node_id': 10}],
         'sensors': []}]})
    i2c_board = FakeUnit()
    can_board = FakeUnit(node_addr=125)

    reports = switch_suite(cfg, _state(tmp_path),
                          opener=lambda kind, **kw:
                          i2c_board if kind == 'i2c' else can_board)
    assert reports[0].ok
    assert can_board.commissioned() == [10]
    assert any('commission node-id 10' in a for a in reports[0].actions)


def test_switch_seeds_time_sync_on_capable_transports():
    """A converged switch leaves the unit disciplined: the seed pushes the
    host estimate and reports the bound as an action."""
    from nxs.client import SupportsTimeSync

    class SyncFakeUnit(FakeUnit, SupportsTimeSync):
        def __init__(self, **kw):
            super().__init__(**kw)
            from nxs.client import TimeSyncEstimator
            self._pushed = None
            self._epoch = 0
            self._ts = TimeSyncEstimator()

        def get_time_sync(self):
            return self._ts

        def time_sync_ping(self):
            import time as _time
            t0 = _time.monotonic_ns()
            device_us = self.read_device_time_us()
            t1 = _time.monotonic_ns()
            self._ts.observe(t0, t1, device_us)
            return True

        def read_device_time_us(self):
            self._epoch += 1000
            return self._epoch

        def push_time_sync(self, offset_us, bound_us, rate_ppb=0,
                           valid_for_us=0):
            self._pushed = (offset_us, bound_us, rate_ppb, valid_for_us)

        def read_time_sync(self):
            if self._pushed is None:
                return (0, 0, 0, 0, 0, False)
            return (self._pushed[0], self._pushed[1], self._pushed[2],
                    self._pushed[3], 1, True)

    fake = SyncFakeUnit()
    with tempfile.TemporaryDirectory() as tmp:
        reports = switch_suite(_cfg(), _state(tmp),
                              opener=lambda k, **kw: fake)
        assert reports[0].ok
        assert any("time sync seeded" in a for a in reports[0].actions)
        assert fake._pushed is not None
        assert fake._pushed[3] == 10_000_000
        _off, _bound, _rate, _window, source, valid = fake.read_time_sync()
        assert valid and source == 1
