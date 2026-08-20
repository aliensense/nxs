"""Tests for `suite freeze` — the reality→manifest verb: adopt live
tuning into suite.yaml surgically, comments intact."""
import os
import tempfile

from nxs.suite.freeze import freeze_suite
from nxs.suite.reconcile import switch_suite
from nxs.suite.schema import load_suite_config
from nxs.suite.state import SuiteState
from nxs.tests.test_suite_reconcile import FakeUnit

MANIFEST = """\
# Bench rig — hand-tuned; comments must survive a freeze.
suite: {name: bench}
units:
  - name: u1            # the mast IMU
    module: nxs
    links: [{transport: mock}]
    sensors:
      - driver: iam20680
        config: {sample_rate: 250}   # conservative default
"""


def _tuned_setup(tmp):
    """Manifest applied to a fake, then live-tuned past it."""
    path = os.path.join(tmp, 'suite.yaml')
    with open(path, 'w') as f:
        f.write(MANIFEST)
    cfg = load_suite_config(path)
    fake = FakeUnit()
    switch_suite(cfg, SuiteState.load(os.path.join(tmp, 'state.yaml')),
                opener=lambda k, **kw: fake)
    fake.set_param('sample_rate', 500)
    return path, cfg, fake


def test_freeze_adopts_tuning_and_preserves_comments():
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, fake = _tuned_setup(tmp)
        reports = freeze_suite(cfg, path, only_unit='u1',
                               opener=lambda k, **kw: fake)
        assert reports[0].ok and reports[0].changed
        assert reports[0].actions == ['sample_rate: 250→500']
        text = open(path).read()
        assert 'comments must survive' in text
        assert 'the mast IMU' in text
        assert 'sample_rate: 500' in text
        # Full capture: every tunable lands, not just the drifted one.
        assert 'gyro_fs: 2000' in text
        # The rewritten manifest is still a valid manifest.
        assert load_suite_config(path).units[0].sensors[0].config['sample_rate'] == 500


def test_freeze_preserves_non_ascii_comments(tmp_path):
    """The round-trip write is UTF-8, so accented operator comments
    survive regardless of the host locale."""
    import os

    path = str(tmp_path / 'suite.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(MANIFEST.replace('the mast IMU', 'l’IMU du mât — °C, µT'))
    cfg = load_suite_config(path)
    fake = FakeUnit()
    switch_suite(cfg, SuiteState.load(os.path.join(str(tmp_path), 's.yaml')),
                opener=lambda k, **kw: fake)
    fake.set_param('sample_rate', 500)
    freeze_suite(cfg, path, only_unit='u1', opener=lambda k, **kw: fake)
    text = open(path, encoding='utf-8').read()
    assert 'l’IMU du mât — °C, µT' in text
    assert 'sample_rate: 500' in text


def test_freeze_preserves_a_per_key_config_comment(tmp_path):
    """An inline comment on a specific config value survives freeze — the
    config map is updated in place, not replaced."""
    import os

    path = str(tmp_path / 'suite.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        f.write("units:\n"
                "  - name: u1\n"
                "    module: nxs\n"
                "    links: [{transport: mock}]\n"
                "    sensors:\n"
                "      - driver: iam20680\n"
                "        config:\n"
                "          sample_rate: 250   # conservative default\n")
    cfg = load_suite_config(path)
    fake = FakeUnit()
    switch_suite(cfg, SuiteState.load(os.path.join(str(tmp_path), 's.yaml')),
                opener=lambda k, **kw: fake)
    fake.set_param('sample_rate', 500)
    freeze_suite(cfg, path, only_unit='u1', opener=lambda k, **kw: fake)
    text = open(path, encoding='utf-8').read()
    assert 'sample_rate: 500' in text
    assert '# conservative default' in text   # per-key comment survived


def test_freeze_dry_run_writes_nothing():
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, fake = _tuned_setup(tmp)
        reports = freeze_suite(cfg, path, only_unit='u1', dry_run=True,
                               opener=lambda k, **kw: fake)
        assert reports[0].changed
        assert 'sample_rate: 250' in open(path).read()


def test_freeze_without_drift_reports_unchanged():
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, fake = _tuned_setup(tmp)
        fake.set_param('sample_rate', 250)  # tune back to the manifest
        reports = freeze_suite(cfg, path, only_unit='u1',
                               opener=lambda k, **kw: fake)
        assert reports[0].ok and not reports[0].changed
        assert reports[0].actions == ['no tuning to adopt']
        assert 'sample_rate: 250' in open(path).read()


def test_freeze_rejects_an_undeclared_active_driver():
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, fake = _tuned_setup(tmp)
        # A tuning session left a different driver running.
        from nxs.descriptor import load_driver
        from nxs.image import serialize
        fake.upload_image(serialize(load_driver('ms5611')().compile({})))
        reports = freeze_suite(cfg, path, only_unit='u1',
                               opener=lambda k, **kw: fake)
        assert not reports[0].ok
        assert 'not declared' in reports[0].error
        assert 'sample_rate: 250' in open(path).read()


def test_freeze_pin_firmware_captures_the_running_version():
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, fake = _tuned_setup(tmp)
        reports = freeze_suite(cfg, path, only_unit='u1', pin_firmware=True,
                               opener=lambda k, **kw: fake)
        assert reports[0].ok and reports[0].firmware == '1.0'
        assert "firmware: '1.0'" in open(path).read() \
            or 'firmware: 1.0' in open(path).read()


def test_freeze_refuses_an_unknown_orientation_code():
    """A device orientation code outside this SDK's vocabulary must not
    reach the manifest — the parser would reject the whole file."""
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, _ = _tuned_setup(tmp)

        from nxs.client import CalibrationRecord, SupportsCalibration

        class NewerOrientationUnit(FakeUnit, SupportsCalibration):
            def read_calibration(self):
                rec = CalibrationRecord()
                rec.orientation = 99
                return rec

            def write_calibration(self, record, persist=True):
                pass

            def set_orientation(self, rotation, persist=True):
                pass

            def cal_gyro(self):
                pass

            def cal_mag_start(self):
                pass

            def cal_mag_stop(self):
                pass

            def cal_abort(self):
                pass

            def save_calibration(self):
                pass

            def read_cal_epoch(self):
                return 1

            def read_cal_progress(self):
                return (0, 0, 0)

        oriented = NewerOrientationUnit()
        switch_suite(cfg, SuiteState.load(os.path.join(tmp, 's2.yaml')),
                    opener=lambda k, **kw: oriented)
        before = open(path).read()
        reports = freeze_suite(cfg, path, only_unit='u1',
                               opener=lambda k, **kw: oriented)
        assert not reports[0].ok
        assert 'orientation code 99' in (reports[0].error or '')
        assert open(path).read() == before


def test_freeze_survives_a_missing_calibration_surface():
    """Concrete transports always type as SupportsCalibration; connected
    firmware may predate the registers. The optional adoption must not
    fail the rest of the freeze."""
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, _ = _tuned_setup(tmp)

        from nxs.client import SupportsCalibration

        class LegacyUnit(FakeUnit, SupportsCalibration):
            def read_calibration(self):
                raise RuntimeError("register does not exist")

            def write_calibration(self, record, persist=True):
                pass

            def set_orientation(self, rotation, persist=True):
                pass

            def cal_gyro(self):
                pass

            def cal_mag_start(self):
                pass

            def cal_mag_stop(self):
                pass

            def cal_abort(self):
                pass

            def save_calibration(self):
                pass

            def read_cal_epoch(self):
                return 1

            def read_cal_progress(self):
                return (0, 0, 0)

        legacy = LegacyUnit()
        switch_suite(cfg, SuiteState.load(os.path.join(tmp, 's2.yaml')),
                    opener=lambda k, **kw: legacy)
        legacy.set_param('sample_rate', 500)
        reports = freeze_suite(cfg, path, only_unit='u1',
                               opener=lambda k, **kw: legacy)
        assert reports[0].ok
        assert any('calibration surface unavailable' in a
                   for a in reports[0].actions)
        assert any('sample_rate' in a for a in reports[0].actions)


def test_freeze_pin_normalizes_a_full_build_identity():
    """A device serving `git describe` pins its proven triple, never the
    raw string the manifest parser would reject."""
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, _ = _tuned_setup(tmp)
        fake = FakeUnit(fw='v1.2.3-4-g87fdf5b')
        switch_suite(cfg, SuiteState.load(os.path.join(tmp, 's2.yaml')),
                    opener=lambda k, **kw: fake)
        reports = freeze_suite(cfg, path, only_unit='u1', pin_firmware=True,
                               opener=lambda k, **kw: fake)
        assert reports[0].ok and reports[0].firmware == '1.2.3'
        # The rewritten manifest is still a valid manifest.
        assert load_suite_config(path).units[0].firmware == '1.2.3'


def test_freeze_pin_leaves_the_pin_on_a_bare_sha():
    """An untagged build proves no version; the pin must not adopt it."""
    with tempfile.TemporaryDirectory() as tmp:
        path, cfg, _ = _tuned_setup(tmp)
        fake = FakeUnit(fw='87fdf5b')
        switch_suite(cfg, SuiteState.load(os.path.join(tmp, 's2.yaml')),
                    opener=lambda k, **kw: fake)
        reports = freeze_suite(cfg, path, only_unit='u1', pin_firmware=True,
                               opener=lambda k, **kw: fake)
        assert reports[0].ok
        assert reports[0].firmware is None
        assert any('proves no version' in a for a in reports[0].actions)


def test_freeze_all_isolates_a_dead_unit():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'suite.yaml')
        with open(path, 'w') as f:
            f.write(MANIFEST.replace(
                "  - name: u1            # the mast IMU",
                "  - name: dead\n"
                "    module: nxs\n"
                "    links: [{transport: mock}]\n"
                "    sensors: [{driver: iam20680}]\n"
                "  - name: u1            # the mast IMU"))
        cfg = load_suite_config(path)
        alive = FakeUnit()
        switch_suite(cfg, SuiteState.load(os.path.join(tmp, 's.yaml')),
                    only_unit='u1', opener=lambda k, **kw: alive)
        alive.set_param('sample_rate', 500)
        units = {'dead': FakeUnit(alive=False), 'u1': alive}
        order = iter(['dead', 'u1'])
        reports = freeze_suite(cfg, path,
                               opener=lambda k, **kw: units[next(order)])
        assert not reports[0].ok and 'no response' in reports[0].error
        assert reports[1].ok and reports[1].changed
        assert 'sample_rate: 500' in open(path).read()


SCI_SERIAL = "2030355845315010003e0028"   # <digits>e<digits>: a YAML
                                          # scientific-notation lookalike


def test_freeze_round_trips_a_scientific_notation_serial(tmp_path):
    """A quoted UID that happens to spell <digits>e<digits> survives a
    freeze byte-for-byte — the exact corruption a float-resolving
    round-trip would inflict (…010003e… rewritten as …009987e…)."""
    path = str(tmp_path / 'suite.yaml')
    with open(path, 'w') as f:
        f.write(MANIFEST.replace('    sensors:',
                                 f'    serial: "{SCI_SERIAL}"\n    sensors:'))
    cfg = load_suite_config(path)
    fake = FakeUnit(serial=SCI_SERIAL)
    switch_suite(cfg, SuiteState.load(os.path.join(str(tmp_path), 's.yaml')),
                opener=lambda k, **kw: fake)
    fake.set_param('sample_rate', 500)
    reports = freeze_suite(cfg, path, only_unit='u1',
                           opener=lambda k, **kw: fake)
    assert reports[0].ok and reports[0].changed
    text = open(path).read()
    assert f'serial: "{SCI_SERIAL}"' in text
    assert '009987' not in text


def test_freeze_refuses_an_unquoted_float_lookalike_serial(tmp_path):
    """An unquoted <digits>e<digits> serial loads as a float in the
    round-trip parser — the digits are already lost, so freeze must
    refuse to write rather than persist the float64-rounded corpse."""
    import pytest

    from nxs.suite.schema import ManifestError

    path = str(tmp_path / 'suite.yaml')
    with open(path, 'w') as f:
        f.write(MANIFEST.replace('    sensors:',
                                 f'    serial: {SCI_SERIAL}\n    sensors:'))
    cfg = load_suite_config(path)   # plain parser reads it as a string
    fake = FakeUnit(serial=SCI_SERIAL)
    switch_suite(cfg, SuiteState.load(os.path.join(str(tmp_path), 's.yaml')),
                opener=lambda k, **kw: fake)
    fake.set_param('sample_rate', 500)
    with pytest.raises(ManifestError, match='quote'):
        freeze_suite(cfg, path, only_unit='u1', opener=lambda k, **kw: fake)
    assert f'serial: {SCI_SERIAL}' in open(path).read()   # file untouched


def test_freeze_refuses_a_driver_revision_mismatch():
    """A same-named driver serving a parameter set that differs from the
    host-compiled driver is a different revision. Freezing it would
    adopt keys a later switch silently drops, so freeze fails fast and
    leaves the manifest untouched."""

    class RevisionSkewedUnit(FakeUnit):
        def read_capabilities(self):
            caps = super().read_capabilities()
            caps.append({"name": "extra_knob", "current": 7})
            return caps

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'suite.yaml')
        with open(path, 'w') as f:
            f.write(MANIFEST)
        cfg = load_suite_config(path)
        fake = RevisionSkewedUnit()
        switch_suite(cfg, SuiteState.load(os.path.join(tmp, 'state.yaml')),
                    opener=lambda k, **kw: fake)
        before = open(path).read()
        reports = freeze_suite(cfg, path, only_unit='u1',
                               opener=lambda k, **kw: fake)
        assert not reports[0].ok
        assert 'different driver revision' in reports[0].error
        assert 'extra_knob' in reports[0].error
        assert open(path).read() == before
