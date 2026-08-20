"""Calibration record contract + the host decode twin (apply_calibration)."""
import math
import struct

import json
import pathlib

import pytest

from nxs._generated_constants import Calibration as CalConstants
from nxs.client import (CalibrationRecord, DeviceRefused, rotation_code,
                        rotation_name)
from nxs.descriptor import IDENTITY_M, apply_calibration, fnv1a32, parse_sample
from nxs.transports.mock import MockTransport

# The IMU field set every twin test decodes: raw counts scale to the same
# physical values the firmware mapper test uses.
FIELDS = [
    {'name': 'accel_x', 'type': 'int16', 'byte_order': 'big', 'byte_off': 0,
     'scale': 0.001, 'offset': 0.0, 'semantic': 1},
    {'name': 'accel_y', 'type': 'int16', 'byte_order': 'big', 'byte_off': 2,
     'scale': 0.001, 'offset': 0.0, 'semantic': 2},
    {'name': 'accel_z', 'type': 'int16', 'byte_order': 'big', 'byte_off': 4,
     'scale': 0.001, 'offset': 0.0, 'semantic': 3},
    {'name': 'gyro_x', 'type': 'int16', 'byte_order': 'big', 'byte_off': 6,
     'scale': 0.0001, 'offset': 0.0, 'semantic': 4},
    {'name': 'gyro_y', 'type': 'int16', 'byte_order': 'big', 'byte_off': 8,
     'scale': 0.0001, 'offset': 0.0, 'semantic': 5},
    {'name': 'gyro_z', 'type': 'int16', 'byte_order': 'big', 'byte_off': 10,
     'scale': 0.0001, 'offset': 0.0, 'semantic': 6},
    {'name': 'temp', 'type': 'int16', 'byte_order': 'big', 'byte_off': 12,
     'scale': 0.01, 'offset': 273.15, 'semantic': 10},
]
SAMPLE = struct.pack('>7h', 1000, 2000, 3000, 100, 200, 300, 2500)


def test_record_roundtrip():
    rec = CalibrationRecord(orientation=rotation_code('YAW_90'),
                            encoder_zero=-0.75,
                            driver_tags=(0xDEADBEEF, 0, 0),
                            encoder_tag=0x12345678)
    data = rec.pack()
    assert len(data) == CalConstants.RECORD_SIZE
    assert data[0] == CalConstants.RECORD_VERSION
    out = CalibrationRecord.unpack(data)
    assert out == rec


def test_record_rejects_bad_version_and_size():
    data = bytearray(CalibrationRecord().pack())
    data[0] += 1
    with pytest.raises(ValueError):
        CalibrationRecord.unpack(bytes(data))
    with pytest.raises(ValueError):
        CalibrationRecord.unpack(bytes(data[:10]))


def test_fnv1a32_matches_firmware():
    assert fnv1a32('') == 0x811C9DC5
    assert fnv1a32('a') == 0xE40C292C


def test_rotation_names():
    assert rotation_name(0) == 'NONE'
    assert rotation_code('yaw_90') == 1
    with pytest.raises(ValueError):
        rotation_code('YAW_45')


def test_rotation_matrices_are_proper():
    seen = set()
    for code, m in CalConstants.Rotation.MATRIX.items():
        assert len(m) == 9 and all(v in (-1, 0, 1) for v in m)
        det = (m[0] * (m[4] * m[8] - m[5] * m[7])
               - m[1] * (m[3] * m[8] - m[5] * m[6])
               + m[2] * (m[3] * m[7] - m[4] * m[6]))
        assert det == 1, f'rotation {code} is not proper'
        seen.add(m)
    assert len(seen) == 24


def test_identity_record_is_noop():
    plain = parse_sample(SAMPLE, FIELDS)
    calibrated = parse_sample(SAMPLE, FIELDS, calibration=CalibrationRecord())
    assert calibrated == plain


FIXTURE = (pathlib.Path(__file__).resolve().parents[3]
           / 'tests' / 'fixtures' / 'calibration-vectors.json')


def test_twin_matches_firmware_math():
    """The design contract: a host decoding RawSample against the served
    descriptors and record reproduces every SI subject exactly.

    The expectation comes from the firmware applier itself, emitted by
    `tests/posix/test_standard_subject_mapper.cpp`. Recomputing it here from a
    hand-written formula — what this test used to do — proves only that the
    formula was typed twice; both suites could drift together and stay green.
    """
    if not FIXTURE.exists():
        pytest.skip(f"{FIXTURE} not generated — run the posix test suite first")
    vectors = json.loads(FIXTURE.read_text())

    rec = CalibrationRecord(orientation=rotation_code(vectors['orientation']))
    rec = rec.replace_vector(0, tuple(vectors['accel_m']),
                             tuple(vectors['accel_b']), 0)
    # The gyro bias the same firmware case applies; it nulls the streamed
    # rates, so the rotation acts on a zero vector.
    rec = rec.replace_vector(1, IDENTITY_M, (-0.01, -0.02, -0.03), 0)

    values = parse_sample(bytes(vectors['raw']), FIELDS, calibration=rec)

    for axis, expected in zip('xyz', vectors['expect_accel']):
        assert values[f'accel_{axis}'] == pytest.approx(expected, abs=1e-5), \
            f"decode twin disagrees with the firmware applier on accel_{axis}"
    for axis in ('gyro_x', 'gyro_y', 'gyro_z'):
        assert values[axis] == pytest.approx(0.0, abs=1e-9)
    assert values['temp'] == pytest.approx(298.15)


def test_driver_tag_guard():
    rec = CalibrationRecord(orientation=rotation_code('YAW_90'))
    from nxs.descriptor import driver_tag
    bound = driver_tag('iim20670', 0, 0x1D)
    rec = rec.replace_vector(0, (2.0, 0, 0, 0, 3.0, 0, 0, 0, 4.0),
                             (0.1, 0.2, 0.3), bound)

    # Mismatched sensor: the affine is off, the mount rotation stays.
    guarded = parse_sample(SAMPLE, FIELDS, calibration=rec,
                           active_tag=driver_tag('mc6470', 0, 0x1D))
    assert guarded['accel_x'] == pytest.approx(-2.0)  # -y of (1, 2, 3)
    assert guarded['accel_y'] == pytest.approx(1.0)
    # Matching driver: the full affine applies.
    active = parse_sample(SAMPLE, FIELDS, calibration=rec, active_tag=bound)

    # The same driver at a second address is a different sensor.
    moved = parse_sample(SAMPLE, FIELDS, calibration=rec,
                         active_tag=driver_tag('iim20670', 0, 0x1E))
    assert moved['accel_x'] == pytest.approx(-2.0)
    assert active['accel_x'] == pytest.approx(-(3.0 * 2 + 0.2))
    assert active['accel_y'] == pytest.approx(2.0 * 1 + 0.1)


def test_unknown_rotation_code_decodes_raw():
    # A newer device may serve an orientation code this tool doesn't know
    # (the vocabulary is append-only, same record version). The post-pass
    # must disable itself — raw is honest, a wrong rotation is not — and
    # never crash the decode.
    rec = CalibrationRecord(orientation=200)
    rec = rec.replace_vector(0, (2.0, 0, 0, 0, 2.0, 0, 0, 0, 2.0),
                             (0.5, 0.5, 0.5), 0)
    values = parse_sample(SAMPLE, FIELDS, calibration=rec)
    assert values == parse_sample(SAMPLE, FIELDS)


def test_partial_vector_passes_through_raw():
    # Only accel_x decodes (the sample ends after 2 bytes): the affine and
    # rotation must not apply — x stays in place, no bias fabricated.
    rec = CalibrationRecord(orientation=rotation_code('YAW_90'))
    rec = rec.replace_vector(0, (2.0, 0, 0, 0, 2.0, 0, 0, 0, 2.0),
                             (0.5, 0.5, 0.5), 0)
    raw = struct.pack('>h', 1000)  # accel_x -> 1.0; y/z truncated
    values = parse_sample(raw, FIELDS, calibration=rec)
    assert values['accel_x'] == pytest.approx(1.0)
    assert 'accel_y' not in values
    # The full-sample case still calibrates (guards the guard).
    full = parse_sample(SAMPLE, FIELDS, calibration=rec)
    assert full['accel_x'] != pytest.approx(1.0)


def test_encoder_zero_wraps():
    fields = [{'name': 'angle', 'type': 'int32', 'byte_order': 'little',
               'byte_off': 0, 'scale': 1e-3, 'offset': 0.0, 'semantic': 23}]
    raw = struct.pack('<i', 6000)  # 6.0 rad
    rec = CalibrationRecord(encoder_zero=1.0)
    values = parse_sample(raw, fields, calibration=rec)
    assert values['angle'] == pytest.approx(7.0 - math.tau)

    rec.encoder_tag = fnv1a32('as5047d')
    from nxs.descriptor import driver_tag
    guarded = parse_sample(raw, fields, calibration=rec,
                           active_tag=driver_tag('mc6470', 0, 0x1D))
    assert guarded['angle'] == pytest.approx(6.0)


def test_mock_transport_calibration():
    t = MockTransport()
    assert t.read_calibration() == CalibrationRecord()
    rec = CalibrationRecord(orientation=rotation_code('ROLL_180'))
    t.write_calibration(rec, persist=True)
    assert t.read_calibration().orientation == rotation_code('ROLL_180')
    assert t.calibration_persisted
    t.set_orientation(rotation_code('NONE'), persist=False)
    assert t.read_calibration().orientation == 0
    assert not t.calibration_persisted


def test_stream_reloads_when_the_calibration_epoch_moves(monkeypatch):
    """A long-running calibrated stream follows a record that moves with
    no descriptor change — a boot-auto solve, an on-device procedure, or
    a write from the other transport."""
    import nxs.client as client_mod
    monkeypatch.setattr(client_mod, 'CAL_REFRESH_S', 0.0)
    m = MockTransport()
    m.upload_image(bytes([0x61, 14]))
    m.vm_run()
    it = m.iter_samples(timeout=0.1)
    first = next(it)

    # The bank moves underneath the stream: +100 m/s² on the accel bucket.
    rec = CalibrationRecord(b=((0.0, 0.0, 100.0), (0.0, 0.0, 0.0),
                               (0.0, 0.0, 0.0)))
    m.write_calibration(rec, persist=False)

    second = next(it)
    m.stop_stream()
    assert second.values["accel_z"] - first.values["accel_z"] > 50.0


def test_stream_first_epoch_observation_reloads(monkeypatch):
    """The bank can move between the startup record read and the first
    successful epoch probe; adopting that epoch as a baseline without
    the record would pin the stale affine forever."""
    import nxs.client as client_mod
    monkeypatch.setattr(client_mod, 'CAL_REFRESH_S', 0.0)

    class _LateEpochMock(MockTransport):
        """The first probe fails, like a probe under a live session."""

        def __init__(self):
            super().__init__()
            self.probes = 0

        def read_cal_epoch(self) -> int:
            self.probes += 1
            if self.probes == 1:
                raise RuntimeError("session held")
            return super().read_cal_epoch()

    m = _LateEpochMock()
    m.upload_image(bytes([0x61, 14]))
    m.vm_run()
    it = m.iter_samples(timeout=0.1)
    first = next(it)   # loads the identity record; the probe fails

    rec = CalibrationRecord(b=((0.0, 0.0, 100.0), (0.0, 0.0, 0.0),
                               (0.0, 0.0, 0.0)))
    m.write_calibration(rec, persist=False)

    second = next(it)  # the first successful observation must reload
    m.stop_stream()
    assert second.values["accel_z"] - first.values["accel_z"] > 50.0


def test_iter_samples_silence_budget_ends_the_stream():
    """A muted sensor must end the loop through its own no-data path,
    not hang the acquisition forever."""
    import time as _time

    class _MutedMock(MockTransport):
        def _next_raw(self, timeout):
            _time.sleep(min(timeout, 0.01))
            return None

    m = _MutedMock()
    m.upload_image(bytes([0x61, 14]))
    m.vm_run()
    t0 = _time.monotonic()
    seen = list(m.iter_samples(timeout=0.02, max_silence_s=0.05))
    assert seen == []
    assert _time.monotonic() - t0 < 2.0


def test_mock_procedures_apply_like_the_device():
    """A mock procedure that reports success must also move the record
    and the epoch — a test observing no calibration change while the
    tool prints "applied" would be passing on a lie."""
    m = MockTransport()
    m.set_gyro_bias_counts((10, -5, 3))
    epoch0 = m.read_cal_epoch()

    m.cal_gyro()
    while m.read_cal_progress() != (0, 0, 0):
        pass
    rec = m.read_calibration()
    assert rec.b[1] == (-0.01, 0.005, -0.003)
    assert m.read_cal_epoch() != epoch0

    epoch1 = m.read_cal_epoch()
    m.cal_mag_start()
    m.cal_mag_stop()
    rec = m.read_calibration()
    assert rec.m[2] != IDENTITY_M
    assert m.read_cal_epoch() != epoch1


def test_mock_calibration_value_semantics():
    # Mirrors real transports' pack/unpack: mutating a read or written
    # record must not change device state without write_calibration.
    t = MockTransport()
    rec = CalibrationRecord(orientation=rotation_code('YAW_90'))
    t.write_calibration(rec)
    rec.orientation = rotation_code('ROLL_180')
    assert t.read_calibration().orientation == rotation_code('YAW_90')
    peek = t.read_calibration()
    peek.encoder_zero = 9.9
    assert t.read_calibration().encoder_zero == 0.0


def test_i2c_unstable_readback_is_a_transport_error():
    # A record that changes on every read means the re-read-until-stable
    # passes never agree — a host coherence failure, not a device verdict:
    # RuntimeError, never DeviceRefused (whose errnos carry calibration
    # meanings like EAGAIN = coverage).
    from nxs.client import DeviceRefused
    from nxs.transports.i2c import NxsI2cTransport

    class ChangingRecordBus:
        def __init__(self):
            self.n = 0
            self.regs = {}

        def read_byte_data(self, addr, reg):
            # Echo register writes (no session holds the mux here) so the
            # mode-claim readback succeeds and the stability loop runs.
            return self.regs.get(reg, 0)

        def write_byte_data(self, addr, reg, val):
            self.regs[reg] = val

        def read_i2c_block_data(self, addr, reg, count):
            self.n += 1
            return [self.n & 0xFF] * count  # every read differs

    t = NxsI2cTransport(_bus_obj=ChangingRecordBus())
    with pytest.raises(RuntimeError) as excinfo:
        t.read_calibration()
    assert not isinstance(excinfo.value, DeviceRefused)


def test_is_still_gate():
    from nxs.calibrate import ACCEL_STILL_THRESHOLD, is_still
    th = ACCEL_STILL_THRESHOLD

    # A quiet window near 1 g: noise well under the threshold.
    still = [(0.1 + 0.01 * (i % 3), -0.2, 9.81 - 0.01 * (i % 2))
             for i in range(50)]
    assert is_still(still, th)

    # Slow drift: each step tiny, but the endpoints span > threshold —
    # exactly the motion p2p catches and a std metric under-reports.
    ramp = [(0.1, -0.2 + i * (1.5 * th / 50), 9.81) for i in range(50)]
    assert not is_still(ramp, th)

    # One spike vetoes the window (conservative: delays capture only).
    spiked = list(still)
    spiked[25] = (0.1, -0.2, 9.81 + 2 * th)
    assert not is_still(spiked, th)

    # An empty window proves nothing.
    assert not is_still([], th)


def test_on_device_gyro_trigger_persists():
    from argparse import Namespace
    from nxs.calibrate import cmd_calibrate
    t = MockTransport()
    rc = cmd_calibrate(t, Namespace(transport='mock', cal_cmd='gyro',
                                    no_persist=False))
    assert rc == 0
    assert t.calibration_persisted


def test_on_device_mag_trigger_auto_stops():
    from argparse import Namespace
    from nxs.calibrate import cmd_calibrate
    t = MockTransport()
    rc = cmd_calibrate(t, Namespace(transport='mock', cal_cmd='mag',
                                    no_persist=True))
    assert rc == 0
    assert not t.calibration_persisted


def test_on_device_mag_refusal_reported(monkeypatch, capsys):
    """EAGAIN keeps the collection open on the device, so the verb keeps
    turning and asks again; it gives up at the watch deadline and uploads
    nothing. Paced down here so the test does not sit out the operator's
    two minutes."""
    from argparse import Namespace
    import nxs.calibrate as cal
    from nxs.calibrate import cmd_calibrate
    monkeypatch.setattr(cal, "MAG_WATCH_TIMEOUT_S", 0.3)
    monkeypatch.setattr(cal, "MAG_RETRY_S", 0.05)
    monkeypatch.setattr(cal, "PROGRESS_POLL_S", 0.01)
    t = MockTransport()
    t.set_cal_stop_result(11)  # EAGAIN: insufficient coverage
    rc = cmd_calibrate(t, Namespace(transport='mock', cal_cmd='mag',
                                    no_persist=False))
    assert rc == 1
    assert not t.calibration_persisted
    err = capsys.readouterr().err
    assert "not yet" in err, "the operator was never told to keep rotating"
    assert "nothing uploaded" in err


def test_mag_retries_a_not_yet_verdict_and_then_succeeds(monkeypatch):
    """The progress byte is an estimate: coverage is decided after the fit.
    A verb that gave up on the first EAGAIN would throw away a rotation
    that a few more seconds of turning would have completed."""
    from argparse import Namespace
    import nxs.calibrate as cal
    from nxs.calibrate import cmd_calibrate
    monkeypatch.setattr(cal, "MAG_WATCH_TIMEOUT_S", 5.0)
    monkeypatch.setattr(cal, "MAG_RETRY_S", 0.01)
    monkeypatch.setattr(cal, "PROGRESS_POLL_S", 0.01)

    class RefusesTwice(MockTransport):
        def __init__(self):
            super().__init__()
            self.stop_calls = 0

        def cal_mag_stop(self):
            self.stop_calls += 1
            if self.stop_calls <= 2:
                raise DeviceRefused(11, "insufficient rotation coverage")
            return super().cal_mag_stop()

    t = RefusesTwice()
    rc = cmd_calibrate(t, Namespace(transport='mock', cal_cmd='mag',
                                    no_persist=False))
    assert rc == 0, "the third attempt succeeded but the verb still failed"
    assert t.stop_calls == 3
    assert t.calibration_persisted


def test_mag_refuses_without_magnetometer_outputs(capsys):
    """The verb reads the descriptor first: a driver with no mag vector is
    refused before any command reaches the device."""
    from argparse import Namespace
    from nxs.calibrate import cmd_calibrate

    class GyroOnly(MockTransport):
        def read_outputs(self):
            return [{'name': 'gyro_x', 'semantic': 4},
                    {'name': 'gyro_y', 'semantic': 5},
                    {'name': 'gyro_z', 'semantic': 6}]

        def cal_mag_start(self):
            raise AssertionError("CAL_MAG_START must not be issued")

    rc = cmd_calibrate(GyroOnly(), Namespace(transport='mock', cal_cmd='mag',
                                             no_persist=True))
    assert rc == 1
    assert 'no mag vector' in capsys.readouterr().err


def test_orientation_names_the_board_rotation():
    """ROLL_90 is the board's physical rotation: a rolled board (gravity
    on its +Y) decodes to level-vehicle, and a level board with the
    setting reads gravity on -Y — the transpose would swap the two."""
    cal = CalibrationRecord(orientation=rotation_code('ROLL_90'))
    fields = [f for f in FIELDS if f['name'].startswith('accel')]

    rolled = apply_calibration(
        {'accel_x': 0.0, 'accel_y': 9.8, 'accel_z': 0.0}, fields, cal)
    assert math.isclose(rolled['accel_z'], 9.8, abs_tol=1e-9)
    assert math.isclose(rolled['accel_y'], 0.0, abs_tol=1e-9)

    level = apply_calibration(
        {'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 9.8}, fields, cal)
    assert math.isclose(level['accel_y'], -9.8, abs_tol=1e-9)
    assert math.isclose(level['accel_z'], 0.0, abs_tol=1e-9)


def test_mock_descriptors_decode_its_own_stream():
    """The mock's read_outputs() must describe _fake_sample()'s packed
    layout: decoding one through the other yields the documented SI
    values (a drifted descriptor mislabels fields silently)."""
    t = MockTransport()
    values = parse_sample(t._fake_sample(), t.read_outputs())
    assert abs(values['accel_z'] - 9.81) < 0.1
    assert abs(values['temp'] - 25.0) < 0.1
    b = (values['mag_x'] ** 2 + values['mag_y'] ** 2
         + values['mag_z'] ** 2) ** 0.5
    assert abs(b - 45.8e-6) < 1.5e-6


def test_calibration_verdicts_render_as_text_not_bare_codes():
    """The device's libc numbers EBADMSG/ETIMEDOUT/ECANCELED 77/116/140 where
    glibc says 74/110/125. Written from the host's own errno the table unmapped
    exactly those three verdicts, and `err_reason` fell back to the raw code —
    a shaken boot-auto gyro pass reported `device error code 116`."""
    from nxs.client import CALIB_ERR_REASON, err_reason

    for code in (77, 116, 140):
        assert not err_reason(code, CALIB_ERR_REASON).startswith(
                'device error code'), f"{code} is unmapped"
    assert 'stillness' in err_reason(116, CALIB_ERR_REASON)
    assert 'sphere self-check' in err_reason(77, CALIB_ERR_REASON)
    assert 'cancelled' in err_reason(140, CALIB_ERR_REASON)


def test_tag_and_guard_match_the_firmware():
    """The guard is the safety half of the contract, and the half most likely
    to drift: it is hand-mirrored in Python while the affine math is generated
    on both sides from constants/calibration.yaml. Pin it against the same
    firmware-emitted vectors."""
    if not FIXTURE.exists():
        pytest.skip(f"{FIXTURE} not generated — run the posix test suite first")
    v = json.loads(FIXTURE.read_text())
    from nxs.descriptor import driver_tag

    inputs = v['tag_inputs']
    bound = driver_tag(inputs['name'], inputs['bus'], inputs['address'])
    assert bound == v['expect_tag'], "host driver_tag disagrees with the firmware"
    assert driver_tag(inputs['name'], inputs['bus'],
                      inputs['address'] + 1) == v['expect_tag_other_address']
    assert driver_tag(inputs['name'], inputs['bus'] + 1,
                      inputs['address']) == v['expect_tag_other_bus']

    rec = CalibrationRecord(driver_tags=(bound, 0, driver_tag('mc6470', 0, 0x1D)))
    got = [rec.bucket_guard(v_i, bound) for v_i in range(3)]
    assert got == v['expect_guards'], "host bucket_guard disagrees with the firmware"
