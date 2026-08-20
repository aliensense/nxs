"""`iter_samples` composes the served calibration record — the host twin
of the device's SI tier — while the wizard's raw opt-out and a
foreign-driver tag leave the descriptor-tier values untouched."""
import struct

from nxs.client import CalibrationRecord, NxsClient
from nxs.descriptor import IDENTITY_M, fnv1a32

GYRO_FIELDS = [
    {'name': 'gyro_x', 'type': 'int16', 'byte_order': 'big',
     'byte_off': 0, 'scale': 0.001, 'semantic': 4},
    {'name': 'gyro_y', 'type': 'int16', 'byte_order': 'big',
     'byte_off': 2, 'scale': 0.001, 'semantic': 5},
    {'name': 'gyro_z', 'type': 'int16', 'byte_order': 'big',
     'byte_off': 4, 'scale': 0.001, 'semantic': 6},
]


class _StreamFake(NxsClient):
    """Minimal streaming client: three gyro fields, one sample per poll,
    and a served record carrying a +100 rad/s bias on the gyro bucket."""

    def __init__(self, tag=0):
        super().__init__()
        self._n = 0
        self._record = CalibrationRecord().replace_vector(
                1, IDENTITY_M, (100.0, 100.0, 100.0), tag)

    def read_outputs(self):
        return list(GYRO_FIELDS)

    def read_driver_name(self):
        return "Iam20680"

    def read_calibration(self):
        return self._record

    def _descriptor_token(self):
        return 1

    def _arm_stream(self, every_nth):
        pass

    def _disarm_stream(self):
        pass

    def _next_raw(self, timeout):
        self._n += 1
        return self._n, struct.pack('>3h', 10, 20, 30), None


for _m in list(_StreamFake.__abstractmethods__):
    setattr(_StreamFake, _m, lambda self, *a, **k: None)
_StreamFake.__abstractmethods__ = frozenset()


def _first(client, **kw):
    it = client.iter_samples(timeout=0.01, **kw)
    s = next(iter(it))
    it.close()
    return s


def test_stream_composes_served_record():
    s = _first(_StreamFake())
    assert s.values['gyro_x'] > 50


def test_raw_opt_out_skips_compose():
    s = _first(_StreamFake(), calibrated=False)
    assert s.values['gyro_x'] < 1


def test_foreign_driver_tag_is_guarded():
    s = _first(_StreamFake(tag=fnv1a32("OtherImu")))
    assert s.values['gyro_x'] < 1
