"""Poll-pacing math for the I2C record transport: the rate-derived
default oversamples the single-slot record at 5x; an explicit output
rate stays the caller's exact cadence."""
from nxs.transports.i2c import NxsI2cTransport


def _bare_transport():
    t = NxsI2cTransport.__new__(NxsI2cTransport)
    t._poll_hz = None
    t._poll_interval = 0.0
    t._sample_period = 0.0
    t._next_due = 0.0
    t._read_record = lambda n: (0, 0, b"")
    t.read_sample_size = lambda: 14
    return t


def test_default_poll_oversamples_at_five_times_the_rate():
    t = _bare_transport()
    t.get_param = lambda name: {"current": 200}
    t._arm_stream(1)
    assert t._sample_period == 1.0 / 200
    assert t._poll_interval == 1.0 / 1000


def test_gnss_rate_param_paces_the_poll():
    t = _bare_transport()
    def get_param(name):
        if name == "sample_rate":
            raise KeyError(name)
        return {"current": 1}
    t.get_param = get_param
    t._arm_stream(1)
    assert t._sample_period == 1.0
    assert t._poll_interval == 1.0 / 5


def test_explicit_output_rate_is_exact():
    t = _bare_transport()
    t.get_param = lambda name: {"current": 200}
    t._poll_hz = 50
    t._arm_stream(1)
    assert t._poll_interval == 1.0 / 50


def test_arm_race_zero_length_heals_on_first_sample():
    # Arming can race the image load: SAMPLE_SIZE reads 0 (no sample yet).
    # The cached length must refresh when the first live sample lands, so
    # the row never streams empty records.
    t = _bare_transport()
    t.get_param = lambda name: {"current": 10}
    state = {"seq": 7, "size": 0}
    t.read_sample_size = lambda: state["size"]
    t._read_record = lambda n: (state["seq"], 111, b"\xAA\xBB\xCC\xDD\xEE"[:n])

    t._arm_stream(1)
    assert t._record_data_len == 0

    state["seq"] = 8
    state["size"] = 5
    t._next_due = 0.0
    seq, raw, _ = t._next_raw(timeout=0.1)
    assert seq == 8
    assert raw == b"\xAA\xBB\xCC\xDD\xEE"
    assert t._record_data_len == 5
