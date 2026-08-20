"""NxsClient conformance — every transport satisfies the contract, the
capability matrix pins each transport's typed capabilities (no
`hasattr`/`args.transport` sniffing), and the shared streaming/decode
layer works.

The decode-at-the-SDK-boundary test is the same honesty bar as
test_universal_decode, raised to the public surface: a fake I2C device
serves descriptors + samples, and `iter_samples()` hands back decoded
physical values with no driver file in the loop.
"""
import struct
import time

import pytest

from nxs.client import (
    NxsClient, Sample, SupportsBitTiming, SupportsCanTermination, SupportsTimeSync,
    SupportsCommissioning, SupportsEgressDecimation, SupportsIdentify,
    SupportsRecovery, SupportsSlotPeek)
from nxs.transports.cyphal_control import CyphalControlClient
from nxs.transports import open_client
from nxs.transports.i2c import (
    NxsI2cTransport,
    REG_CMD, CMD_IDENTIFY,
    REG_WHO_AM_I, REG_SAMPLE_COUNT, REG_SAMPLE_SIZE, REG_NUM_OUTPUTS,
    REG_DESCRIPTOR_EPOCH, REG_OUTPUT_SELECT, REG_SAMPLE_DATA,
    REG_SEL_NAME_LEN, REG_SEL_NAME, REG_SEL_TYPE, REG_SEL_OUTPUT_SCALE,
    REG_SEL_OUTPUT_OFFSET, REG_SEL_OUTPUT_BYTE_ORDER, REG_SEL_UNIT_LEN,
    REG_SEL_OUTPUT_SEMANTIC, REG_SEL_OUTPUT_COUNT, WHO_AM_I_VALUE)
from nxs.transports.mock import MockTransport


class _MinimalI2cBus:
    """Smallest fake that satisfies probe and the streaming path."""

    def __init__(self):
        self._regs = bytearray(256)
        self._regs[REG_WHO_AM_I] = WHO_AM_I_VALUE

    def read_byte_data(self, addr, reg):
        return self._regs[reg]

    def write_byte_data(self, addr, reg, val):
        self._regs[reg] = val & 0xFF

    def read_word_data(self, addr, reg):
        return self._regs[reg] | (self._regs[reg + 1] << 8)

    def read_i2c_block_data(self, addr, reg, n):
        return list(self._regs[reg:reg + n])

    def write_i2c_block_data(self, addr, reg, data):
        self._regs[reg:reg + len(data)] = bytes(data)

    def close(self):
        pass


class _StreamingI2cBus(_MinimalI2cBus):
    """Serves one int16-BE 'accel_x' output descriptor and an
    incrementing sample, so the I2C SDK path can be exercised end to
    end: read_outputs → iter_samples → decoded values."""

    def __init__(self):
        super().__init__()
        self._regs[REG_SAMPLE_SIZE] = 2
        self._regs[REG_NUM_OUTPUTS] = 1
        self._regs[REG_DESCRIPTOR_EPOCH] = 1
        self._fill_accel_x()
        self._t0 = time.monotonic()

    def _fill_accel_x(self):
        name = b"accel_x"
        self._regs[REG_SEL_NAME_LEN] = len(name)
        self._regs[REG_SEL_NAME:REG_SEL_NAME + len(name)] = name
        self._regs[REG_SEL_TYPE] = 2  # int16
        self._regs[REG_SEL_OUTPUT_SCALE:REG_SEL_OUTPUT_SCALE + 4] = \
            struct.pack('<f', 0.5)
        self._regs[REG_SEL_OUTPUT_OFFSET:REG_SEL_OUTPUT_OFFSET + 4] = \
            struct.pack('<f', 0.0)
        self._regs[REG_SEL_OUTPUT_BYTE_ORDER] = 0  # big
        self._regs[REG_SEL_UNIT_LEN] = 0
        self._regs[REG_SEL_OUTPUT_SEMANTIC] = 1  # accel_x
        self._regs[REG_SEL_OUTPUT_COUNT:REG_SEL_OUTPUT_COUNT + 2] = b'\x00\x00'

    def _count(self):
        # Advances on wall-clock (~50 Hz), not per read, mirroring a
        # runner that outpaces neither the host nor itself.
        return (int((time.monotonic() - self._t0) / 0.02) + 1) & 0x7FFF

    def read_word_data(self, addr, reg):
        if reg == REG_SAMPLE_COUNT:
            return self._count()
        return super().read_word_data(addr, reg)

    def read_i2c_block_data(self, addr, reg, n):
        if reg == REG_SAMPLE_DATA:
            # The firmware latch: stage one coherent record for this
            # transaction — latch time, acquisition time, seq, then the
            # staged sample raw = 100 + seq, so a decoder can be checked
            # against `count` independently of re-reading the bytes.
            count = self._count()
            now_us = 1_000_000 + count * 20_000
            rec = struct.pack('<QQH', now_us, now_us - 1_000, count)
            rec += struct.pack('>h', 100 + count)
            self._regs[REG_SAMPLE_DATA:REG_SAMPLE_DATA + len(rec)] = rec
        return super().read_i2c_block_data(addr, reg, n)


def _i2c(bus):
    return NxsI2cTransport(_bus_obj=bus)


# ── Conformance ────────────────────────────────────────────────

def test_i2c_and_mock_instantiate_against_the_abc():
    # The ABC fails construction if any abstract method is missing.
    assert isinstance(_i2c(_MinimalI2cBus()), NxsClient)
    assert isinstance(MockTransport(), NxsClient)


# ── The capability matrix ─────────────────────────────────────
# One row per transport class, one column per capability. This is the
# transport-symmetry contract: swapping the transport must be invisible
# to the SDK's upper layers, with each False below being a documented,
# wire-inherent exception (host-interface spec, "Capability matrix").
# A transport drifting from its declared column fails here, not in the
# field. Class-level issubclass checks need no bus or pycyphal session.

CAPABILITY_MATRIX = {
    NxsI2cTransport: {
        SupportsIdentify: True,
        SupportsSlotPeek: True,
        SupportsCommissioning: True,
        SupportsRecovery: True,
        SupportsEgressDecimation: False,  # polled stream, host-paced
        SupportsBitTiming: True,
        SupportsCanTermination: True,
        SupportsTimeSync: True,
    },
    CyphalControlClient: {
        SupportsIdentify: True,
        SupportsSlotPeek: True,
        SupportsCommissioning: True,
        SupportsRecovery: True,
        SupportsEgressDecimation: True,  # pushed stream, device-thinned
        SupportsBitTiming: True,
        SupportsCanTermination: True,
        SupportsTimeSync: True,
    },
    MockTransport: {
        SupportsIdentify: True,
        SupportsSlotPeek: True,
        SupportsCommissioning: True,
        SupportsRecovery: True,
        SupportsEgressDecimation: False,
        SupportsBitTiming: True,
        SupportsCanTermination: True,
        SupportsTimeSync: True,
    },
}


def test_capability_matrix_holds_for_every_transport():
    for transport, row in CAPABILITY_MATRIX.items():
        for capability, expected in row.items():
            assert issubclass(transport, capability) is expected, (
                f"{transport.__name__} vs {capability.__name__}: "
                f"expected {expected}")


def test_every_transport_satisfies_the_base_contract():
    # NxsClient is the swap-invariant core: identity reads, decimation
    # knobs, params, store, DFU, streaming. issubclass proves no
    # abstract method is missing without touching hardware.
    for transport in CAPABILITY_MATRIX:
        assert issubclass(transport, NxsClient)


def test_identify_is_typed_and_reaches_the_device():
    """`identify` is a typed capability: mock and I2C support it, and
    the I2C call lands as a command-register write."""

    mock = MockTransport()
    assert isinstance(mock, SupportsIdentify)
    mock.identify()
    assert mock.identify_calls == 1

    bus = _MinimalI2cBus()
    t = _i2c(bus)
    assert isinstance(t, SupportsIdentify)
    t.identify()
    assert bus.read_byte_data(0, REG_CMD) == CMD_IDENTIFY


def test_open_client_factory():
    assert isinstance(open_client("mock"), MockTransport)
    with pytest.raises(ValueError):
        open_client("zigbee")


def test_i2c_requires_a_bus():
    # bus=None with no injected _bus_obj is a clear error, not a
    # cryptic int(None) TypeError downstream.
    with pytest.raises(ValueError):
        NxsI2cTransport()


def test_set_output_rate_rejects_nonpositive():
    # The uncapped operator-set rate contract only holds for a positive
    # rate. Base (push transports) and the I²C override both reject <= 0
    # rather than silently defaulting or setting a negative poll interval.
    for bad in (0, -5):
        with pytest.raises(ValueError):
            MockTransport().set_output_rate(bad)
        with pytest.raises(ValueError):
            _i2c(_MinimalI2cBus()).set_output_rate(bad)


def test_i2c_poll_hz_constructor_rejects_nonpositive():
    for bad in (0, -10):
        with pytest.raises(ValueError):
            NxsI2cTransport(_bus_obj=_MinimalI2cBus(), poll_hz=bad)
    # None (use advertised/default) and a positive rate are accepted.
    assert _i2c(_MinimalI2cBus())._poll_hz is None
    NxsI2cTransport(_bus_obj=_MinimalI2cBus(), poll_hz=50)


# ── Shared streaming / decode layer ────────────────────────────

def test_mock_streams_via_iterator_and_callback():
    m = MockTransport()
    m.upload_image(bytes([0x61, 14]))  # SET_SAMPLE_SIZE 14
    m.vm_run()

    # Pull model.
    seen = []
    for s in m.iter_samples(timeout=0.1):
        assert isinstance(s, Sample)
        seen.append(s)
        if len(seen) >= 3:
            m.stop_stream()
            break
    assert [s.count for s in seen] == [1, 2, 3]

    # Push model: on_sample + spin, stopped from within a handler.
    got = []

    def handler(sample):
        got.append(sample)
        if len(got) >= 2:
            m.stop_spin()

    m.vm_run()
    m.on_sample(handler)
    m.spin(timeout=0.1)
    assert len(got) >= 2
    # spin() disarms the stream on exit — no lingering armed stream.
    assert m._streaming is False


def test_poll_sample_is_nonblocking_when_idle():
    m = MockTransport()  # not running → no samples
    assert m.poll_sample(timeout=0.0) is None


def test_stop_stream_is_idempotent():
    # stop on a never-started stream is a no-op, not an error or a
    # spurious disarm command.
    m = MockTransport()
    m.stop_stream()
    assert m._streaming is False


def test_iter_samples_disarms_on_break():
    # The documented `for s in iter_samples(): … break` idiom must not
    # leave the stream armed — else a push transport keeps emitting and
    # its RX queue grows unbounded. The finally in iter_samples disarms.
    m = MockTransport()
    m.upload_image(bytes([0x61, 14]))  # SET_SAMPLE_SIZE 14
    m.vm_run()
    for _ in m.iter_samples(timeout=0.1):
        break
    assert m._streaming is False


def test_iter_samples_disarms_on_exception():
    m = MockTransport()
    m.upload_image(bytes([0x61, 14]))
    m.vm_run()
    with pytest.raises(RuntimeError):
        for _ in m.iter_samples(timeout=0.1):
            raise RuntimeError("boom")
    assert m._streaming is False


class _DisarmRaisesTransport(MockTransport):
    """Mock whose stream disarm fails — mimics a serial link that NACKs
    or times out the stop-stream frame during teardown."""

    def _disarm_stream(self):
        raise RuntimeError("disarm failed on a dead link")


def test_iter_samples_cleanup_does_not_mask_loop_error():
    # The finally's disarm can itself raise on a dead link; it must be
    # best-effort so it never supersedes the loop's original exception
    # (matching spin() and host teardown). A no-op disarm can't
    # surface this — the disarm has to actually fail.
    t = _DisarmRaisesTransport()
    t.upload_image(bytes([0x61, 14]))
    t.vm_run()
    with pytest.raises(ValueError, match="boom"):
        for _ in t.iter_samples(timeout=0.1):
            raise ValueError("boom")
    assert t._streaming is False


def test_i2c_iter_samples_decodes_from_device_descriptors():
    """The SDK boundary: a host with no driver file streams decoded
    physical values, using only what the device serves."""
    t = _i2c(_StreamingI2cBus())
    out = []
    for s in t.iter_samples(timeout=0.2):
        out.append(s)
        if len(out) >= 3:
            t.stop_stream()
            break

    assert all(isinstance(s, Sample) for s in out)
    # Independent oracle: the fake stages raw = 100 + count, descriptor
    # scale = 0.5, so the decoded value is (100 + count) * 0.5 derived
    # from s.count — not by re-decoding s.raw. A missing scale, raw-only
    # output, or a count/data mismatch all fail this.
    for s in out:
        assert s.values["accel_x"] == pytest.approx((100 + s.count) * 0.5)
    # Counts strictly increase — no duplicated or stale sample.
    counts = [s.count for s in out]
    assert counts == sorted(set(counts))


class _StaleRecordBus(_MinimalI2cBus):
    """A load boundary just cleared SAMPLE_COUNT, but the record window
    still holds the previous driver's last sample."""

    def __init__(self):
        super().__init__()
        self._regs[REG_SAMPLE_SIZE] = 2
        self.stage_record(57)

    def stage_record(self, seq):
        rec = struct.pack('<QQH', 5_000_000, 4_999_000, seq)
        rec += struct.pack('>h', 100 + seq)
        self._regs[REG_SAMPLE_DATA:REG_SAMPLE_DATA + len(rec)] = rec


def test_arm_baselines_from_record_seq_not_sample_count():
    """The poll cursor must baseline from the record's own seq — the
    field _next_raw compares. SAMPLE_COUNT reads 0 after a load
    boundary while the record keeps the old driver's seq; a register
    baseline would emit that stale record as a fresh sample."""
    bus = _StaleRecordBus()
    t = _i2c(bus)
    assert t.poll_sample(timeout=0.05) is None  # stale record: no phantom
    bus.stage_record(58)                        # the next real sample
    s = t.poll_sample(timeout=0.5)
    assert s is not None
    assert s.count == 58
    assert s.timestamp_us == 4_999_000


def test_top_level_exports_are_the_canonical_surface():
    """`from nxs import open_client, NxsClient` is the documented entry
    point (host-interface §9.1); the deep module paths are layout, not
    contract. The DSL `Sample` keeps the top-level name — the decoded
    stream sample stays at nxs.client.Sample to avoid aliasing two
    different types under one import."""
    import nxs
    from nxs.transports import open_client as deep_factory

    assert nxs.open_client is deep_factory
    assert nxs.NxsClient is NxsClient
    assert "open_client" in nxs.__all__ and "NxsClient" in nxs.__all__
    from nxs.compiler import Sample as DslSample
    assert nxs.Sample is DslSample


# ── Descriptor refresh across a driver swap ───────────────────────────

class _RefreshDuck:
    """Just the two methods _refreshed_fields consults."""

    def __init__(self, tokens, fetches):
        self._tokens = list(tokens)
        self._fetches = list(fetches)

    def _descriptor_token(self):
        return self._tokens.pop(0)

    def _fields_for(self, _token):
        return self._fetches.pop(0)


def test_refreshed_fields_goes_raw_only_while_swap_unresolved():
    """A token change whose descriptor fetch is transiently unreadable
    must neither keep decoding with the previous driver's map (the
    stale-map misgrade) nor commit the new token (which would pin the
    failure); it decodes raw-only and retries on the next sample."""
    from nxs.client import NxsClient

    old_fields = [{"name": "version"}]
    new_fields = [{"name": "verdict"}]
    duck = _RefreshDuck(tokens=[2, 2], fetches=[None, new_fields])

    token, fields = NxsClient._refreshed_fields(duck, 1, old_fields)
    assert (token, fields) == (1, [])

    token, fields = NxsClient._refreshed_fields(duck, token, fields)
    assert token == 2
    assert fields == new_fields


def test_xfer_reason_map_names_the_retryable_drop():
    # A queue-dropped announce/begin resolves to EAGAIN(11); the reason map
    # must give retry guidance, not a bare "device error code 11".
    from nxs.client import XFER_ERR_REASON, err_reason
    assert "retry" in err_reason(11, XFER_ERR_REASON).lower()
    assert "code 11" not in err_reason(11, XFER_ERR_REASON)
