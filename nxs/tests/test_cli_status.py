"""`nxs status` output contract for the fault-counter line: printed
when the transport serves the counters, silently absent when the
firmware lacks the registers (older images refuse the read)."""

import argparse

from nxs.cli import cmd_status
from nxs.client import SupportsFaultCounters, SUPPORTED_PROTO_VERSION


class _StatusFake:
    def probe(self):
        return True

    def read_status(self):
        return 0x80

    def read_vm_state(self):
        return 1

    def read_error_code(self):
        return 0

    def read_sample_count(self):
        return 42

    def read_driver_name(self):
        return 'imu'

    def read_store_count(self):
        return 1

    def read_active_slot(self):
        return 0

    def read_runner_state(self):
        return 0

    def read_probe_retries(self):
        return 0

    def read_serial(self):
        return b''

    def read_fw_version(self):
        return None


class _FaultFake(_StatusFake, SupportsFaultCounters):
    def __init__(self, fail):
        self._fail = fail

    def read_io_err_count(self):
        if self._fail:
            raise RuntimeError('register missing')
        return 7

    def read_probe_failed_count(self):
        if self._fail:
            raise RuntimeError('register missing')
        return 1

    def read_drdy_coalesced_count(self):
        if self._fail:
            raise RuntimeError('register missing')
        return 3

    def read_ingress_reject_count(self):
        if self._fail:
            raise RuntimeError('register missing')
        return 2


def _args():
    return argparse.Namespace(transport='i2c', bus='/dev/i2c-9', addr=0x30,
                              port=None, unit=None, remote_node_id=None)


def test_version_gate_lets_the_repair_verbs_through():
    # push-fw and recover carry the firmware that ends a contract mismatch;
    # gating them would strand a fielded device with no upgrade path but a
    # J-Link. They warn and proceed; every other verb refuses.
    from nxs.cli import _require_supported_version

    class _Off:
        def interface_version(self):
            return SUPPORTED_PROTO_VERSION + 1

    # The gate refuses an off-contract device for every mutating verb...
    try:
        _require_supported_version(_Off())
    except SystemExit as e:
        assert (f"v{SUPPORTED_PROTO_VERSION + 1}" in str(e)
                and f"v{SUPPORTED_PROTO_VERSION}" in str(e))
    else:
        raise AssertionError("the gate must refuse an off-contract device")

    # ...and main() never calls it for probe / push-fw / recover, so those
    # reach the device and can repair the mismatch.
    from nxs.cli import VERSION_GATE_EXEMPT
    assert set(VERSION_GATE_EXEMPT) == {'probe', 'push-fw', 'recover'}


def test_status_prints_fault_counters(capsys):
    assert cmd_status(_FaultFake(fail=False), _args()) == 0
    out = capsys.readouterr().out
    assert 'Faults:  7 I/O errors absorbed, 1 probe failures' in out
    assert '3 samples missed (VM busy at DRDY)' in out
    assert '2 commands rejected (bus contention)' in out


def test_status_survives_missing_fault_registers(capsys):
    assert cmd_status(_FaultFake(fail=True), _args()) == 0
    out = capsys.readouterr().out
    assert 'Faults' not in out


class _HealthyFake(_FaultFake):
    def __init__(self):
        super().__init__(fail=False)

    def read_io_err_count(self): return 0
    def read_probe_failed_count(self): return 0
    def read_drdy_coalesced_count(self): return 0
    def read_ingress_reject_count(self): return 0


def test_status_is_silent_when_no_faults(capsys):
    # A healthy device prints no Faults block — a fault line always means
    # something happened.
    assert cmd_status(_HealthyFake(), _args()) == 0
    assert 'Faults' not in capsys.readouterr().out
