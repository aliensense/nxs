"""`nxs status` output contract for the fault-counter line: printed
when the transport serves the counters, silently absent when the
firmware lacks the registers (older images refuse the read)."""

import argparse

from nxs.cli import cmd_status
from nxs.client import SupportsFaultCounters


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


def _args():
    return argparse.Namespace(transport='i2c', bus='/dev/i2c-9', addr=0x30,
                              port=None, unit=None, remote_node_id=None)


def test_status_prints_fault_counters(capsys):
    assert cmd_status(_FaultFake(fail=False), _args()) == 0
    out = capsys.readouterr().out
    assert 'Faults:  7 I/O errors absorbed, 1 probe failures' in out


def test_status_survives_missing_fault_registers(capsys):
    assert cmd_status(_FaultFake(fail=True), _args()) == 0
    out = capsys.readouterr().out
    assert 'Faults' not in out
