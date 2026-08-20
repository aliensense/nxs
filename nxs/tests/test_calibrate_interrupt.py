"""An interrupted calibration releases the device and applies nothing.

A procedure holds the device's transfer session for its duration. Leaving
it held wedges every later command — a store read, and the firmware push
that would recover the unit — until the device's stale window expires.

`cal_mag_stop` also releases, but it solves and applies on the way out, so
using it here would commit a calibration the operator just cancelled.
"""
from argparse import Namespace

import pytest

from nxs.calibrate import cmd_gyro, cmd_mag
from nxs.transports.mock import MockTransport


class _Interrupted(MockTransport):
    """A device whose progress poll raises, and that records how the verb
    ended the procedure. Built on the real mock so the verbs run their
    whole preamble — capability, driver and panel checks included."""

    def __init__(self, raise_on_poll):
        super().__init__()
        self._raise = raise_on_poll
        self.stopped = False

    def cal_mag_stop(self):
        self.stopped = True
        super().cal_mag_stop()

    def read_cal_progress(self):
        raise self._raise


def _args():
    return Namespace(transport='mock', cal_cmd='mag', no_persist=True)


@pytest.mark.parametrize("verb", [cmd_mag, cmd_gyro])
@pytest.mark.parametrize("raised", [KeyboardInterrupt(), OSError("bus glitch")])
def test_an_interrupted_procedure_releases_the_device(verb, raised):
    t = _Interrupted(raised)
    with pytest.raises(type(raised)):
        verb(t, _args())
    assert t.cal_aborted, "the session stayed held after the interrupt"


def test_an_interrupt_never_commits_a_calibration(capsys):
    """`stop` is the verb that solves and applies. An interrupt must not
    reach it: a sweep cut short after enough rotation would otherwise
    commit a record the operator just cancelled."""
    t = _Interrupted(KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        cmd_mag(t, _args())
    assert not t.stopped, "the interrupt solved and applied a record"
    assert t.cal_aborted
    assert not t.calibration_persisted
    assert "nothing applied" in capsys.readouterr().err


def test_a_release_that_itself_fails_does_not_mask_the_interrupt(capsys):
    """The original exception is what the operator needs to see; a failing
    release on the way out must not replace it."""
    class _Stuck(_Interrupted):
        def cal_abort(self):
            raise OSError("device not answering")

    t = _Stuck(KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        cmd_mag(t, _args())
    assert "may still hold the calibration session" in capsys.readouterr().err
