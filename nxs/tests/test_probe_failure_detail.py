"""A failed probe says why, when the transport knows why.

"NOT FOUND" is the same output whether the device is absent, the bus
refused the read, or the interface never put the question on the wire —
and only the last of those is fixed by checking cabling. The reason is
held by the transport that saw the error and was being discarded there.
"""
import asyncio
import errno
import threading

from nxs.transports.cyphal_control import CyphalControlClient
from nxs.transports.i2c import NxsI2cTransport


def _cancel_all_and_stop(loop):
    for task in asyncio.all_tasks(loop):
        task.cancel()
    loop.stop()


class _Loop:
    """A live event loop bound to a stopped client, torn down without
    leaving pending tasks behind."""

    def __init__(self, client):
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()
        client._loop = self._loop

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self._loop.call_soon_threadsafe(_cancel_all_and_stop, self._loop)


def _client():
    return CyphalControlClient(port="(fake)", autostart=False)


def _can_client():
    return CyphalControlClient(can_iface="can1", autostart=False)


class _Boom:
    """Every call fails at the socket — the request never reaches the bus."""
    response_timeout = 1.0

    async def call(self, request):
        raise OSError(errno.ENOBUFS, "No buffer space available")


class _Mute:
    """Answers nothing; the caller's timeout expires."""
    response_timeout = 0.05

    async def call(self, request):
        await asyncio.sleep(30)


class _Ok:
    response_timeout = 1.0

    async def call(self, request):
        return (object(), None)


def test_a_send_failure_is_kept_and_named():
    # ENOBUFS: the CAN interface accepted nothing for transmission. Reported
    # as a bare NOT FOUND, it sends an operator to check wiring for what is
    # a host-side bit-timing problem.
    c = _can_client()
    with _Loop(c):
        assert c._call(_Boom(), None) is None
    detail = c.probe_failure_detail()
    assert "No buffer space" in detail
    assert "can1" in detail                 # the interface, not a placeholder
    assert "ERROR-PASSIVE" in detail        # where to confirm it


def test_a_serial_link_gets_no_can_advice():
    # One class serves cyphal-serial and cyphal-can. A serial write buffer
    # filling must not send the operator to inspect a CAN interface.
    c = _client()
    with _Loop(c):
        assert c._call(_Boom(), None) is None
    detail = c.probe_failure_detail()
    assert "No buffer space" in detail
    assert "ERROR-PASSIVE" not in detail
    assert "sjw" not in detail


def test_a_silent_device_adds_nothing():
    # A plain timeout is the ordinary "nothing answered" case; the message
    # must not grow a spurious reason for it.
    c = _client()
    with _Loop(c):
        assert c._call(_Mute(), None, timeout=0.2) is None
    assert c.probe_failure_detail() is None


def test_a_later_success_clears_a_stale_reason():
    c = _client()
    with _Loop(c):
        assert c._call(_Boom(), None) is None
        assert c.probe_failure_detail() is not None
        assert c._call(_Ok(), None) is not None
    assert c.probe_failure_detail() is None


def test_i2c_probe_keeps_the_bus_error():
    class _Bus:
        def read_byte_data(self, addr, reg):
            raise OSError(errno.EACCES, "Permission denied")

    t = NxsI2cTransport(0, _bus_obj=_Bus())
    assert t.probe() is False
    assert "Permission denied" in t.probe_failure_detail()


def test_i2c_probe_of_an_answering_wrong_device_adds_nothing():
    # A read that succeeds and returns the wrong WHO_AM_I is a genuine
    # "not an NXS", not a bus error — no reason to append.
    class _Bus:
        def read_byte_data(self, addr, reg):
            return 0x00

    t = NxsI2cTransport(0, _bus_obj=_Bus())
    assert t.probe() is False
    assert t.probe_failure_detail() is None
