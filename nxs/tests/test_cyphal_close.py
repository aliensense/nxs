"""close() must not close the asyncio loop while its thread survived the
join — closing a loop under a live thread races with it."""
import warnings

from nxs.transports.cyphal_control import CyphalControlClient


class _StuckThread:
    def join(self, timeout=None):
        pass

    def is_alive(self):
        return True


class _RecordingLoop:
    def __init__(self):
        self.closed = False

    def call_soon_threadsafe(self, cb, *args):
        pass

    def stop(self):
        pass

    def close(self):
        self.closed = True


def test_close_skips_loop_close_when_thread_survives(tmp_path):
    client = CyphalControlClient.__new__(CyphalControlClient)
    client._node = None
    loop = _RecordingLoop()
    client._loop = loop
    client._thread = _StuckThread()
    client._serve_dir = str(tmp_path / 'serve')
    with warnings.catch_warnings():
        # run_coroutine_threadsafe rejects the fake loop before consuming
        # the shutdown coroutine; ignore the never-awaited warning.
        warnings.simplefilter('ignore', RuntimeWarning)
        client.close()
    assert not loop.closed
