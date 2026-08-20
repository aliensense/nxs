"""NxsI2cTransport identity reads: the 12-byte chip UID96 from the
SERIAL window and the firmware version from the FW_VERSION registers;
transports without them return None."""
from nxs.transports.i2c import (
    NxsI2cTransport, REG_SERIAL, SERIAL_LEN,
    REG_FW_VERSION_MAJOR, REG_FW_VERSION_MINOR,
    REG_XFER_TYPE, REG_STORE_SELECT, REG_PROGRAM_DATA,
    XFER_TYPE_BUILD_INFO, BUILD_INFO_SIZE)
from nxs.transports.mock import MockTransport


class _FakeSerialBus:
    """Serves the identity surface of the register map: SERIAL, the
    FW_VERSION pair, and the BUILD_INFO transfer (the PROGRAM_DATA
    window pages the describe record only while XFER_TYPE selects it,
    like the firmware latch)."""

    def __init__(self, uid: bytes = b"", fw=(0, 0), describe: bytes = b""):
        self._regs = bytearray(256)
        self._regs[REG_SERIAL:REG_SERIAL + len(uid)] = uid
        self._regs[REG_FW_VERSION_MAJOR] = fw[0]
        self._regs[REG_FW_VERSION_MINOR] = fw[1]
        self._build_info = describe[:BUILD_INFO_SIZE].ljust(BUILD_INFO_SIZE, b"\x00")
        self.writes = []

    def read_i2c_block_data(self, addr, reg, n):
        if reg == REG_PROGRAM_DATA and self._regs[REG_XFER_TYPE] == XFER_TYPE_BUILD_INFO:
            page = self._regs[REG_STORE_SELECT]
            return list(self._build_info[page * n:(page + 1) * n].ljust(n, b"\x00"))
        return list(self._regs[reg:reg + n])

    def read_byte_data(self, addr, reg):
        return self._regs[reg]

    def write_byte_data(self, addr, reg, val):
        self.writes.append((reg, val))
        self._regs[reg] = val


def test_read_serial_returns_uid96():
    uid = bytes(range(0xA0, 0xA0 + SERIAL_LEN))
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(uid))
    assert t.read_serial() == uid
    assert len(t.read_serial()) == SERIAL_LEN


def test_mock_serves_its_synthetic_identity():
    t = MockTransport()
    assert t.read_serial() == MockTransport.SERIAL
    assert len(t.read_serial()) == SERIAL_LEN
    assert t.read_fw_version() == MockTransport.FW_VERSION


def test_read_fw_version_waits_for_the_mode_echo():
    """XFER_TYPE commits deferred: paging before the echo consumes the
    previous mode's window and silently degrades to the legacy pair."""
    from nxs.transports.i2c import (REG_PROGRAM_DATA, REG_XFER_TYPE,
                                    XFER_TYPE_BUILD_INFO)

    class _DeferredModeBus(_FakeSerialBus):
        """Echoes a mode write only after two readbacks, like the real
        comm-thread drain; serves the identity only once in mode 5."""

        def __init__(self):
            super().__init__(fw=(1, 4))
            self.pending = None
            self.lag = 0

        def write_byte_data(self, addr, reg, val):
            if reg == REG_XFER_TYPE:
                self.pending = val
                self.lag = 2
                return
            super().write_byte_data(addr, reg, val)

        def read_byte_data(self, addr, reg):
            if reg == REG_XFER_TYPE:
                if self.lag > 0:
                    self.lag -= 1
                    return self._regs[REG_XFER_TYPE]
                if self.pending is not None:
                    self._regs[REG_XFER_TYPE] = self.pending
                    self.pending = None
                return self._regs[REG_XFER_TYPE]
            return super().read_byte_data(addr, reg)

        def read_i2c_block_data(self, addr, reg, n):
            if reg == REG_PROGRAM_DATA:
                if self._regs[REG_XFER_TYPE] == XFER_TYPE_BUILD_INFO:
                    return list(b"v1.2.3-4-g87fdf5b".ljust(n, b"\x00"))[:n]
                return [0xFF] * n
            return super().read_i2c_block_data(addr, reg, n)

    t = NxsI2cTransport(_bus_obj=_DeferredModeBus())
    assert t.read_fw_version() == "v1.2.3-4-g87fdf5b"


def test_read_fw_version_session_held_falls_back():
    """A live transfer session owns the window: the reader must not page
    it, and the legacy pair is the honest degraded answer."""
    from nxs.transports.i2c import (REG_PROGRAM_DATA, REG_XFER_TYPE,
                                    XFER_TYPE_DFU_IMAGE)

    class _HeldBus(_FakeSerialBus):
        def __init__(self):
            super().__init__(fw=(1, 4))
            self._regs[REG_XFER_TYPE] = XFER_TYPE_DFU_IMAGE
            self.window_reads = 0

        def write_byte_data(self, addr, reg, val):
            if reg == REG_XFER_TYPE:
                return          # silent refusal: the session holds the mux
            super().write_byte_data(addr, reg, val)

        def read_i2c_block_data(self, addr, reg, n):
            if reg == REG_PROGRAM_DATA:
                self.window_reads += 1
            return super().read_i2c_block_data(addr, reg, n)

    bus = _HeldBus()
    t = NxsI2cTransport(_bus_obj=bus)
    assert t.read_fw_version() == "1.4"
    assert bus.window_reads == 0


def test_read_fw_version_serves_major_minor():
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(fw=(1, 4)))
    assert t.read_fw_version() == "1.4"


def test_read_fw_version_unseeded_is_none():
    """Major 0 is the sentinel for firmware predating the registers."""
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(fw=(0, 7)))
    assert t.read_fw_version() is None


def test_read_fw_version_serves_the_describe_string():
    bus = _FakeSerialBus(fw=(1, 0), describe=b"v1.0.0-4-g87fdf5b")
    t = NxsI2cTransport(_bus_obj=bus)
    assert t.read_fw_version() == "v1.0.0-4-g87fdf5b"


def test_read_fw_version_spans_the_page_boundary():
    long = b"v1.0.0-1234-g87fdf5bdeadbeefcafe-dirty"
    assert len(long) > 32
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(describe=long))
    assert t.read_fw_version() == long.decode()


def test_read_fw_version_restores_xfer_state_and_never_writes_zero():
    bus = _FakeSerialBus(describe=b"v1.0.0-4-g87fdf5b")
    bus._regs[REG_XFER_TYPE] = 4      # a calibration read-back in progress
    bus._regs[REG_STORE_SELECT] = 3
    t = NxsI2cTransport(_bus_obj=bus)
    assert t.read_fw_version() == "v1.0.0-4-g87fdf5b"
    assert bus._regs[REG_XFER_TYPE] == 4
    assert bus._regs[REG_STORE_SELECT] == 3
    # An XFER_TYPE = 0 write clears CMD_ERROR on the device; a version
    # read must never issue one.
    assert (REG_XFER_TYPE, 0) not in bus.writes


def test_read_fw_version_falls_back_when_the_window_is_dark():
    t = NxsI2cTransport(_bus_obj=_FakeSerialBus(fw=(1, 4)))
    assert t.read_fw_version() == "1.4"
