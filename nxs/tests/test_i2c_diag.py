"""The SEL diag view from the host side: select the view, page a counter
index, read the u32 window — and the paging protocol the fake enforces is
the one the firmware serves."""

import pytest

from nxs._generated_constants import NxsRegisters
from nxs.transports.i2c import (
    DRIVER_VIEW_DIAG,
    NxsI2cTransport,
    REG_DRIVER_SELECT, REG_SEL_VALUE, REG_SEL_VALUE_INDEX,
)

_Diag = NxsRegisters.DiagCounter


class _DiagBus:
    """Serves the diag view like the register map does: entering the view
    resets the page to 0, an out-of-range index reads 0."""

    def __init__(self, counters):
        self.counters = counters
        self.view = None
        self.index = 0

    def write_byte_data(self, addr, reg, val):
        if reg == REG_DRIVER_SELECT:
            self.view = val
            self.index = 0
        elif reg == REG_SEL_VALUE_INDEX:
            self.index = val

    def read_i2c_block_data(self, addr, reg, n):
        assert reg == REG_SEL_VALUE and n == 4
        assert self.view == DRIVER_VIEW_DIAG, "read outside the diag view"
        value = self.counters.get(self.index, 0)
        return list(value.to_bytes(4, "little"))

    def read_byte_data(self, addr, reg):
        # The register map echoes a committed selector; _await_selector polls
        # for it before reading the value.
        if reg == REG_DRIVER_SELECT:
            return self.view if self.view is not None else 0
        if reg == REG_SEL_VALUE_INDEX:
            return self.index
        return 0


class _StuckSelectorBus(_DiagBus):
    """The comm thread never commits the selector: DRIVER_SELECT never
    echoes the diag view, so the counter is unreadable."""

    def read_byte_data(self, addr, reg):
        if reg == REG_DRIVER_SELECT:
            return 0            # never DRIVER_VIEW_DIAG
        return super().read_byte_data(addr, reg)


def _transport(bus) -> NxsI2cTransport:
    return NxsI2cTransport(0, _bus_obj=bus)


def test_each_getter_pages_its_own_index():
    bus = _DiagBus({
        int(_Diag.DRDY_COALESCED): 7,
        int(_Diag.INGRESS_REJECTS): 2,
        int(_Diag.I2C_CMD_QUEUE_OVERFLOWS): 1,
        int(_Diag.VM_IO_ERRORS): 30,
        int(_Diag.PROBE_FAILURES): 4,
    })
    t = _transport(bus)
    assert t.read_drdy_coalesced_count() == 7
    assert t.read_ingress_reject_count() == 2
    assert t.read_cmd_queue_overflow_count() == 1
    assert t.read_io_err_count() == 30
    assert t.read_probe_failed_count() == 4


def test_saturated_counter_reads_sticky_max():
    bus = _DiagBus({int(_Diag.DRDY_COALESCED): 0xFFFF})
    assert _transport(bus).read_drdy_coalesced_count() == 0xFFFF


def test_unfed_counter_reads_zero():
    bus = _DiagBus({})
    assert _transport(bus).read_ingress_reject_count() == 0


def test_uncommitted_selector_raises_instead_of_serving_garbage():
    import pytest
    # A losing/wedged comm thread never commits the diag view; the read must
    # fail loudly (nxs status omits the line) rather than return another
    # view's bytes as a counter.
    with pytest.raises(TimeoutError, match="did not commit"):
        _transport(_StuckSelectorBus({})).read_drdy_coalesced_count()
