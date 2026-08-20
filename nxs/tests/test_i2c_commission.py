"""i2c read_identity resolves UNSET record fields to effective compiled defaults."""
import struct

import pytest

from nxs.client import COMMISSION_TOPICS, validate_commission
from nxs._generated_constants import CyphalDefaults
from nxs.transports.i2c import NxsI2cTransport, _TOPIC_DEFAULTS


class _FakeConfigBus:
    """Serves a fixed 20-byte CONFIG record for read_identity. Models the
    mode register (`XFER_TYPE` echoes writes) so the claim-check readback
    the config paths perform sees its echo; `held_mode` pins it like a
    live transfer session would."""

    def __init__(self, record, held_mode=None):
        self._record = bytes(record)
        self.held_mode = held_mode
        self.xfer_type = held_mode if held_mode is not None else 0

    def write_byte_data(self, addr, reg, val):
        from nxs.transports.i2c import REG_XFER_TYPE
        if reg == REG_XFER_TYPE and self.held_mode is None:
            self.xfer_type = val

    def read_byte_data(self, addr, reg):
        from nxs.transports.i2c import REG_XFER_TYPE
        if reg == REG_XFER_TYPE:
            return self.xfer_type
        return 0

    def read_i2c_block_data(self, addr, reg, length):
        return list(self._record[:length])


class _RecordingBus(_FakeConfigBus):
    """Records byte-register writes so tests can assert the mode sequence."""

    def __init__(self, record):
        super().__init__(record)
        self.byte_writes = []

    def write_byte_data(self, addr, reg, val):
        self.byte_writes.append((reg, val))
        super().write_byte_data(addr, reg, val)

    def write_i2c_block_data(self, addr, reg, data):
        pass


def _transport(record):
    return NxsI2cTransport(0, _bus_obj=_FakeConfigBus(record))


def test_read_identity_resolves_unset_to_defaults():
    ident = _transport(b"\xff\xff" * 10).read_identity()   # factory-fresh: all UNSET
    assert ident["node_addr"] == CyphalDefaults.DEFAULT_NODE_ID
    for name in COMMISSION_TOPICS:
        assert ident["topics"][name] == _TOPIC_DEFAULTS[name]


def test_read_identity_reports_commissioned_values():
    record = bytearray(b"\xff\xff" * 10)
    struct.pack_into("<H", record, 0, 10)     # node = 10
    struct.pack_into("<H", record, 2, 6200)   # sample subject = 6200
    ident = _transport(record).read_identity()
    assert ident["node_addr"] == 10
    assert ident["topics"]["sample"] == 6200
    assert ident["topics"]["status"] == _TOPIC_DEFAULTS["status"]   # unset → default


def test_commission_against_a_held_mux_streams_nothing():
    # A live transfer session refuses the CONFIG mode write silently; the
    # claim readback detects it before a record byte can land in the
    # holder's sink (the cross-flow corruption the session lock exists
    # to stop).
    from nxs.client import DeviceRefused
    from nxs.transports.i2c import (REG_PROGRAM_DATA, REG_XFER_TYPE,
                                    XFER_TYPE_DFU_IMAGE)
    bus = _RecordingBus(b"\xff" * 20)
    bus.held_mode = XFER_TYPE_DFU_IMAGE
    bus.xfer_type = XFER_TYPE_DFU_IMAGE
    t = NxsI2cTransport(0, _bus_obj=bus)
    with pytest.raises(DeviceRefused) as ei:
        t.commission(10, {})
    assert ei.value.code == 16
    assert not any(reg == REG_PROGRAM_DATA for reg, _ in bus.byte_writes)


def test_validate_commission_rejects_out_of_range():
    with pytest.raises(ValueError):
        validate_commission(126, None)               # reserved for diagnostic tools
    with pytest.raises(ValueError):
        validate_commission(127, None)               # the host tooling's own ID
    with pytest.raises(ValueError):
        validate_commission(200, None)               # node > 125, not a sentinel
    with pytest.raises(ValueError):
        validate_commission(-1, None)                # negative never wraps
    with pytest.raises(ValueError):
        validate_commission(None, {"sample": 9000})  # subject > 8191
    with pytest.raises(ValueError):
        validate_commission(None, {"scalar": 8182})  # base + span past the ceiling
    with pytest.raises(ValueError):
        validate_commission(None, {"bogus": 100})    # unknown topic


def test_validate_commission_accepts_valid():
    validate_commission(125, {"acceleration": 6246})
    validate_commission(255, None)                   # anonymous sentinel
    validate_commission(None, {"sample": 0})         # 0 disables the topic
    validate_commission(0xFFFF, None)                # revert node to the default
    validate_commission(None, {"sample": 0xFFFF})    # revert a subject to the default
    validate_commission(None, {"scalar": 8181})      # span exactly fits the ceiling


def test_commission_restores_xfer_type():
    from nxs.transports.i2c import (REG_XFER_TYPE, XFER_TYPE_CONFIG,
                                    XFER_TYPE_VM_BYTECODE)
    bus = _RecordingBus(b"\xff\xff" * 10)
    NxsI2cTransport(0, _bus_obj=bus).commission(node_addr=10)
    xfer_writes = [v for r, v in bus.byte_writes if r == REG_XFER_TYPE]
    assert XFER_TYPE_CONFIG in xfer_writes            # entered config mode
    assert xfer_writes[-1] == XFER_TYPE_VM_BYTECODE   # and left it — uploads rely on 0


def test_read_identity_restores_xfer_type():
    from nxs.transports.i2c import (REG_XFER_TYPE, XFER_TYPE_CONFIG,
                                    XFER_TYPE_VM_BYTECODE)
    bus = _RecordingBus(b"\xff\xff" * 10)
    NxsI2cTransport(0, _bus_obj=bus).read_identity()
    xfer_writes = [v for r, v in bus.byte_writes if r == REG_XFER_TYPE]
    assert xfer_writes == [XFER_TYPE_CONFIG, XFER_TYPE_VM_BYTECODE]
