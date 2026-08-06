"""i2c read_identity resolves UNSET record fields to effective compiled defaults."""
import struct

import pytest

from nxs.client import COMMISSION_TOPICS, validate_commission
from nxs._generated_constants import CyphalDefaults
from nxs.transports.i2c import NxsI2cTransport, _TOPIC_DEFAULTS


class _FakeConfigBus:
    """Serves a fixed 20-byte CONFIG record for read_identity."""

    def __init__(self, record):
        self._record = bytes(record)

    def write_byte_data(self, addr, reg, val):
        pass

    def read_byte_data(self, addr, reg):
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
