"""The client registry: `open_client()` hands out the library's unit client
for every wire."""
from nxs.client import NxsClient


def open_client(transport: str, **kwargs) -> NxsClient:
    """Construct the client of one unit. `transport` is "i2c", "cyphal-serial"
    or "cyphal-can"; kwargs go to the opener (bus/address for I2C,
    port/baud for serial, can_iface/can_mtu for CAN, and the node-IDs)."""
    kind = transport.lower()
    if kind in ("i2c", "smbus"):
        from nxs._libnxs_unit import I2cUnit
        return I2cUnit.open_i2c(**kwargs)
    if kind in ("cyphal-serial", "cyphal"):
        from nxs._libnxs_unit import CyphalUnit
        return CyphalUnit.open_serial(**kwargs)
    if kind == "cyphal-can":
        from nxs._libnxs_unit import CyphalUnit
        return CyphalUnit.open_can(**kwargs)
    raise ValueError(f"unknown transport: {transport!r}")
