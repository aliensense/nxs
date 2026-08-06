"""Transport implementations and the open_client() registry.

Each transport the registry can construct is a peer module here (i2c,
cyphal_source / cyphal_control, mock). open_client() maps a transport name to its
class, importing the backend lazily so optional deps (smbus2,
pycyphal) stay optional until the transport is actually used.
"""
from nxs.client import NxsClient


def open_client(transport: str, **kwargs) -> NxsClient:
    """Construct the transport-specific NxsClient.

    `transport` is "i2c", "cyphal-serial", "cyphal-can", or "mock";
    kwargs are forwarded to the transport constructor (e.g. bus/address
    for I2C, port/baud for Cyphal/serial). Backends import lazily so
    this package stays free of their dependencies until used.
    """
    kind = transport.lower()
    if kind in ("i2c", "smbus"):
        from nxs.transports.i2c import NxsI2cTransport
        return NxsI2cTransport(**kwargs)
    if kind in ("cyphal-serial", "cyphal-can", "cyphal"):
        from nxs.transports.cyphal_control import CyphalControlClient
        return CyphalControlClient(**kwargs)
    if kind == "mock":
        from nxs.transports.mock import MockTransport
        return MockTransport(**kwargs)
    raise ValueError(f"unknown transport: {transport!r}")
