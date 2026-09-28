"""nxs: sensor driver compiler, client SDK, and suite tooling for NXS. `Sample`
here is the driver DSL's measure-loop return type; the decoded stream sample
`open_client(...).iter_samples()` yields is `nxs.client.Sample`."""

from importlib.metadata import PackageNotFoundError, version as _installed_version

from nxs.client import NxsClient
from nxs.compiler import (
    CameraSensor,
    CompiledDriver,
    I2cCommandDriver,
    RegisterDriver,
    Sample,
    SensorDriver,
    StreamDriver,
)
from nxs.profiles import I2cProfile, SpiProfile
from nxs.transports import open_client



def _version() -> str:
    """The installed wheel's version, `0.0.0+source` in a checkout."""
    try:
        return _installed_version("aliensense-nxs")
    except PackageNotFoundError:
        return "0.0.0+source"


__version__ = _version()

__all__ = [
    "CameraSensor",
    "CompiledDriver",
    "I2cCommandDriver",
    "I2cProfile",
    "NxsClient",
    "RegisterDriver",
    "Sample",
    "SensorDriver",
    "SpiProfile",
    "StreamDriver",
    "open_client",
]
