"""nxs: personality compiler, client SDK, and suite tooling for NXS. `Sample`
here is the personality DSL's measure-loop return type; the decoded stream sample
`open_client(...).iter_samples()` yields is `nxs.client.Sample`."""

from importlib.metadata import PackageNotFoundError, version as _installed_version

from nxs.client import NxsClient
from nxs.compiler import (
    CamPersonality,
    CompiledDriver,
    I2cCommandClickPersonality,
    RegisterClickPersonality,
    Sample,
    ClickPersonality,
    StreamClickPersonality,
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
    "CamPersonality",
    "CompiledDriver",
    "I2cCommandClickPersonality",
    "I2cProfile",
    "NxsClient",
    "RegisterClickPersonality",
    "Sample",
    "ClickPersonality",
    "SpiProfile",
    "StreamClickPersonality",
    "open_client",
]
