"""
nxs — Declarative sensor driver compiler, client SDK, and suite tooling for NXS.

Two surfaces, one package. Authoring: write sensor drivers as Python
classes, compile to VM bytecode. Integration: drive a device over any
transport and consume decoded samples.

    # Authoring
    from nxs import RegisterDriver, Sample, SpiProfile

    # Integration
    from nxs import open_client
    client = open_client("i2c", bus="/dev/i2c-2", address=0x30)
    for sample in client.iter_samples():
        handle(sample.values)

`Sample` at this level is the driver DSL's measure-loop return type;
the decoded stream sample `iter_samples()` yields lives at
`nxs.client.Sample` (received, not constructed — rarely imported).
"""

from nxs.client import NxsClient
from nxs.compiler import (
    CompiledDriver,
    I2cCommandDriver,
    RegisterDriver,
    Sample,
    SensorDriver,
    StreamDriver,
)
from nxs.profiles import I2cProfile, SpiProfile, UartProfile
from nxs.transports import open_client

__all__ = [
    "CompiledDriver",
    "I2cCommandDriver",
    "I2cProfile",
    "NxsClient",
    "RegisterDriver",
    "Sample",
    "SensorDriver",
    "SpiProfile",
    "StreamDriver",
    "UartProfile",
    "open_client",
]
