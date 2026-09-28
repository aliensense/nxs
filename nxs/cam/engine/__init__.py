# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Sequence engine: parse a plan config, execute it over I2C.
``Manager`` runs a plan live with per-op retries; ``SequenceExecutor``
dispatches one to any sink (``SpyRunner`` for dry-runs and byte-diff tests)."""

from .config import ConfigHandler
from .i2c import CamI2c, I2CHandlerError
from .manager import Manager
from .models import (
    CmdStep,
    DeviceStep,
    ExpectFailedError,
    ExpectStep,
    ParsedConfig,
    ReadStep,
    RetryBlockStep,
    Step,
)
from .executor import SequenceExecutor
from .testing import SpyRunner

__all__ = [
    "CamI2c",
    "CmdStep",
    "ConfigHandler",
    "DeviceStep",
    "ExpectFailedError",
    "ExpectStep",
    "I2CHandlerError",
    "Manager",
    "ParsedConfig",
    "ReadStep",
    "RetryBlockStep",
    "SequenceExecutor",
    "SpyRunner",
    "Step",
]
