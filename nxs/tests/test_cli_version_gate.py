"""The tool refuses a device speaking a different register-map contract
loudly before any session-stateful traffic, and lets `probe` through so the
mismatch can be diagnosed."""
import types

import pytest

from nxs.cli import _require_supported_version
from nxs.client import SUPPORTED_PROTO_VERSION


def _fake(version):
    return types.SimpleNamespace(interface_version=lambda: version)


def test_matching_version_passes():
    _require_supported_version(_fake(SUPPORTED_PROTO_VERSION))   # no raise


def test_mismatched_contract_exits_loudly():
    other = SUPPORTED_PROTO_VERSION + 1
    with pytest.raises(SystemExit) as ei:
        _require_supported_version(_fake(other))
    assert f"register-map v{other}" in str(ei.value)


def test_unversioned_firmware_exits():
    with pytest.raises(SystemExit):
        _require_supported_version(_fake(0))


def test_cyphal_none_is_skipped():
    _require_supported_version(_fake(None))     # no register, no gate


def test_unreadable_version_is_left_to_the_command():
    def raiser():
        raise OSError("bus")
    _require_supported_version(types.SimpleNamespace(interface_version=raiser))
