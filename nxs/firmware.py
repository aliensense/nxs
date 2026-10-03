"""What firmware a device runs: the register-map contract it speaks, the version floor this tool drives, and the verdict on a push."""

import os
import re
import time
from typing import Optional


# Highest register-map contract version this SDK understands; bump it only
# when this code speaks a new contract version.
SUPPORTED_PROTO_VERSION = 1

def contract_mismatch(transport, *, unreadable_is_skew: bool = False) -> Optional[str]:
    """None when the device speaks this SDK's register-map contract or serves
    no version, else a one-line description of the skew. Pass
    `unreadable_is_skew=True` before mutating."""
    reader = getattr(transport, "interface_version", None)
    if reader is None:
        return None                 # serves no version, like Cyphal
    try:
        ver = reader()
    except NotImplementedError:
        return None
    except (OSError, RuntimeError) as e:
        if unreadable_is_skew:
            return f"could not read the device's register-map version ({e})"
        return None
    if ver is None or ver == SUPPORTED_PROTO_VERSION:
        return None
    return (f"device speaks register-map v{ver}; this nxs speaks "
            f"v{SUPPORTED_PROTO_VERSION} only")

# The oldest firmware this wheel drives: the image format the compiler emits
# and the camera-run surface both arrived with it. Older units are refused
# with the push that ends the mismatch; the asset is the release's
# production-signed firmware.
MIN_FIRMWARE = "1.1.0"

MIN_FIRMWARE_ASSET = f"nxs-v{MIN_FIRMWARE}-firmware.bin"

_ASSET_NAME = re.compile(r"^nxs-v(\d+)\.(\d+)\.(\d+)(?:-rc(\d+))?-firmware\.bin$")


def push_fw_asset(store: Optional[str] = None) -> str:
    """The firmware file the refusal tells the operator to push: the newest
    release image in the assets store (`nxs assets install` fills it), else
    the path the current release's image takes there."""
    from nxs.suite import FIRMWARE_DIR
    store = store or FIRMWARE_DIR
    found = []
    try:
        names = os.listdir(store)
    except OSError:
        names = []
    for name in names:
        m = _ASSET_NAME.match(name)
        if m:
            major, minor, patch, rc = m.groups()
            key = (int(major), int(minor), int(patch), 1 if rc is None else 0,
                   int(rc or 0))
            found.append((key, name))
    if found:
        return os.path.join(store, max(found)[1])
    return os.path.join(store, MIN_FIRMWARE_ASSET)

class FirmwareTooOld(RuntimeError):
    """The unit runs firmware older than `MIN_FIRMWARE`; the message is the
    refusal line, ending in the `push-fw` command."""

def parse_fw_identity(identity) -> Optional[tuple]:
    """The `(major, minor, patch)` a firmware identity proves: a `git
    describe` build identity (`v1.0.1-4-g87fdf5b` → (1, 0, 1)) or the
    legacy `MAJOR.MINOR` pair. None for an identity that proves nothing (a
    prerelease tag, a bare SHA, no identity)."""
    from nxs.suite.schema import parse_device_version
    return parse_device_version(identity) if identity else None

def read_firmware_version(transport) -> Optional[tuple]:
    """The `(major, minor, 0)` a unit states as its own firmware version,
    or None when it serves none. This is the version the application was
    built as, which a release tag matches; the build identity beside it
    names the commit, and on a build between tags describes off the
    earlier one."""
    try:
        pair = transport.read_fw_version_pair()
    except (AttributeError, Exception):
        return None
    if not pair or not pair[0]:
        return None
    return (int(pair[0]), int(pair[1]), 0)

def firmware_too_old(transport, target: str = "<target>") -> Optional[str]:
    """None when the unit runs `MIN_FIRMWARE` or newer, or proves no
    version to judge by, else the refusal line. `target` is the unit as
    the operator addresses it on the command line. The verdict takes the
    higher of what the unit states and what its identity proves: a
    development build carries the version it was built as and an identity
    described from the previous tag, and refusing it would strand the
    bench deployment path between releases."""
    identity = read_identity(transport)
    proven = [v for v in (read_firmware_version(transport),
                          parse_fw_identity(identity)) if v is not None]
    if not proven:
        return None
    ver = max(proven)
    from nxs.suite.schema import parse_version
    if ver >= parse_version(MIN_FIRMWARE):
        return None
    return (f"unit runs firmware {identity}; this nxs needs v{MIN_FIRMWARE} "
            f"or newer — push it: nxs {target} push-fw {push_fw_asset()}")

def require_firmware(transport, target: str = "<target>") -> None:
    """Refuse a unit older than `MIN_FIRMWARE`: raises `FirmwareTooOld`
    carrying the refusal line. A unit serving no readable identity passes,
    like the contract gate; the command then fails on its own terms."""
    line = firmware_too_old(transport, target)
    if line is not None:
        raise FirmwareTooOld(line)

# One wait for the device to come back after a firmware swap: the reboot,
# the MCUboot swap (several seconds), and the new image's boot.
DFU_REBOOT_TIMEOUT_S = 30.0

def await_reachable(transport, timeout_s: float, poll_s: float = 0.5) -> bool:
    """Poll `probe()` until the device answers or `timeout_s` elapses. A dead
    bus raises rather than answering False."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if transport.probe():
                return True
        except Exception:
            pass
        time.sleep(poll_s)
    return False

def read_identity(transport):
    """The device's build identity (`git describe`, or "MAJOR.MINOR" on
    older firmware); None when the transport does not serve one."""
    try:
        return transport.read_fw_version()
    except Exception:
        return None

def update_verdict(before, after, pushed_version, confirmed=None, pushed_identity=None):
    """What a push achieved, from the identities read either side of it, as
    `(ok, line)`. When `pushed_identity` is known the device serving it
    afterwards is the proof; `confirmed` False proves it is running."""
    from nxs.suite.schema import device_runs, parse_version
    if after is None:
        return False, "✗ the device answers but reports no firmware version"
    if pushed_identity is not None:
        # Identity is authoritative: the device must serve the pushed image.
        # `confirmed` only annotates a verdict that already matches.
        if after != pushed_identity:
            return False, (f"✗ rejected or reverted: the device still reports {after}; "
                           f"the pushed image is {pushed_identity} — check the device log")
        test = " (running in TEST until confirmed)" if confirmed is False else ""
        if before != after:
            return True, f"✓ updated: {before or 'unknown'} → {after}{test}"
        return True, f"✓ unchanged: the device already ran {after}{test}"
    if confirmed is False:
        return True, f"✓ updated: {before or 'unknown'} → {after} (running in TEST until confirmed)"
    if before != after:
        return True, f"✓ updated: {before or 'unknown'} → {after}"
    if device_runs(after, parse_version(pushed_version.split("+", 1)[0])):
        return True, f"✓ unchanged: the device already ran {after}"
    return False, (f"✗ rejected or reverted: the device still reports {after} "
                   f"after the push of {pushed_version} — check the device log")

def push_and_verify(transport, bin_path, pushed_version, progress_cb=None, before=None):
    """Push an image, wait for the reboot, and judge what it achieved as
    `(ok, line, after)`. `before` is the identity already read for the
    downgrade check."""
    from nxs import mcuboot_image
    with open(bin_path, "rb") as fh:
        pushed_identity = mcuboot_image.identity(fh.read())
    if before is None:
        before = read_identity(transport)
    # The client's push returns once the unit is back from its reboot; the
    # wait here only covers a unit that took longer to answer than that.
    transport.push_image(bin_path, progress_cb=progress_cb)
    if not await_reachable(transport, DFU_REBOOT_TIMEOUT_S):
        return False, (f"✗ pushed {pushed_version}, but the device did not answer "
                       f"within {DFU_REBOOT_TIMEOUT_S:.0f} s — check the device log"), None
    after = read_identity(transport)
    ok, line = update_verdict(before, after, pushed_version, transport.read_fw_confirmed(),
                              pushed_identity)
    return ok, line, after
