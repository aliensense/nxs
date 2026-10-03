"""Firmware image store: resolve a manifest version pin to a signed image by
its MCUboot header version, never by filename."""
import glob
import os
from typing import Optional

from nxs.mcuboot_image import HEADER, IMAGE_MAGIC, identity
from nxs.suite.schema import parse_version


def read_image_version(path: str) -> tuple:
    """(major, minor, revision) from the fixed MCUboot header of a signed
    image. Raises ValueError when the file is not an MCUboot image."""
    with open(path, "rb") as f:
        header = f.read(HEADER.size)
    if len(header) < HEADER.size:
        raise ValueError(f"{path}: too short for an MCUboot header")
    magic, _, _, _, _, _, major, minor, revision, _ = HEADER.unpack_from(header)
    if magic != IMAGE_MAGIC:
        raise ValueError(f"{path}: MCUboot magic mismatch "
                         f"(0x{magic:08X} != 0x{IMAGE_MAGIC:08X})")
    return major, minor, revision


def image_identity(path: str) -> Optional[str]:
    """The build identity a signed image carries (`v1.1.0-rc1`), None for
    an image that carries none."""
    with open(path, "rb") as f:
        return identity(f.read())


def _same_build(carried: Optional[str], wanted: Optional[str]) -> bool:
    """Whether two build identities name one build: `-dirty` marks the tree
    a build came from, not the build, so it tells no two apart."""
    strip = lambda name: name[:-len("-dirty")] if name and name.endswith("-dirty") else name
    return strip(carried) == strip(wanted)


def find_image(store_dir: str, pin: str, build: Optional[str] = None) -> str:
    """Path of the image in `store_dir` whose header version equals `pin`.
    With `build`, the image carrying that build identity, else one carrying
    none: a release candidate's header carries the bare version, so only
    the identity tells two candidates apart. Raises FileNotFoundError,
    naming the store contents, when none matches."""
    want = parse_version(pin)
    seen, matches = [], []
    for path in sorted(glob.glob(os.path.join(store_dir, "*.bin"))):
        try:
            version = read_image_version(path)
        except (OSError, ValueError):
            continue
        if version == want:
            matches.append(path)
        else:
            seen.append(f"{os.path.basename(path)} ({'.'.join(map(str, version))})")
    if matches and build is None:
        return matches[0]
    built = [(path, image_identity(path)) for path in matches]
    for wanted in (build, None):
        for path, carried in built:
            if _same_build(carried, wanted):
                return path
    if built:
        found = ", ".join(f"{os.path.basename(path)} ({carried})" for path, carried in built)
        raise FileNotFoundError(
            f"no image of {build} for firmware {pin} in {store_dir} (found: {found}) "
            f"— install this release's assets: nxs assets install")
    detail = ", ".join(seen) if seen else "no MCUboot images"
    raise FileNotFoundError(
        f"no image for firmware {pin} in {store_dir} (found: {detail})")
