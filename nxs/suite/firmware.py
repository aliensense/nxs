"""Firmware image store: resolve a manifest version pin to a signed image by
its MCUboot header version, never by filename."""
import glob
import os

from nxs.mcuboot_image import HEADER, IMAGE_MAGIC
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


def find_image(store_dir: str, pin: str) -> str:
    """Path of the image in `store_dir` whose header version equals `pin`.
    Raises FileNotFoundError, naming the store contents, when none matches."""
    want = parse_version(pin)
    seen = []
    for path in sorted(glob.glob(os.path.join(store_dir, "*.bin"))):
        try:
            version = read_image_version(path)
        except (OSError, ValueError):
            continue
        if version == want:
            return path
        seen.append(f"{os.path.basename(path)} ({'.'.join(map(str, version))})")
    detail = ", ".join(seen) if seen else "no MCUboot images"
    raise FileNotFoundError(
        f"no image for firmware {pin} in {store_dir} (found: {detail})")
