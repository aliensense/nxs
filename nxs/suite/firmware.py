"""Firmware image store: resolve a manifest version pin to a signed
image by reading each candidate's MCUboot header — the pin matches
image content, never a filename convention.
"""
import glob
import os
import struct

from nxs.suite.schema import parse_version

IMAGE_MAGIC = 0x96F3B83D
# image_header: magic u32, load_addr u32, hdr_size u16, protect_tlv_size u16,
# img_size u32, flags u32, then image_version {u8 major, u8 minor, u16 revision,
# u32 build_num} at byte 20 — all little-endian.
_VERSION_OFFSET = 20
_HEADER_MIN = 28


def read_image_version(path: str) -> tuple:
    """(major, minor, revision) from a signed image's MCUboot header.

    Raises ValueError when the file is not an MCUboot image.
    """
    with open(path, "rb") as f:
        header = f.read(_HEADER_MIN)
    if len(header) < _HEADER_MIN:
        raise ValueError(f"{path}: too short for an MCUboot header")
    magic = struct.unpack_from("<I", header, 0)[0]
    if magic != IMAGE_MAGIC:
        raise ValueError(f"{path}: MCUboot magic mismatch "
                         f"(0x{magic:08X} != 0x{IMAGE_MAGIC:08X})")
    major, minor, revision = struct.unpack_from("<BBH", header, _VERSION_OFFSET)
    return major, minor, revision


def find_image(store_dir: str, pin: str) -> str:
    """Path of the image in `store_dir` whose header version equals `pin`.

    Raises FileNotFoundError naming the store contents when no image
    matches, so the operator sees what is actually available.
    """
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
