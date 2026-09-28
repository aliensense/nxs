"""MCUboot image container parsing for `push-fw`. Layout per bootutil/image.h:
a 32-byte header, the image, the protected TLVs, then a TLV info word opening
the signature TLVs."""
import re
import struct
from dataclasses import dataclass

from ._generated_constants import NxsMcuboot

IMAGE_MAGIC = NxsMcuboot.IMAGE_MAGIC
IMAGE_HEADER_SIZE = NxsMcuboot.IMAGE_HEADER_SIZE
TLV_INFO_MAGIC = NxsMcuboot.TLV_INFO_MAGIC
HEADER = struct.Struct("<IIHHIIBBHI")
TLV_INFO = struct.Struct("<HH")
TLV_ENTRY = struct.Struct("<HH")
TLV_SHA256 = 0x10


@dataclass(frozen=True)
class ImageInfo:
    version: str      # "1.0.0", "+<build>" appended when the build number is set
    total_size: int   # header + image + TLVs: the bytes the device stages


def parse(data: bytes) -> ImageInfo:
    """Describe an MCUboot image container; ValueError names what is wrong.
    Checks magic and lengths only, the device verifies the signature."""
    if len(data) < HEADER.size:
        raise ValueError("shorter than an MCUboot image header")
    magic, _, hdr_size, prot_size, img_size, _, major, minor, rev, build = \
        HEADER.unpack_from(data)
    if magic != IMAGE_MAGIC:
        raise ValueError("not an MCUboot image (no header magic) — push "
                         "zephyr.signed.bin, not zephyr.bin")
    if hdr_size < IMAGE_HEADER_SIZE:
        raise ValueError(f"header size {hdr_size} is below the "
                         f"{IMAGE_HEADER_SIZE}-byte MCUboot header")
    tlv_off = hdr_size + img_size + prot_size
    if tlv_off + TLV_INFO.size > len(data):
        raise ValueError(f"truncated: the header declares {tlv_off} bytes "
                         f"before the TLVs, the file has {len(data)}")
    tlv_magic, tlv_total = TLV_INFO.unpack_from(data, tlv_off)
    if tlv_magic != TLV_INFO_MAGIC:
        raise ValueError("no TLV trailer after the image — unsigned or corrupt")
    if tlv_total < TLV_INFO.size:
        raise ValueError(f"TLV info size {tlv_total} is below the "
                         f"{TLV_INFO.size}-byte info record")
    total = tlv_off + tlv_total
    if total > len(data):
        raise ValueError(f"truncated: the TLVs declare {total} bytes, "
                         f"the file has {len(data)}")
    version = f"{major}.{minor}.{rev}" + (f"+{build}" if build else "")
    return ImageInfo(version=version, total_size=total)


# The firmware banner line, "<app> <git describe> (<toolchain>)", after a
# newline and six spaces. The describe string is the build identity.
_IDENTITY_RE = re.compile(
    rb"\n {6}[A-Za-z0-9._-]+ "
    rb"(v\d+\.\d+\.\d+(?:-(?:rc|alpha|beta)\d*)?(?:-\d+-g[0-9a-f]{7,40})?(?:-dirty)?"
    rb"|[0-9a-f]{7,40}(?:-dirty)?) \(")


def identity(data: bytes):
    """The build identity a signed image carries, or None when the image
    embeds none or more than one candidate."""
    found = {m.group(1).decode("ascii") for m in _IDENTITY_RE.finditer(data)}
    return found.pop() if len(found) == 1 else None


def tlvs(data: bytes) -> dict:
    """The unprotected TLVs of an image `parse()` accepted, by type."""
    _, _, hdr_size, prot_size, img_size, *_ = HEADER.unpack_from(data)
    off = hdr_size + img_size + prot_size
    _, total = TLV_INFO.unpack_from(data, off)
    end = off + total
    off += TLV_INFO.size
    out = {}
    while off + TLV_ENTRY.size <= end:
        tag, length = TLV_ENTRY.unpack_from(data, off)
        off += TLV_ENTRY.size
        out[tag] = data[off:off + length]
        off += length
    return out
