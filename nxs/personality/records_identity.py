"""The IDENTITY record's head: what a block is, `IDENTITY_SIZE` bytes in the
`IdentityField` layout of constants/nxs_map.yaml, which every image kind
carries. A cam personality appends its head's bus profile to it
(`records_sensor`)."""

from __future__ import annotations

import dataclasses
import struct
from typing import Tuple

from nxs._generated_constants import NxsMap
from nxs.personality.records_fields import RecordError

Field = NxsMap.IdentityField
#: The head: address, flags, the firmware version, the serial's length, two
#: reserved bytes, then the serial, the compatible and the product.
HEAD = struct.Struct("<BBBBBB2x16s32s8s")
assert HEAD.size == NxsMap.IDENTITY_SIZE
_SERIAL_LEN = Field.COMPATIBLE - Field.SERIAL
_COMPATIBLE_LEN = Field.PRODUCT - Field.COMPATIBLE
_PRODUCT_LEN = NxsMap.IDENTITY_SIZE - Field.PRODUCT
#: The record's lengths: the head alone, or a camera's head and tail.
_SIZES = (NxsMap.IDENTITY_SIZE, NxsMap.IDENTITY_SIZE + NxsMap.IDENTITY_CAM_SIZE)


@dataclasses.dataclass(frozen=True)
class IdentityHead:
    """What a block is: its bus address (0 where it has none), its
    `IDENTITY_FLAG_*` bits, the firmware version it runs or was built
    against, its serial, its compatible and the product name the tool
    prints."""

    address: int = 0
    flags: int = 0
    fw: Tuple[int, int, int] = (0, 0, 0)
    serial: bytes = b""
    compatible: str = ""
    product: str = ""

    @property
    def name(self) -> str:
        """The block's name: the model its compatible names after the
        vendor, the whole compatible when it names no vendor."""
        return self.compatible.split(",", 1)[-1]


def encode_identity_head(head: IdentityHead, what: str) -> bytes:
    """The `IDENTITY_SIZE` bytes of a head. RecordError names a field the
    head cannot carry."""
    if not 0 <= int(head.address) <= 0x7F:
        raise RecordError(f"{what}: address {head.address:#x} is no 7-bit address")
    if not 0 <= int(head.flags) <= 0xFF:
        raise RecordError(f"{what}: flags {head.flags:#x} do not fit a byte")
    if len(head.fw) != 3 or not all(0 <= int(part) <= 0xFF for part in head.fw):
        raise RecordError(f"{what}: firmware {head.fw} is no major, minor and patch of a "
                          f"byte each")
    serial = bytes(head.serial)
    if len(serial) > _SERIAL_LEN:
        raise RecordError(f"{what}: a {len(serial)}-byte serial, the head holds {_SERIAL_LEN}")
    compatible = _ascii(head.compatible, _COMPATIBLE_LEN, f"{what} compatible")
    if not compatible:
        raise RecordError(f"{what}: a block names its compatible")
    return HEAD.pack(int(head.address), int(head.flags), *(int(part) for part in head.fw),
                     len(serial), serial, compatible,
                     _ascii(head.product, _PRODUCT_LEN, f"{what} product"))


def decode_identity_head(payload: bytes) -> IdentityHead:
    """The head of an IDENTITY record. RecordError for a record of another
    length than a head's or a camera's head and tail, or a field the head
    cannot hold."""
    if len(payload) not in _SIZES:
        raise RecordError(f"IDENTITY record of {len(payload)} B: a head is {_SIZES[0]} B and "
                          f"a camera's head and tail {_SIZES[1]} B")
    (address, flags, major, minor, patch, serial_len, serial, compatible,
     product) = HEAD.unpack_from(payload)
    if serial_len > _SERIAL_LEN:
        raise RecordError(f"IDENTITY record names a {serial_len}-byte serial, past the "
                          f"head's {_SERIAL_LEN}")
    return IdentityHead(address=address, flags=flags, fw=(major, minor, patch),
                        serial=bytes(serial[:serial_len]),
                        compatible=_text(compatible, "IDENTITY compatible"),
                        product=_text(product, "IDENTITY product"))


def _ascii(text: str, limit: int, what: str) -> bytes:
    try:
        raw = str(text).encode("ascii")
    except UnicodeEncodeError:
        raise RecordError(f"{what}: {text!r} carries a character past ASCII") from None
    if any(c < 0x20 or c > 0x7E for c in raw):
        raise RecordError(f"{what}: {text!r} is not printable ASCII")
    if len(raw) > limit:
        raise RecordError(f"{what}: {text!r} is longer than {limit} characters")
    return raw


def _text(raw: bytes, what: str) -> str:
    try:
        return bytes(raw).rstrip(b"\0").decode("ascii")
    except UnicodeDecodeError:
        raise RecordError(f"{what} carries a byte past ASCII") from None
