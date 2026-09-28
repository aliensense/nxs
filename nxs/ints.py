# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""The integer spelling the shipped schemas admit: a YAML integer as is, a
string with a base prefix (`0x30`, `0o60`, `0b11`) in that base, a digit-only
string in base 10 with leading zeros allowed (`010` is ten)."""
import re

_DECIMAL = re.compile(r"[+-]?[0-9]+")


def parse_int(text: str) -> int:
    """A string integer by the schema's rule; ValueError for any other
    spelling."""
    s = str(text).strip()
    if _DECIMAL.fullmatch(s):
        return int(s, 10)
    return int(s, 0)
