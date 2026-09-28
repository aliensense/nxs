# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
"""What every manifest parser shares: the error that names the YAML path and
the strict key and integer checks."""

class ManifestError(ValueError):
    """A manifest that failed validation: the fact names the YAML path, then
    one line per lawful alternative."""

    def __init__(self, fact: str, alternatives=None):
        self._fact = fact
        self._alternatives = list(alternatives or [])
        super().__init__("\n".join([fact, *(f"  - {alt}" for alt in self._alternatives)]))

    @property
    def fact(self) -> str:
        return self._fact

    @property
    def alternatives(self) -> list:
        return list(self._alternatives)


def _require_keys(mapping: dict, allowed: set, where: str):
    if not isinstance(mapping, dict):
        raise ManifestError(f"{where}: expected a mapping, got {type(mapping).__name__}")
    unknown = set(mapping) - allowed
    if unknown:
        raise ManifestError(
            f"{where}: unknown key(s) {sorted(unknown)} (allowed: {sorted(allowed)})")


def _parse_int(value, where: str) -> int:
    if isinstance(value, bool):
        raise ManifestError(f"{where}: expected an integer, got a boolean")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        from nxs.ints import parse_int

        try:
            return parse_int(value)
        except ValueError:
            pass
    raise ManifestError(f"{where}: expected an integer (any base), got {value!r}")
