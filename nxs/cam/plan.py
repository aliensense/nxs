# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Composed-config container, flattening, and the never-write guard."""

from __future__ import annotations

import copy
from typing import Any, Dict, List, Optional, Set, Tuple

import yaml

from .contracts import ContractError
from .descriptors import STANDARD_ALIASES
from .engine import ConfigHandler
from .engine import SequenceExecutor
from .engine import SpyRunner


def _int(v: Any) -> int:
    """A step's integer field the way the executor reads it: an int as
    is, a string by the shared spelling rule (`0x30`, `0o60`, `48`,
    `010`)."""
    from nxs.ints import parse_int

    if isinstance(v, int) and not isinstance(v, bool):
        return v
    return parse_int(v)


#: Step keys that name the device a step addresses (the step grammar:
#: a write's `device`, a read's `read`, an `expect`/`poll`'s own key).
_DEVICE_KEYS = ("device", "read", "expect", "poll")


def _remap_devices(steps: List[Any], device_map: Dict[str, str]) -> List[Any]:
    """Copies of `steps` with every device alias renamed through `device_map`,
    in every step kind that names one and inside retry blocks."""
    out: List[Any] = []
    for step in steps:
        if not isinstance(step, dict):
            out.append(step)
            continue
        copy = dict(step)
        for key in _DEVICE_KEYS:
            if isinstance(copy.get(key), str):
                copy[key] = device_map.get(copy[key], copy[key])
        if isinstance(copy.get("steps"), list):
            copy["steps"] = _remap_devices(copy["steps"], device_map)
        out.append(copy)
    return out


def wait_budget_ms(step: Dict[str, Any]) -> int:
    """The milliseconds a raw CMD_WAIT_MILLIS step waits (either of the
    two shapes the blobs use: ``args: [1, ms]`` or ``ms:``)."""
    if "ms" in step:
        return _int(step["ms"])
    args = step.get("args") or [1, 0]
    return _int(args[-1])


class RawConfig:
    """A composed device_stack config built from descriptor output. Holds raw
    step dicts, so :meth:`to_yaml` output is itself a valid engine config."""

    def __init__(self, name: str) -> None:
        self._name = name
        self._aliases = copy.deepcopy(STANDARD_ALIASES)
        self._event_list: List[str] = []
        self._sequences: Dict[str, List[Dict[str, Any]]] = {}
        self._blob_sequences: Set[str] = set()
        #: Sequences whose I2C failures are notes: a device already off.
        self._best_effort: Set[str] = set()
        # Marker sequence -> (link, mode, trigger): what the link's unit runs
        # there. A side table: the engine config carries only the marker.
        self._unit_programs: Dict[str, Tuple[str, str, str]] = {}

    @property
    def unit_programs(self) -> Dict[str, Tuple[str, str, str]]:
        """Marker sequence name -> (link, mode, trigger), in no order; the
        event list gives the order."""
        return dict(self._unit_programs)

    def add_unit_program(self, name: str, link: str, mode: str,
                         trigger: str = "freerun") -> str:
        """Append the marker at which `link`'s unit runs its personality with
        `mode` and `trigger`; returns the (collision-suffixed) sequence
        name."""
        from .descriptors import unit_program

        before = set(self._sequences)
        self.add(name, [unit_program(link, mode, trigger)])
        unique = next(n for n in self._sequences if n not in before)
        self._unit_programs[unique] = (link, mode, trigger)
        return unique

    def is_unit_program(self, name: str) -> bool:
        """Whether a sequence is a unit-program marker."""
        return name in self._unit_programs

    def subset(self, names: List[str]) -> Dict[str, Any]:
        """The engine config of the named sequences only, in the given
        order (the host segments between markers)."""
        return {
            "meta": {"name": self._name, "version": "1"},
            "aliases": copy.deepcopy(self._aliases),
            "event_list": list(names),
            "sequences": {n: copy.deepcopy(self._sequences[n]) for n in names},
        }

    def set_address(self, alias: str, addr: int) -> None:
        """Point a device alias at a bus address; a second alias (``ADR_SENSOR_A``)
        carries a hub's other sensor when the two differ."""
        self._aliases.setdefault("addresses", {})[alias] = hex(int(addr))

    def address_of(self, alias: str) -> Optional[int]:
        """The bus address an alias resolves to, if declared."""
        value = (self._aliases.get("addresses") or {}).get(alias)
        return int(str(value), 0) if value is not None else None

    def add(
        self,
        name: str,
        steps: List[Dict[str, Any]],
        from_blob: bool = False,
        device_map: Optional[Dict[str, str]] = None,
        best_effort: bool = False,
    ) -> None:
        """Append a named sequence (suffixed on collision). ``from_blob`` exempts it
        from the runtime never-write guard; ``device_map`` renames step devices
        on copies of the steps; ``best_effort`` lets a device that does not
        answer end the sequence with a note instead of an error."""
        if device_map:
            steps = _remap_devices(steps, device_map)
        unique = name
        n = 2
        while unique in self._sequences:
            unique = f"{name}__{n}"
            n += 1
        self._sequences[unique] = steps
        self._event_list.append(unique)
        if from_blob:
            self._blob_sequences.add(unique)
        if best_effort:
            self._best_effort.add(unique)

    def add_blob(
        self,
        descriptor: Any,
        blob_name: str,
        prefix: str = "",
        insert_before: Optional[Dict[str, List[Dict[str, Any]]]] = None,
        append_after: Optional[Dict[str, List[Dict[str, Any]]]] = None,
        device_map: Optional[Dict[str, str]] = None,
        only_device: Optional[str] = None,
    ) -> None:
        """Append every sequence of a descriptor blob in its event order, on copies
        of the steps. ``insert_before`` / ``append_after`` map a blob sequence
        name to composed steps added as their own sequence around it;
        ``only_device`` keeps the steps of one alias (and the waits)."""
        before = insert_before or {}
        after = append_after or {}
        for seq_name, steps in descriptor.blob_sequences(blob_name):
            if seq_name in before:
                self.add(f"{prefix}before_{seq_name}", before[seq_name])
            steps = copy.deepcopy(steps)
            if only_device is not None:
                steps = [s for s in steps if s.get("device") in (None, only_device)]
            self.add(f"{prefix}{seq_name}", steps, from_blob=True, device_map=device_map)
            if seq_name in after:
                self.add(f"{prefix}after_{seq_name}", after[seq_name])

    @property
    def blob_sequences(self) -> Set[str]:
        return set(self._blob_sequences)

    def sequence_names(self) -> List[str]:
        """The event list in order."""
        return list(self._event_list)

    def steps_of(self, name: str) -> List[Dict[str, Any]]:
        """The raw steps of a sequence (the config's own list)."""
        return self._sequences[name]

    def insert_after(
        self, anchor: str, name: str, steps: List[Dict[str, Any]],
    ) -> str:
        """Add a sequence right after ``anchor`` in the event order; returns the
        (collision-suffixed) name it landed under."""
        if anchor not in self._sequences:
            raise KeyError(f"no sequence {anchor!r} to insert after")
        unique = name
        n = 2
        while unique in self._sequences:
            unique = f"{name}__{n}"
            n += 1
        self._sequences[unique] = steps
        self._event_list.insert(self._event_list.index(anchor) + 1, unique)
        return unique

    def take_settle(self, name: str, floor_ms: int = 0) -> int:
        """Strip the trailing settle of a sequence and return it in ms: the last
        step's ``sleep_ms`` (lowered to ``floor_ms``) or a trailing ``CMD_WAIT_MILLIS``
        (removed). 0 when the sequence ends on neither."""
        steps = self._sequences[name]
        if not steps:
            return 0
        last = steps[-1]
        if "cmd" in last and last.get("cmd") == "CMD_WAIT_MILLIS":
            budget = wait_budget_ms(last)
            steps.pop()
            return budget
        budget = _int(last.get("sleep_ms") or 0)
        if budget <= 0:
            return 0
        if floor_ms > 0:
            last["sleep_ms"] = min(budget, floor_ms)
        else:
            last.pop("sleep_ms", None)
        return budget

    def wrap_retry(
        self, name: str, times: int, delay_ms: int = 0,
        then: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """Re-run a sequence (plus ``then`` steps) as one bounded block: any failing
        step re-runs the whole block, up to ``times``."""
        steps = self._sequences[name]
        block = {"retry": times, "steps": list(steps) + list(then or [])}
        if delay_ms:
            block["delay_ms"] = delay_ms
        self._sequences[name] = [block]

    def written_value(self, device: str, reg16: int) -> Optional[int]:
        """The byte the last write step gives ``reg16`` (None if unwritten)."""
        found = None
        for steps in self._sequences.values():
            for step in steps:
                if not isinstance(step, dict): continue
                if step.get("device") != device: continue
                reg, off = step.get("reg"), step.get("offset")
                vals = step.get("value")
                if reg is None or vals is None: continue
                base = _int(reg)
                if off is not None:
                    base = (base << 8) | _int(off)
                span = vals if isinstance(vals, list) else [vals]
                if base <= reg16 < base + len(span):
                    found = _int(span[reg16 - base])
        return found

    def patch_write(self, device: str, reg16: int, value: int) -> int:
        """Replace the byte an existing write step gives ``reg16``, in place and
        spanning multi-byte value lists; returns the number of steps patched."""
        patched = 0
        for steps in self._sequences.values():
            for step in steps:
                if not isinstance(step, dict): continue
                if step.get("device") != device: continue
                reg, off = step.get("reg"), step.get("offset")
                vals = step.get("value")
                if reg is None or vals is None: continue
                base = _int(reg)
                if off is not None:
                    base = (base << 8) | _int(off)
                span = vals if isinstance(vals, list) else [vals]
                if not base <= reg16 < base + len(span): continue
                idx = reg16 - base
                new = (f"0x{value:02X}" if isinstance(span[idx], str)
                       else int(value))
                span[idx] = new
                if not isinstance(vals, list):
                    step["value"] = new
                patched += 1
        return patched

    def to_dict(self) -> Dict[str, Any]:
        doc = {
            "meta": {"name": self._name, "version": "1"},
            "aliases": copy.deepcopy(self._aliases),
            "event_list": list(self._event_list),
            "sequences": copy.deepcopy(self._sequences),
        }
        if self._best_effort:
            doc["best_effort"] = sorted(self._best_effort)
        return doc

    def to_yaml(self) -> str:
        return yaml.safe_dump(self.to_dict(), sort_keys=False, width=100)

    def config_handler(self) -> ConfigHandler:
        """Parse the composed config through the engine's own parser."""
        return ConfigHandler(self.to_dict())


NormStep = Tuple[Any, ...]


def flatten(
    config: "RawConfig | Dict[str, Any]",
    keep_sequences: bool = False,
) -> List[NormStep] | List[Tuple[str, List[NormStep]]]:
    """Flatten a config into a normalized stream: writes ``("w", dev, reg16, bytes)``,
    reads ``("r", dev, reg16, len)``, expects ``("e", dev, reg16, mask, op, value)``;
    waits drop out, retries flatten to their success path. ``keep_sequences`` groups."""
    if isinstance(config, RawConfig):
        handler = config.config_handler()
    else:
        handler = ConfigHandler(config)
    plan = handler.build_event_plan()

    result: List[Tuple[str, List[NormStep]]] = []
    for seq_name, steps in plan:
        spy = SpyRunner()
        SequenceExecutor(spy).execute_plan([(seq_name, steps)])
        norm: List[NormStep] = []
        for call in spy.calls:
            if call.name == "device_write":
                device, _width, reg, offset, value = call.args
                reg16 = (reg << 8) | (offset & 0xFF)
                values = tuple(value) if isinstance(value, list) else (value,)
                norm.append(("w", device, reg16, values))
            elif call.name == "read":
                device, reg, offset, length = call.args
                norm.append(("r", device, (reg << 8) | (offset & 0xFF), length))
            elif call.name == "expect":
                device, reg, offset, value, mask, op = call.args
                norm.append(
                    ("e", device, (reg << 8) | (offset & 0xFF), mask, op, value)
                )
            # cmd (waits) drop out of the normalized stream
        result.append((seq_name, norm))

    if keep_sequences:
        return result
    flat: List[NormStep] = []
    for _, norm in result:
        flat.extend(norm)
    return flat


def scan_forbidden(
    config: RawConfig,
    forbidden: Dict[int, List[int]],
) -> None:
    """Reject knob-emitted writes to runtime-forbidden registers (``forbidden``:
    device address -> 16-bit registers); blob-sourced sequences are exempt.
    Raises ContractError."""
    per_seq = flatten(config, keep_sequences=True)
    blob_names = config.blob_sequences
    for seq_name, steps in per_seq:  # type: ignore[union-attr]
        if seq_name in blob_names:
            continue
        for step in steps:
            if step[0] != "w":
                continue
            _, device, reg16, values = step[0], step[1], step[2], step[3]
            # A multi-byte write lands on every address from its start:
            # each one is judged, not only the first.
            banned = forbidden.get(device, [])
            for offset in range(max(len(values), 1)):
                if reg16 + offset in banned:
                    raise ContractError(
                        f"Sequence {seq_name!r} writes runtime-forbidden register "
                        f"0x{reg16 + offset:04X} on device 0x{device:02X}"
                        + (f" (byte {offset} of a {len(values)}-byte write at "
                           f"0x{reg16:04X})" if offset else "")
                        + " — refusing to emit (this is the des-killer class of write)"
                    )
