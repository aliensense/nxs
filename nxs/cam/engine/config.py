# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""YAML-backed configuration loading and sequence normalization."""

from __future__ import annotations

from typing import Any, Dict, List, Optional


from .models import (
    ParsedConfig,
    Meta,
    CmdStep,
    DeviceStep,
    ReadStep,
    ExpectStep,
    RetryBlockStep,
    Step,
)


class ConfigHandler:
    def __init__(self, data: Dict[str, Any]) -> None:
        self._raw: Dict[str, Any] = data
        self._cfg = self._parse(self._raw)
        #: Sequences whose exhausted I2C retries are a note, not an error.
        self.best_effort = frozenset(str(n) for n in (data.get("best_effort") or []))

    def build_event_plan(self) -> List[tuple[str, List[Step]]]:
        """The (sequence name, normalized steps) plan in ``event_list`` order;
        KeyError when ``event_list`` names a missing sequence."""
        if self._cfg is None:
            raise RuntimeError("Config not loaded. Call load_config() first.")

        plan: List[tuple[str, List[Step]]] = []
        for name in self._cfg.event_list:
            if name not in self._cfg.sequences:
                raise KeyError(f"event_list references missing sequence: {name!r}")
            plan.append((name, self._cfg.sequences[name]))
        return plan

    def get_parsed_config(self) -> ParsedConfig:
        """The cached parsed configuration."""
        if self._cfg is None:
            raise RuntimeError("Config not loaded. Call load_config() first.")
        return self._cfg

    # ---------------------------
    # Parsing / normalization
    # ---------------------------

    def _parse(self, raw: Dict[str, Any]) -> ParsedConfig:
        """Convert the raw YAML mapping into a ParsedConfig; ValueError on a
        section of the wrong type."""
        meta_raw = raw.get("meta") or {}
        if not isinstance(meta_raw, dict):
            raise ValueError("meta must be a mapping")

        meta = Meta(
            name=str(meta_raw.get("name", "")),
            version=str(meta_raw.get("version", "")),
        )

        aliases = raw.get("aliases") or {}
        if not isinstance(aliases, dict):
            raise ValueError("aliases must be a mapping")

        commands = self._ensure_int_map(
            aliases.get("commands") or {}, "aliases.commands"
        )
        addresses = self._ensure_int_map(
            aliases.get("addresses") or {}, "aliases.addresses"
        )

        event_list = raw.get("event_list") or []
        if not isinstance(event_list, list):
            raise ValueError("event_list must be a list")
        event_list = [str(x) for x in event_list]

        sequences_raw = raw.get("sequences") or {}
        if not isinstance(sequences_raw, dict):
            raise ValueError("sequences must be a mapping")

        sequences: Dict[str, List[Step]] = {}
        for seq_name, steps_raw in sequences_raw.items():
            sequences[str(seq_name)] = self._parse_sequence_steps(
                seq_name=str(seq_name),
                steps_raw=steps_raw,
                commands=commands,
                addresses=addresses,
            )

        return ParsedConfig(
            meta=meta,
            commands=commands,
            addresses=addresses,
            event_list=event_list,
            sequences=sequences,
        )

    def _parse_sequence_steps(
        self,
        seq_name: str,
        steps_raw: Any,
        commands: Dict[str, int],
        addresses: Dict[str, int],
    ) -> List[Step]:
        """Normalize one sequence into typed steps; ValueError on a bad step shape,
        KeyError on an undefined alias."""
        if not isinstance(steps_raw, list):
            raise ValueError(f"Sequence '{seq_name}' must be a list")

        out: List[Step] = []
        for i, item in enumerate(steps_raw):
            if not isinstance(item, dict):
                raise ValueError(f"Sequence '{seq_name}' step {i} must be a mapping")

            comment = item.get("comment")
            comment = str(comment) if comment is not None else None
            sleep_ms = 0
            if "sleep_ms" in item and item["sleep_ms"] is not None:
                sleep_ms = self._to_int(item["sleep_ms"], f"{seq_name}[{i}].sleep_ms")

            # Read step: {read: ADR_ALIAS, reg, offset, length?, store?}
            if "read" in item:
                out.append(
                    self._parse_read(
                        item, addresses, where=f"{seq_name}[{i}]",
                        sleep_ms=sleep_ms, comment=comment,
                    )
                )
                continue

            # Expect step: {expect|poll: ADR_ALIAS, reg, offset, value, mask?, op?,
            # timeout_ms?, poll_ms?}; poll requires timeout_ms.
            if "expect" in item or "poll" in item:
                out.append(
                    self._parse_expect(
                        item, addresses, where=f"{seq_name}[{i}]",
                        sleep_ms=sleep_ms, comment=comment,
                    )
                )
                continue

            # Retry block: {retry: TIMES, delay_ms?, on_fail?, steps: [...]}
            if "retry" in item:
                out.append(
                    self._parse_retry(
                        item, commands, addresses,
                        seq_name=seq_name, index=i, comment=comment,
                    )
                )
                continue

            # Command step
            if "cmd" in item:
                cmd_name = item["cmd"]
                if not isinstance(cmd_name, str):
                    raise ValueError(
                        f"Sequence '{seq_name}' step {i}: cmd must be a string"
                    )
                if cmd_name not in commands:
                    raise KeyError(
                        f"Unknown cmd alias {cmd_name!r} in sequence '{seq_name}' step {i}"
                    )

                args_raw = item.get("args") or []
                if not isinstance(args_raw, list):
                    raise ValueError(
                        f"Sequence '{seq_name}' step {i}: args must be a list"
                    )
                if "ms" in item:
                    # A wait is spelled `ms:` in captured corpora and in args in
                    # composed plans: one spelling per step, wait command only.
                    if args_raw:
                        raise ValueError(
                            f"Sequence '{seq_name}' step {i}: ms and args are two "
                            "spellings of one wait; use one"
                        )
                    if cmd_name != "CMD_WAIT_MILLIS":
                        raise ValueError(
                            f"Sequence '{seq_name}' step {i}: ms belongs to "
                            f"CMD_WAIT_MILLIS, not {cmd_name}"
                        )
                    args_raw = [1, item["ms"]]

                args = [self._to_int(v, f"{seq_name}[{i}].args") for v in args_raw]
                out.append(
                    CmdStep(
                        kind="cmd",
                        cmd=commands[cmd_name],
                        args=args,
                        sleep_ms=sleep_ms,
                        comment=comment,
                    )
                )
                continue

            # Device step
            if "device" in item:
                dev_name = item["device"]
                if not isinstance(dev_name, str):
                    raise ValueError(
                        f"Sequence '{seq_name}' step {i}: device must be a string"
                    )
                if dev_name not in addresses:
                    raise KeyError(
                        f"Unknown address alias {dev_name!r} in sequence '{seq_name}' step {i}"
                    )

                width = self._to_int(item.get("width", 1), f"{seq_name}[{i}].width")
                reg = self._to_byte(item.get("reg"), f"{seq_name}[{i}].reg")
                offset = self._to_byte(item.get("offset", 0), f"{seq_name}[{i}].offset")
                raw_value = item.get("value")
                if isinstance(raw_value, list):
                    value = [
                        self._to_byte(v, f"{seq_name}[{i}].value[{j}]")
                        for j, v in enumerate(raw_value)
                    ]
                else:
                    value = self._to_byte(raw_value, f"{seq_name}[{i}].value")

                out.append(
                    DeviceStep(
                        kind="device",
                        device=addresses[dev_name],
                        width=width,
                        reg=reg,
                        offset=offset,
                        value=value,
                        sleep_ms=sleep_ms,
                        comment=comment,
                    )
                )
                continue

            raise ValueError(
                f"Sequence '{seq_name}' step {i} must contain 'cmd', 'device', "
                "'read', 'expect', 'poll' or 'retry'"
            )

        return out

    def _resolve_device(
        self, alias: Any, addresses: Dict[str, int], where: str
    ) -> int:
        """Resolve a device alias to its numeric address; ValueError when not a
        string, KeyError when undefined."""
        if not isinstance(alias, str):
            raise ValueError(f"{where}: device alias must be a string")
        if alias not in addresses:
            raise KeyError(f"Unknown address alias {alias!r} in {where}")
        return addresses[alias]

    def _parse_read(
        self,
        item: Dict[str, Any],
        addresses: Dict[str, int],
        where: str,
        sleep_ms: int,
        comment: Optional[str],
    ) -> ReadStep:
        """Normalize a read step (``read`` holds the device alias)."""
        device = self._resolve_device(item["read"], addresses, where)
        store = item.get("store")
        return ReadStep(
            kind="read",
            device=device,
            reg=self._to_byte(item.get("reg"), f"{where}.reg"),
            offset=self._to_byte(item.get("offset", 0), f"{where}.offset"),
            length=self._to_int(item.get("length", 1), f"{where}.length"),
            store=str(store) if store is not None else None,
            sleep_ms=sleep_ms,
            comment=comment,
        )

    def _parse_expect(
        self,
        item: Dict[str, Any],
        addresses: Dict[str, int],
        where: str,
        sleep_ms: int,
        comment: Optional[str],
    ) -> ExpectStep:
        """Normalize an expect/poll step; ``poll`` requires ``timeout_ms``, ``op``
        is eq or ne."""
        verb = "expect" if "expect" in item else "poll"
        device = self._resolve_device(item[verb], addresses, where)
        timeout_ms = self._to_int(item.get("timeout_ms", 0), f"{where}.timeout_ms")
        if verb == "poll" and timeout_ms <= 0:
            raise ValueError(f"{where}: poll requires timeout_ms > 0")
        op = str(item.get("op", "eq"))
        if op not in ("eq", "ne"):
            raise ValueError(f"{where}: op must be 'eq' or 'ne', got {op!r}")
        poll_ms = self._to_int(item.get("poll_ms", 50), f"{where}.poll_ms")
        if poll_ms <= 0:
            raise ValueError(
                f"{where}: poll_ms must be positive, got {poll_ms}"
            )
        soft = item.get("soft", False)
        if not isinstance(soft, bool):
            raise ValueError(f"{where}: soft must be a boolean, got {soft!r}")
        if soft and timeout_ms <= 0:
            raise ValueError(f"{where}: a soft expect needs timeout_ms > 0")
        return ExpectStep(
            kind="expect",
            device=device,
            reg=self._to_byte(item.get("reg"), f"{where}.reg"),
            offset=self._to_byte(item.get("offset", 0), f"{where}.offset"),
            value=self._to_byte(item.get("value", 0), f"{where}.value"),
            mask=self._to_byte(item.get("mask", 0xFF), f"{where}.mask"),
            op=op,
            timeout_ms=timeout_ms,
            poll_ms=poll_ms,
            sleep_ms=sleep_ms,
            comment=comment,
            soft=soft,
        )

    def _parse_retry(
        self,
        item: Dict[str, Any],
        commands: Dict[str, int],
        addresses: Dict[str, int],
        seq_name: str,
        index: int,
        comment: Optional[str],
    ) -> RetryBlockStep:
        """Normalize a retry block, recursing into its nested steps; ``on_fail`` is
        abort or continue, ``steps`` must be non-empty."""
        where = f"{seq_name}[{index}]"
        times = self._to_int(item["retry"], f"{where}.retry")
        on_fail = str(item.get("on_fail", "abort"))
        if on_fail not in ("abort", "continue"):
            raise ValueError(
                f"{where}: on_fail must be 'abort' or 'continue', got {on_fail!r}"
            )
        steps_raw = item.get("steps")
        if not isinstance(steps_raw, list) or not steps_raw:
            raise ValueError(f"{where}: retry requires a non-empty steps list")
        nested = self._parse_sequence_steps(
            seq_name=f"{seq_name}[{index}].steps",
            steps_raw=steps_raw,
            commands=commands,
            addresses=addresses,
        )
        return RetryBlockStep(
            kind="retry",
            times=times,
            delay_ms=self._to_int(item.get("delay_ms", 0), f"{where}.delay_ms"),
            on_fail=on_fail,
            steps=nested,
            comment=comment,
        )

    def _ensure_int_map(self, m: Any, label: str) -> Dict[str, int]:
        """A mapping with string keys and integer values; ValueError otherwise."""
        if not isinstance(m, dict):
            raise ValueError(f"{label} must be a mapping")
        out: Dict[str, int] = {}
        for k, v in m.items():
            out[str(k)] = self._to_int(v, f"{label}.{k}")
        return out

    def _to_int(self, v: Any, where: str) -> int:
        """An int-like value (int, or a string in any base spelling) as ``int``;
        ValueError otherwise."""
        if v is None:
            raise ValueError(f"{where}: expected int-like value, got None")
        if isinstance(v, bool):
            raise ValueError(f"{where}: expected int-like value, got bool: {v}")
        if isinstance(v, int):
            return v
        if isinstance(v, str):
            from nxs.ints import parse_int

            try:
                return parse_int(v)
            except ValueError:
                raise ValueError(f"{where}: not an integer spelling: {v!r}") from None
        raise ValueError(
            f"{where}: expected int-like value, got {type(v).__name__}: {v!r}"
        )

    def _to_byte(self, v: Any, where: str) -> int:
        """`_to_int` bounded to one bus byte: the executor sends 8-bit
        data behind a 16-bit address split into (reg, offset), so a wider
        value would be silently masked on the wire."""
        n = self._to_int(v, where)
        if not 0 <= n <= 0xFF:
            raise ValueError(f"{where}: {v!r} is not one byte (0..255)")
        return n

    def _get_by_dot_path(self, data: Any, path: str) -> Any:
        """Resolve a dotted path against nested dicts; None when absent."""
        cur = data
        for part in path.split("."):
            if isinstance(cur, dict) and part in cur:
                cur = cur[part]
            else:
                return None
        return cur
