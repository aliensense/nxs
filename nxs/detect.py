# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Naming the Click a unit carries when the unit runs nothing: every
personality the tool knows is uploaded in turn, and the one whose sensor
answers its probe stays running from RAM. `nxs generate` calls it for a
unit found without a personality, and seeds the manifest with the answer."""

import time
from typing import Callable, List, Optional, Tuple

from nxs._generated_constants import RunnerStates
from nxs.client import ACTIVE_SLOT, await_driver_up
from nxs.image import serialize


def candidates() -> List[Tuple[str, type, dict]]:
    """The (module, class, config) trials: every personality the tool
    knows, I2C first, then SPI, UART last. A two-bus part is tried once
    per bus."""
    from nxs.suite.reconcile import (DriverNotFound, known_driver_modules,
                                     load_unit_driver)
    passes = {"i2c": [], "spi": [], "uart": [], "other": []}
    for name in known_driver_modules():
        try:
            cls = load_unit_driver(name)
        except DriverNotFound:
            continue
        buses = tuple(getattr(cls, "BUSES", None) or ())
        if not buses:
            buses = (getattr(cls, "BUS", None) or "other",)
        for bus in buses:
            config = {"bus": bus} if len(buses) > 1 else {}
            passes.get(bus, passes["other"]).append((name, cls, config))
    return passes["i2c"] + passes["spi"] + passes["uart"] + passes["other"]


def identity_note(cls) -> str:
    """` (WHO_AM_I 0xA9, I2C 0x68/0x69)` from a personality's declared
    identity, or nothing for a part without one."""
    parts = []
    values = getattr(cls, "WHO_AM_I_VALUES", None) or []
    if values:
        parts.append(f"WHO_AM_I 0x{int(values[0]):02X}")
    addrs = getattr(cls, "I2C_ADDRS", None) or []
    if addrs:
        parts.append("I2C " + "/".join(f"0x{int(a):02X}" for a in addrs))
    return f" ({', '.join(parts)})" if parts else ""


def _await_loaded(t, timeout_s: float = 2.0) -> None:
    """Block until the runner has left a terminal state after a LOAD, so the
    next RUN is judged on its own probe; a stuck device is left to
    `await_driver_up`."""
    state = RunnerStates.RunnerState
    deadline = time.monotonic() + timeout_s
    while t.read_runner_state() in (state.MEASURING, state.PROBE_FAILED):
        if time.monotonic() >= deadline:
            return
        time.sleep(0.05)


def _restore_slot(t, before: int, report: Callable[[str], None]) -> None:
    """Leave the unit as the trials found it: the slot that was active
    reloads; a RAM-only personality cannot come back, so the last trial is
    cleared."""
    count = t.read_store_count()
    if count <= 0 or before == ACTIVE_SLOT:
        t.vm_reset()
        return
    for _ in range(count):
        t.cycle()
        await_driver_up(t)
        if t.read_active_slot() == before:
            return
    report(f"slot {before} did not come back after cycling the store "
           f"({count} populated); nxs store ls shows what runs")


def detect(t, report: Callable[[str], None] = lambda line: None) -> Optional[str]:
    """Upload each candidate and run it; the first whose sensor answers stays
    running and its name is returned. None when nothing answers, with the
    slot that ran before put back. `report` takes one line per trial."""
    state = RunnerStates.RunnerState
    before = t.read_active_slot()
    for name, cls, config in candidates():
        label = name if not config else f"{name} ({config['bus']})"
        try:
            img = serialize(cls().compile(dict(config)))
        except Exception as e:             # noqa: BLE001 (the personality's own code)
            report(f"{label}: skipped ({type(e).__name__}: {e})")
            continue
        t.upload_image(img)
        _await_loaded(t)
        t.vm_run()
        if await_driver_up(t) != state.MEASURING:
            report(f"{label}: no answer")
            continue
        report(f"{name} answers{identity_note(cls)}")
        return name
    _restore_slot(t, before, report)
    return None
