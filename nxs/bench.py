"""End-to-end bring-up harness — `nxs bench`.

Walks an operator through a fresh-board check:
  1. Build and flash MCUboot + firmware (per-domain so the mass-erase
     scopes to MCUboot only and the signed app isn't re-erased).
  2. Probe the NXS over the chosen transport.
  3. For each sensor on the bench: pick its driver, upload, stream a
     burst, save to the next sequential slot.
  4. After all sensors are saved, reset the board with one of them
     wired up and verify the runner auto-binds via the persisted slot.

Every step pauses for Enter; `s` skips, `q` quits. The driver menu
lists every concrete `SensorDriver` subclass discovered at runtime,
so new drivers appear without harness edits.

Calls into the rest of `nxs` directly — no subprocess to a
sibling `nxs` binary, no argv string-marshalling, no stale
default-address pitfalls. The `west` flash step shells out via
`subprocess` and bails with a clear message if west isn't on PATH.
"""

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

from nxs import cli as _cli
from nxs.driver_select import discover_drivers, pick_driver
from nxs.term import banner, info, err, GREEN, YELLOW, RED, CYAN, NC


# Repo root: bench.py lives at sdk/nxs/bench.py.
# Walking up three parents lands at the firmware repo root, which is
# what `west` expects as its working directory.
REPO_ROOT = Path(__file__).resolve().parents[2]


def prompt(label: str) -> str:
    """Return `'go'` | `'skip'` | `'quit'` from the operator's keypress."""
    while True:
        a = input(
            f"{YELLOW}[{label}]{NC} Enter=run, s=skip, q=quit: "
        ).strip().lower()
        if a == "":
            return "go"
        if a in ("s", "skip"):
            return "skip"
        if a in ("q", "quit", "exit"):
            return "quit"


def _call(base_args, cmd_fn, transport, **overrides):
    """Invoke an `nxs.cli.cmd_*` function with a synthesised
    Namespace built from `base_args` plus per-call overrides. Returns
    the (ok, exception-or-None) pair so callers can decide whether to
    continue the bench loop."""
    ns = argparse.Namespace(**vars(base_args))
    for k, v in overrides.items():
        setattr(ns, k, v)
    try:
        cmd_fn(transport, ns)
        return True, None
    except Exception as exc:
        return False, exc


def print_probe_diagnostic(args) -> None:
    """One-screen guide listing the usual probe-failure causes for the
    selected transport. Called when `cmd_probe` reports the board is
    silent so the operator can see what to check before retrying."""
    err("Probe failed — the board didn't acknowledge.")
    if args.transport == "i2c":
        print(f"  Bus:    {args.bus}")
        print(f"  I²C @:  0x{args.addr:02x}")
        print( "  Common causes:")
        print( "    • I²C address mismatch — check the RTT line")
        print( "      `I2C target registered @ 0xNN`.")
        print( "    • Bus device doesn't exist on this host.")
        print( "    • Pull-ups missing or weak; bus capacitance too high.")
    elif args.transport == "cyphal-can":
        print(f"  Iface:  {args.port or 'can0'}")
        print( "  Common causes:")
        print( "    • SocketCAN interface down or wrong name.")
        print( "    • Bitrate / sample-point mismatch (1 Mbps arb, 4 Mbps data).")
        print( "    • No other node on the bus, or a node-ID collision.")
    else:
        print(f"  Wire:   {args.port} @ {args.baud} baud")
        print( "  Common causes:")
        print( "    • TX/RX wired backwards on the USB-UART.")
        print( "    • Cable lands on the wrong UART (not the host link).")
        print( "    • Baud mismatch with the firmware's devicetree config.")
    print( "  Use RTT to confirm the firmware booted and the comm thread")
    print( "  is up before assuming a wire problem.")


# ── Steps ──────────────────────────────────────────────────

def step_erase_and_flash(args) -> str:
    banner("Step 1 — Erase + flash MCUboot and firmware")
    print("Builds both sysbuild images, then writes MCUboot with a")
    print("chip mass-erase and the signed app without re-erasing.")
    print("A single `--erase` against multi-domain sysbuild erases")
    print("twice and wipes MCUboot after writing it; the per-domain")
    print("split below scopes the erase to the mcuboot domain only.")
    decision = prompt("erase + flash")
    if decision == "quit":
        return "quit"
    if decision == "skip":
        return "skip"

    if shutil.which("west") is None:
        err("`west` is not on PATH — activate your Zephyr venv first "
            "(e.g. `source <your-zephyr-venv>/bin/activate`) or answer "
            "`skip` at the next erase+flash prompt.")
        return "skip"

    app_dir = REPO_ROOT / "apps" / "zephyr" / args.app

    def run_west(*west_args: str) -> None:
        rendered = "west " + " ".join(west_args)
        info(f"$ {rendered}")
        subprocess.check_call(["west", *west_args], cwd=REPO_ROOT)

    run_west("build", "-b", args.board, str(app_dir), "-p")
    run_west("flash", "--domain", "mcuboot", "--erase")
    run_west("flash", "--domain", args.app)
    return "go"


def step_probe(t, args) -> str:
    banner("Step 2 — Probe the NXS")
    print(f"Transport: {args.transport}; checking the board is alive.")
    decision = prompt("probe")
    if decision == "quit":
        return "quit"
    if decision == "skip":
        return "skip"
    while True:
        ok, exc = _call(args, _cli.cmd_probe, t)
        if ok:
            return "go"
        # A permission error, a missing smbus2, or a timeout already carrying
        # the mux-holder state is a better answer than the wiring checklist.
        if exc is not None:
            err(f"probe failed: {exc}")
        print_probe_diagnostic(args)
        a = input(
            f"\n{YELLOW}[probe]{NC} Enter=retry, s=skip, q=quit: "
        ).strip().lower()
        if a in ("s", "skip"):
            return "skip"
        if a in ("q", "quit"):
            return "quit"


def step_upload_stream_save(t, args, driver_name: str, driver_cls,
                            slot: int) -> str:
    """Upload `driver_name`, stream a burst, save to `slot`. Each
    sub-action can be skipped independently. `driver_cls.__module__`
    is the authoritative module name (e.g. `NeoM9N` lives in
    `neo_m9n.py`)."""
    banner(f"Sensor — upload + stream + save → slot {slot}: {driver_name}")
    print(f"Wire up the {driver_name}-compatible sensor on mikroBUS.")

    decision = prompt(f"upload {driver_name}")
    if decision == "quit":
        return "quit"
    if decision == "skip":
        return "skip"
    module_name = driver_cls.__module__.rsplit('.', 1)[-1]
    ok, exc = _call(args, _cli.cmd_upload, t,
                    driver=module_name, config=None)
    if not ok:
        err(f"upload failed: {exc} — skipping rest of this sensor's steps")
        return "skip"

    decision = prompt(f"stream {args.stream_samples} samples")
    if decision == "quit":
        return "quit"
    if decision != "skip":
        ok, exc = _call(args, _cli.cmd_stream, t,
                        count=args.stream_samples, raw=False,
                        every=None, hz=None, driver=None, quiet=False)
        if not ok:
            err(f"stream failed: {exc} — continuing to save step")

    decision = prompt(f"save {driver_name} to slot {slot}")
    if decision == "quit":
        return "quit"
    if decision == "skip":
        return "skip"
    ok, exc = _call(args, _cli.cmd_save, t, slot=slot)
    if not ok:
        err(f"save failed: {exc} — slot {slot} likely unchanged")
        return "save_failed"
    return "go"


def step_verify_autobind(t, args, expected_name: str) -> str:
    banner(f"Auto-bind check — reset and verify {expected_name}")
    print(f"  1. Reconnect the {expected_name}-compatible sensor.")
    print( "  2. Press the hardware RESET button on the NXS.")
    print( "  3. Wait for boot; the runner reads slot 0, probes, and")
    print( "     advances through populated slots until a driver matches.")
    decision = prompt("verify auto-bind")
    if decision == "quit":
        return "quit"
    if decision == "skip":
        return "skip"

    info("Waiting 3 s for boot + first probe...")
    time.sleep(3)
    # `cmd_status` prints directly; capture stdout via redirection so we
    # can both display it AND parse the Driver: line.
    import io, contextlib
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ok, exc = _call(args, _cli.cmd_status, t)
    out = buf.getvalue()
    print(out, end="")
    if not ok:
        err(f"status query failed: {exc} — can't confirm auto-bind")
        return "skip"

    bound = next(
        (line.split(":", 1)[1].strip()
         for line in out.splitlines() if line.lstrip().startswith("Driver:")),
        "",
    )
    if bound.lower() == expected_name.lower():
        print(f"{GREEN}OK — NXS auto-bound to {expected_name}{NC}")
    else:
        err(f"FAIL — bound driver is '{bound or '(none)'}', "
            f"expected '{expected_name}'")
    return "go"


# ── Entry ──────────────────────────────────────────────────

def cmd_bench(t, args) -> int:
    """`nxs bench` dispatcher — drives the full bring-up flow."""
    drivers = discover_drivers()
    if not drivers:
        print("No drivers found under nxs.drivers", file=sys.stderr)
        return 1

    print(f"{CYAN}═══════════════════════════════════════════════════════{NC}")
    print(f"{CYAN}   NXS bring-up bench{NC}")
    print(f"{CYAN}═══════════════════════════════════════════════════════{NC}")
    info(f"Board:     {args.board}   App: {args.app}")
    if args.transport == "i2c":
        info(f"Transport: i2c — register-map on {args.bus} @ "
             f"0x{args.addr:02x}")
    elif args.transport == "cyphal-can":
        info(f"Transport: cyphal-can on {args.port or 'can0'}")
    else:
        info(f"Transport: cyphal-serial on {args.port} @ {args.baud} baud")
    info(f"Compiled drivers available: {', '.join(sorted(drivers))}")

    if step_erase_and_flash(args) == "quit":
        return 0
    if step_probe(t, args) == "quit":
        return 0

    # Upload + save loop: each iteration picks a driver, uploads it,
    # streams, and saves to the next sequential slot. Operator stops
    # the loop when they've provisioned every sensor on the bench.
    saved: list = []
    next_slot = 0
    while True:
        banner(f"Pick a sensor to provision (next slot = {next_slot})")
        pick = pick_driver(drivers)
        if pick is None:
            break
        name, cls = pick
        result = step_upload_stream_save(t, args, name, cls, next_slot)
        if result == "quit":
            return 0
        if result == "go":
            saved.append((name, next_slot))
            next_slot += 1
        # "skip" / "save_failed" → don't bump next_slot; flash state
        # for this slot is unchanged, so the next iteration retargets it.
        decision = prompt("provision another sensor")
        if decision == "quit":
            return 0
        if decision == "skip":
            break

    if not saved:
        info("No sensors provisioned — skipping auto-bind verification.")
        return 0

    banner("Auto-bind verification")
    print("Reconnect any of the saved sensors below; on reset the runner")
    print("will probe slot 0 then advance until a saved driver matches.")
    for i, (name, slot) in enumerate(saved, 1):
        print(f"  {i}) {name}  (slot {slot})")
    while True:
        a = input(
            f"{YELLOW}[reconnected sensor]{NC} number (Enter=skip, q=quit): "
        ).strip().lower()
        if not a or a == "s":
            break
        if a == "q":
            return 0
        try:
            expected = saved[int(a) - 1][0]
        except (ValueError, IndexError):
            err("Invalid choice — try again")
            continue
        if step_verify_autobind(t, args, expected) == "quit":
            return 0
        break

    print(f"\n{GREEN}═══ bench complete ═══{NC}")
    return 0
