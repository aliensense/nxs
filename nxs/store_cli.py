"""`nxs store`: the personality store, one verb per flash-slot operation."""

import sys

from nxs._generated_constants import RunnerStates
from nxs.client import ERRNO_EEXIST, DeviceRefused, peek_slot, SupportsSlotPeek


def cmd_save(t, args):
    slot = args.slot
    if slot is None:
        # Default: next free slot (after currently-populated ones)
        slot = t.read_store_count()
    try:
        t.save_slot(slot)
    except DeviceRefused as e:
        if e.code == ERRNO_EEXIST:
            # The goal state (this image persisted) already holds.
            print("Already stored: an identical personality image occupies "
                  "another slot. Nothing to do.")
            return 0
        print(f"Save refused: {e}", file=sys.stderr)
        return 1
    except RuntimeError as e:
        print(f"Save failed: {e}", file=sys.stderr)
        return 1
    # STORE_COUNT is repainted on the runner's status cadence, not by the
    # save, so a read here still shows the pre-save count; `store ls` has it.
    print(f"Saved to slot {slot}.")
    return 0

def cmd_store_ls(t, args):
    count = t.read_store_count()
    active = t.read_active_slot()
    if count == 0:
        print("Store is empty.")
        return 0
    print(f"Personality store: {count}/8 populated")
    # Slot-name peek is a typed capability; a client without it only
    # knows the count.
    can_peek = isinstance(t, SupportsSlotPeek)
    session_held = False
    for i in range(count):
        marker = " ← active" if i == active else ""
        name = ""
        info = None
        if can_peek:
            info, held = peek_slot(t, i)
            session_held = session_held or held
            if info is not None and info.name:
                name = info.name
        elif i == active:
            # No per-slot peek on this transport: the active slot's
            # personality is loaded, so its name is readable from the device.
            name = t.read_personality_name()
        if name:
            extras = []
            kind = getattr(info, 'kind', None)
            if kind is not None:
                extras.append(kind)
            if info is not None and info.num_outputs:
                extras.append(f"{info.num_outputs} outputs")
            if info is not None and info.num_params:
                extras.append(f"{info.num_params} params")
            suffix = f"  ({', '.join(extras)})" if extras else ""
            print(f"  [{i}] {name}{suffix}{marker}")
        else:
            print(f"  [{i}]{marker}")
    if session_held:
        print("  (slot details unavailable: session held by a calibration "
              "procedure or firmware push)")
    return 0

def cmd_store_rm(t, args):
    try:
        t.delete_slot(args.slot)
    except DeviceRefused as e:
        print(f"Remove refused: {e}", file=sys.stderr)
        return 1
    except RuntimeError as e:
        print(f"Remove failed: {e}", file=sys.stderr)
        return 1
    # Same lag as cmd_save: a count read here is the pre-delete one.
    print(f"Removed slot {args.slot}.")
    return 0

def cmd_store_clear(t, args):
    try:
        t.clear_store()
    except DeviceRefused as e:
        print(f"Clear refused: {e}", file=sys.stderr)
        return 1
    except RuntimeError as e:
        print(f"Clear failed: {e}", file=sys.stderr)
        return 1
    print("Personality store cleared.")
    return 0

def cmd_cycle(t, args):
    from nxs.time_sync import cycle_driver
    old_slot = t.read_active_slot()
    # The cycled slot re-loads and re-probes asynchronously; a fixed sleep
    # either reports a stale slot or waits longer than the probe needs.
    cycle_driver(t)
    new_slot = t.read_active_slot()
    runner = RunnerStates.RunnerState._NAMES.get(t.read_runner_state(), "?")
    print(f"Cycled: slot {old_slot} → {new_slot}  state={runner}")
    return 0


def add_store_parser(sub) -> None:
    """Register `nxs store` and its slot verbs."""
    p_store = sub.add_parser('store', help='Manage the personality store (the flash slots)')
    store_sub = p_store.add_subparsers(dest='store_cmd', required=True)
    store_sub.add_parser('ls', help='List populated slots')
    p_save = store_sub.add_parser(
        'save', help='Save the last upload to a flash slot')
    p_save.add_argument('slot', type=int, nargs='?', default=None,
                        help='Slot index (default: next free)')
    p_store_rm = store_sub.add_parser('rm', help='Delete a slot')
    p_store_rm.add_argument('slot', type=int)
    store_sub.add_parser('clear', help='Wipe all slots')
    store_sub.add_parser(
        'cycle', help='Force advance to next populated slot')
