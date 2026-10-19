# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The fault block of `nxs <unit> status`: the counters a unit absorbed and
the errno behind the latest of each, printed only when one is nonzero."""

from __future__ import annotations

from nxs.capabilities import SupportsFaultCounters
from nxs.device_errors import errno_name


def last_errno(code: int) -> str:
    """The errno behind a fault count, as ` (last EBUSY)`; nothing for 0."""
    return f" (last {errno_name(code)})" if code else ""


def print_faults(t) -> None:
    """Print the Faults block for a transport that reads the counters."""
    if isinstance(t, SupportsFaultCounters):
        # Firmware without the fault-counter surface refuses the
        # read; status stays useful without the lines.
        overflow_read = getattr(t, "read_cmd_queue_overflow_count", None)
        try:
            io_err = t.read_io_err_count()
            io_errno = t.read_io_errno()
            probe_failed = t.read_probe_failed_count()
            drdy = t.read_drdy_coalesced_count()
            rejects = t.read_ingress_reject_count()
            heals = t.read_bus_heal_count()
            heal_errno = t.read_bus_heal_errno()
            overflows = overflow_read() if overflow_read else None
        except (RuntimeError, OSError, TimeoutError):
            pass
        else:
            # A healthy device stays quiet: the block appears only when a
            # counter is nonzero, so a Faults line always means something.
            if any((io_err, probe_failed, drdy, rejects, heals, overflows)):
                print(f"  Faults:       {io_err} I/O errors absorbed{last_errno(io_errno)}, "
                      f"{probe_failed} probe failures")
                print(f"                {drdy} samples missed (VM busy at DRDY), "
                      f"{rejects} commands rejected (bus contention)")
                if heals:
                    print(f"                {heals} bus heals{last_errno(heal_errno)}")
                if overflows is not None:
                    print(f"                {overflows} writes dropped "
                          f"(device queue full)")
