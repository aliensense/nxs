"""What a device refuses with: the errno values it returns, the reason behind each code, and the exception a refusal raises."""

from nxs._generated_constants import OpErrors


class DeviceRefused(RuntimeError):
    """The device answered and said no, distinct from a dead link. `code` is
    the device's positive errno in its own libc numbering."""

    def __init__(self, code: int, reason: str):
        super().__init__(reason)
        self.code = code

# EBUSY against a command that needs the stage to itself; the store and the
# LOAD verdict share it, XFER_ERR_REASON keeps its own text.
SESSION_LIVE_REASON = ("a transfer session is live (an upload or firmware "
                       "push holds the device) — retry in a few seconds; a "
                       "session whose holder has gone silent is reclaimed "
                       "automatically")

# Device store-op errno to reason (device libc numbering). The I2C path reads
# the errno from CMD_ERROR, the Cyphal path from `aliensense.nxs.cmd_error`.
STORE_ERR_REASON = {
    16: SESSION_LIVE_REASON,                                            # EBUSY
    61: "no driver image loaded — upload a driver first",               # ENODATA
    17: "an identical driver image is already stored in another slot",  # EEXIST
    28: "the driver store is full",                                     # ENOSPC
    22: "invalid slot",                                                 # EINVAL
    2:  "no such slot",                                                 # ENOENT
    5:  "flash (NVS) write error",                                      # EIO
    134: "the store takes no hub image; the host runs those",           # ENOTSUP
}

# STORE_PERSIST (identity commit) shares the error channel with the store ops,
# but EINVAL here is a rejected record, not a bad slot.
COMMISSION_ERR_REASON = {
    22: "identity record rejected (node-id or subject-id out of range)",  # EINVAL
    5:  "flash (NVS) write error",                                        # EIO
}

# Transfer-session refusals, keyed by the device's errno numbers (Zephyr
# newlib values, which match Linux here; macOS differs).
XFER_ERR_REASON = {
    16: "another transfer session is live (a concurrent upload or firmware "
        "push) — retry when it ends, or release it with XFER_ABORT",  # EBUSY
    71: "out-of-sequence transfer op: wrong XFER_TYPE for this operation, "
        "or a config stage trampled by an interleaved write",         # EPROTO
    27: "image exceeds the device staging buffer",                    # EFBIG
    2:  "no transfer session to release",                             # ENOENT
    11: "the device dropped the command before dispatch (queue full) — "
        "retry",                                                      # EAGAIN
}

# CMD_LOAD's verdict: the device confirms the parse before "Uploaded" means
# anything.
LOAD_ERR_REASON = {
    16: SESSION_LIVE_REASON,                                            # EBUSY
    61: "the staged image arrived short of the announced size — bytes were "
        "lost in transit; re-run the upload",                         # ENODATA
    8:  "the device rejected the image: not a valid driver (stale SDK "
        "wheel or corrupt content — detail in the device log)",       # ENOEXEC
    11: "the device dropped the command before dispatch (queue full) — "
        "retry",                                                      # EAGAIN
    5:  "the file pull failed on the device (server silent past the "
        "retry budget, or a refused write) — retry the upload",       # EIO
}

# Calibration errnos in the device's numbering: its libc diverges from glibc
# above 71 (EBADMSG 77, ETIMEDOUT 116, ECANCELED 140).
CALIB_ERR_REASON = {
    22: "calibration record rejected (bad version, orientation, or non-finite values)",  # EINVAL
    19: "device has no calibration engine",                                              # ENODEV
    5:  "flash (NVS) write error",                                                       # EIO
    16: "another calibration procedure is running",                                      # EBUSY
    11: "insufficient rotation coverage or samples",                                     # EAGAIN
    34: "degenerate fit (cloud is not an ellipsoid)",                                    # ERANGE
    77: "fit failed the sphere self-check (iron near the unit, or a sweep "
        "that never inverted an axis) — the collection is closed, run again",           # EBADMSG
    116: "timed out waiting for stillness",                                              # ETIMEDOUT
    95: "the loaded driver has no source for this calibration",                 # EOPNOTSUPP
    61: "the loaded driver is not measuring — no samples for this calibration",  # ENODATA
    140: "procedure cancelled (host abort or driver change)",                           # ECANCELED
}

# DFU begin/write/finish errnos.
DFU_ERR_REASON = {
    16: "a firmware update is already in progress",   # EBUSY
    22: "image rejected (bad size, type, or state)",  # EINVAL
    5:  "flash write error",                          # EIO
    8:  "the staged bytes are not an MCUboot image — push "
        "zephyr.signed.bin, not zephyr.bin",          # ENOEXEC
    61: "no image data staged, or the image ends before the length its "
        "header declares",                            # ENODATA
    2:  "the DFU session was evicted (the host went silent mid-push) — "
        "restart the push",                           # ENOENT
}

# Cyphal file-pull start (LOAD_FROM_FILE / BEGIN_SOFTWARE_UPDATE) reasons; both
# commands share the device's single file client.
PULL_ERR_REASON = {
    16: "another file transfer is already in progress",  # EBUSY
    22: "no valid file-server node for the pull",        # EINVAL
    5:  "flash write error",                             # EIO
    8:  "the pulled file is not an MCUboot image — serve "
        "zephyr.signed.bin, not zephyr.bin",             # ENOEXEC
    61: "the pulled file ends before the length its header declares",  # ENODATA
    28: "the image is larger than the update slot",                     # ENOSPC
    116: "no response from the file server",                           # ETIMEDOUT
}

# The verdict of a finished run: `CAM_ERROR` on the terminal edge (the
# device's libc numbering: ETIMEDOUT 116, EILSEQ 138, ECANCELED 140).
CAM_VERDICT_REASON = {
    5:   "the sensor stopped answering past the register retries",      # EIO
    6:   "no sensor answered at the personality's address",             # ENXIO
    19:  "no sensor answered at the personality's address",             # ENODEV
    116: "a poll did not see its value within its timeout",             # ETIMEDOUT
    140: "the run was aborted",                                         # ECANCELED
    8:   "the slot holds a driver personality, not a camera one",      # ENOEXEC
    9:   "the slot's camera personality does not parse; reinstall it: "
         "nxs switch",                                                  # EBADF
    14:  "the personality's program faulted; the unit's log names the "
         "instruction",                                                 # EFAULT
    27:  "the personality's program is larger than the unit's camera "
         "runner holds",                                                # EFBIG
    71:  "the personality's program raised an error of its own; the "
         "unit's log names the instruction",                            # EPROTO
    134: "the personality's I2C profile tops out below the pod bus's "
         "clock",                                                       # ENOTSUP
    138: "a register read did not match the value the personality "
         "expects",                                                     # EILSEQ
}

# `Cmd::CAM_RUN`'s result, device numbering: the accept's refusal, or the verdict of a run
# that ended before the host read the accept; the accept's reason stands where both name a code.
CAM_RUN_ERR_REASON = {
    **CAM_VERDICT_REASON,
    8:  "the slot holds a driver personality, not a camera one",       # ENOEXEC
    2:  "the slot is empty — upload the camera personality and save it",  # ENOENT
    16: "the unit is busy: a run, an upload, or a firmware push holds "
        "the transfer session",                                         # EBUSY
    19: "the unit's firmware carries no camera runner",                 # ENODEV
    22: "a staged parameter value the personality does not accept",     # EINVAL
    11: "the device dropped the command before dispatch (queue full) — "
        "retry",                                                        # EAGAIN
}

# `Cmd::CAM_ABORT`'s result.
CAM_ABORT_ERR_REASON = {
    2:  "no camera run is live",                                        # ENOENT
    11: "the device dropped the command before dispatch (queue full) — "
        "retry",                                                        # EAGAIN
}

ERRNO_EEXIST = 17

# Device libc numbering, not the host's: a mag solve answers EAGAIN when the
# cloud is insufficient, and leaves the collection open to retry.
ERRNO_EAGAIN = 11

ERRNO_ENOENT = 2

ERRNO_EINVAL = 22

ERRNO_ENODATA = 61

# A calibration procedure on a personality without the vector it solves.
ERRNO_EOPNOTSUPP = 95

ERRNO_ECANCELED = 140

# A hub image offered to the store: the host runs those. A slot's peek
# answers it for an image of another format version.
ERRNO_ENOTSUP = 134

# A slot's peek on an image that does not parse.
ERRNO_EBADF = 9

# Raised at the I2C claim sites too (a mode readback that does not echo the
# write means the session is held), which never see a CMD_ERROR value.
ERRNO_EBUSY = 16

def exc_detail(exc: BaseException) -> str:
    """`str(exc)`, or the type name when that is empty (a bare `ImportError()`
    stringifies to "")."""
    return str(exc) or type(exc).__name__

def import_failure_detail(exc: BaseException, path: str) -> str:
    """`str(exc)` plus the file:line it happened at, walking the traceback to
    the last frame in `path`."""
    import traceback
    detail = f"{type(exc).__name__}: {exc}"
    # A SyntaxError from a module the driver imports carries that module's
    # position, not the driver's; trust it only when the files match.
    line = (exc.lineno if isinstance(exc, SyntaxError) and exc.filename == path
            else None)
    if line is None:
        for frame in reversed(traceback.extract_tb(exc.__traceback__)):
            if frame.filename == path:
                line = frame.lineno
                break
    return f"{detail} (line {line})" if line else detail

def err_reason(code: int, reasons=None) -> str:
    """Human-readable reason for a device errno, defaulting to the
    store vocabulary."""
    return (reasons or STORE_ERR_REASON).get(code, f"device error code {code}")

def op_error_name(code: int) -> str:
    """The `ERROR_CODE` byte as `TIMEOUT (17)`, or `driver code (N)` for
    a driver-defined value."""
    name = OpErrors.OpErrorCode._NAMES.get(code)
    return f"{name} ({code})" if name else f"driver code ({code})"

# The queued personality-info select: the unit's verdict on the slot asked for.
PERSONALITY_INFO_ERR_REASON = {
    2: "the slot is empty",
    8: "the slot holds a driver personality, which has no descriptor trailer",
    19: "the device has no personality store",
}
