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

# ENOTSUP from LOAD and the store: the image format gate (Interface Description §10.1). A hub, graph
# or module image the device log names never reaches a unit, whose store takes personalities alone.
FORMAT_NEWER_REASON = ("the unit does not take this image: it needs a newer firmware "
                       "(nxs <unit> push-fw <file>)")

# Device store-op errno to reason (device libc numbering). The I2C path reads
# the errno from CMD_ERROR, the Cyphal path from `aliensense.nxs.cmd_error`.
STORE_ERR_REASON = {
    16: SESSION_LIVE_REASON,                                            # EBUSY
    61: "no personality image loaded; upload a personality first",               # ENODATA
    17: "an identical personality image is already stored in another slot",  # EEXIST
    28: "the personality store is full",                                     # ENOSPC
    22: "invalid slot",                                                 # EINVAL
    2:  "no such slot",                                                 # ENOENT
    5:  "flash (NVS) write error",                                      # EIO
    134: FORMAT_NEWER_REASON,                                           # ENOTSUP
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
    16: "another session is live (an upload, a firmware push or a calibration "
        "procedure) — retry when it ends, or release it with SESSION_ABORT",  # EBUSY
    71: "a chunk arrived outside its session: no upload or firmware push is "
        "open",                                                       # EPROTO
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
    8:  "the device rejected the image: not a valid personality image (corrupt "
        "content — detail in the device log)",                        # ENOEXEC
    134: FORMAT_NEWER_REASON,                                           # ENOTSUP
    11: "the device dropped the command before dispatch (queue full) — "
        "retry",                                                      # EAGAIN
    5:  "the file pull failed on the device (server silent past the "
        "retry budget, or a refused write) — retry the upload",       # EIO
}

# Calibration errnos in the device's numbering: its libc diverges from glibc
# above 71 (EBADMSG 77, ETIMEDOUT 116, ECANCELED 140).
CALIB_ERR_REASON = {
    22: "calibration record rejected (bad version or non-finite values)",  # EINVAL
    19: "device has no calibration engine",                                              # ENODEV
    5:  "flash (NVS) write error",                                                       # EIO
    16: "another calibration procedure is running",                                      # EBUSY
    11: "insufficient rotation coverage or samples",                                     # EAGAIN
    34: "degenerate fit (cloud is not an ellipsoid)",                                    # ERANGE
    77: "fit failed the sphere self-check (iron near the unit, or a sweep "
        "that never inverted an axis) — the collection is closed, run again",           # EBADMSG
    116: "timed out waiting for stillness",                                              # ETIMEDOUT
    95: "the loaded personality has no source for this calibration",                 # EOPNOTSUPP
    61: "the loaded personality is not measuring; no samples for this calibration",  # ENODATA
    140: "procedure cancelled (host abort or personality change)",                           # ECANCELED
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

# The verdict of a camera run: the map's `ERROR` when its commit ends FAULT
# (the device's libc numbering: ETIMEDOUT 116, EILSEQ 138, ECANCELED 140).
CAM_VERDICT_REASON = {
    5:   "the sensor stopped answering past the register retries",      # EIO
    6:   "the personality names no I²C address or bus profile",         # ENXIO
    19:  "no sensor answered at the personality's address",             # ENODEV
    116: "a poll did not see its value within its timeout",             # ETIMEDOUT
    140: "the run was aborted",                                         # ECANCELED
    8:   "the slot holds a click personality, not a cam one",      # ENOEXEC
    9:   "the slot's cam personality does not parse; reinstall it: "
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
    133: "the slot took another image between the run's accept and its "
         "load; run it again",                                          # ESTALE
}

# A map commit that ended FAULT, the map's `ERROR`: its own refusals, else its run's verdict
# (`src/nxs/unit/DeviceErrors.h`, COMMIT then CAM_VERDICT).
COMMIT_ERR_REASON = {
    **CAM_VERDICT_REASON,
    22:  "the commit broke a rule: one owner and one kind, rows the door "
         "serves and may write, listed values, a RUN in standby",       # EINVAL
    140: "the commit was aborted",                                      # ECANCELED
}

#: The device's errno numbering, its C library's (newlib's): `ETIMEDOUT` is
#: 116 there and `EILSEQ` 138, where a Linux host would read other names.
DEVICE_ERRNO_NAMES = {
    1: "EPERM",
    2: "ENOENT",
    3: "ESRCH",
    4: "EINTR",
    5: "EIO",
    6: "ENXIO",
    7: "E2BIG",
    8: "ENOEXEC",
    9: "EBADF",
    10: "ECHILD",
    11: "EAGAIN",
    12: "ENOMEM",
    13: "EACCES",
    14: "EFAULT",
    15: "ENOTBLK",
    16: "EBUSY",
    17: "EEXIST",
    18: "EXDEV",
    19: "ENODEV",
    20: "ENOTDIR",
    21: "EISDIR",
    22: "EINVAL",
    23: "ENFILE",
    24: "EMFILE",
    25: "ENOTTY",
    26: "ETXTBSY",
    27: "EFBIG",
    28: "ENOSPC",
    29: "ESPIPE",
    30: "EROFS",
    31: "EMLINK",
    32: "EPIPE",
    33: "EDOM",
    34: "ERANGE",
    35: "ENOMSG",
    36: "EIDRM",
    37: "ECHRNG",
    38: "EL2NSYNC",
    39: "EL3HLT",
    40: "EL3RST",
    41: "ELNRNG",
    42: "EUNATCH",
    43: "ENOCSI",
    44: "EL2HLT",
    45: "EDEADLK",
    46: "ENOLCK",
    50: "EBADE",
    51: "EBADR",
    52: "EXFULL",
    53: "ENOANO",
    54: "EBADRQC",
    55: "EBADSLT",
    56: "EDEADLOCK",
    57: "EBFONT",
    60: "ENOSTR",
    61: "ENODATA",
    62: "ETIME",
    63: "ENOSR",
    64: "ENONET",
    65: "ENOPKG",
    66: "EREMOTE",
    67: "ENOLINK",
    68: "EADV",
    69: "ESRMNT",
    70: "ECOMM",
    71: "EPROTO",
    74: "EMULTIHOP",
    75: "ELBIN",
    76: "EDOTDOT",
    77: "EBADMSG",
    79: "EFTYPE",
    80: "ENOTUNIQ",
    81: "EBADFD",
    82: "EREMCHG",
    83: "ELIBACC",
    84: "ELIBBAD",
    85: "ELIBSCN",
    86: "ELIBMAX",
    87: "ELIBEXEC",
    88: "ENOSYS",
    89: "ENMFILE",
    90: "ENOTEMPTY",
    91: "ENAMETOOLONG",
    92: "ELOOP",
    95: "EOPNOTSUPP",
    96: "EPFNOSUPPORT",
    104: "ECONNRESET",
    105: "ENOBUFS",
    106: "EAFNOSUPPORT",
    107: "EPROTOTYPE",
    108: "ENOTSOCK",
    109: "ENOPROTOOPT",
    110: "ESHUTDOWN",
    111: "ECONNREFUSED",
    112: "EADDRINUSE",
    113: "ECONNABORTED",
    114: "ENETUNREACH",
    115: "ENETDOWN",
    116: "ETIMEDOUT",
    117: "EHOSTDOWN",
    118: "EHOSTUNREACH",
    119: "EINPROGRESS",
    120: "EALREADY",
    121: "EDESTADDRREQ",
    122: "EMSGSIZE",
    123: "EPROTONOSUPPORT",
    124: "ESOCKTNOSUPPORT",
    125: "EADDRNOTAVAIL",
    126: "ENETRESET",
    127: "EISCONN",
    128: "ENOTCONN",
    129: "ETOOMANYREFS",
    130: "EPROCLIM",
    131: "EUSERS",
    132: "EDQUOT",
    133: "ESTALE",
    134: "ENOTSUP",
    135: "ENOMEDIUM",
    136: "ENOSHARE",
    137: "ECASECLASH",
    138: "EILSEQ",
    139: "EOVERFLOW",
    140: "ECANCELED",
    141: "ENOTRECOVERABLE",
    142: "EOWNERDEAD",
    143: "ESTRPIPE",
}


def errno_name(code: int) -> str:
    """The name of a device errno in the device's own numbering, the number
    as text where it has none."""
    return DEVICE_ERRNO_NAMES.get(int(code), str(code))


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

# An image of another format than the unit runs, at LOAD, at the store and at a slot's peek, or a
# hub, graph or module image offered to the store.
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
    # A SyntaxError from a module the personality imports carries that module's
    # position, not the personality's; trust it only when the files match.
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
    """The `ERROR_CODE` byte as `TIMEOUT (17)`, or `device code (N)` for
    a personality-defined value."""
    name = OpErrors.OpErrorCode._NAMES.get(code)
    return f"{name} ({code})" if name else f"device code ({code})"
