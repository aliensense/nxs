# AUTO-GENERATED from constants/*.yaml. DO NOT EDIT BY HAND.
# CI regenerates this on every build via
# `python3 scripts/generate-constants.py constants/`.

# ruff: noqa: E501


class NxsRegisters:

    class Reg:
        """
        I²C-target register map addresses.

        The register map is a 256-byte flat space indexed by u8. `Reg::*`
        are the named slots for VM status, command dispatch, driver
        metadata, and the per-descriptor window. Unary `+Reg::X` yields
        the underlying `uint8_t` for index math; comparison overloads let
        callers write `addr == Reg::CMD` without explicit casts on the
        I²C-ingress path that receives raw bytes from the wire.

        The selected-descriptor window (0xC0..0xE9) is an overlay: the
        last selector written — `PARAM_SELECT`, `OUTPUT_SELECT`, or
        `DRIVER_SELECT` — decides whether it exposes a parameter
        descriptor, an output descriptor, or the driver identity.
        `SEL_NAME*`, `SEL_TYPE`, and `SEL_UNIT*` mean the same in the
        param and output views; the `SEL_PARAM_*` and `SEL_OUTPUT_*`
        slots share addresses but are view-specific. The window starts
        at 0xC0 because `SAMPLE_DATA` spans 128 B (matching
        `vm::VM_SAMPLE_BUF_SIZE`). A parameter's value set is paged, not
        windowed: write an index to `SEL_VALUE_INDEX` and read the u32
        at `SEL_VALUE` — 5 bytes serving up to `vm::MAX_PARAM_VALUES`
        values. The driver view serves the loaded driver's name through
        `SEL_NAME_LEN`/`SEL_NAME` (there is no fixed driver-name window;
        a name is per-driver state, not a device-wide knob).

        `SERIAL` (0xF0..0xFB): a read-only 12-byte chip-UID96 window.
        `FW_VERSION_MAJOR`/`FW_VERSION_MINOR` (0xFC..0xFD): the running
        firmware version, mirroring Cyphal's `GetInfo.software_version`.

        A fixed `Reg::*` address is earned only by a device-wide knob every
        host must reach without a driver loaded (`DECIMATION`, `CMD`, the DFU
        window). Per-block and per-descriptor knobs are name-addressed through
        the `PARAM_SELECT` window here and `register.Access` over Cyphal, so a
        processing block adds a name, never a new fixed slot — the flat
        256-byte space is the scarce resource that rule protects.
        """
        WHO_AM_I = 0
        STATUS = 1  # bits: SAMPLE_READY / ERROR / RUNNING.
        VM_STATE = 2  # VmState enum.
        ERROR_CODE = 3  # VM error code synced from `vm_status`: 0 when the VM is healthy, otherwise the positive errno of the most recent VM fault (e.g. a probe/measure I/O error). Host command and DFU results report in `CMD_ERROR` instead.
        SAMPLE_COUNT_LO = 4
        SAMPLE_COUNT_HI = 5
        SAMPLE_SIZE = 6
        NUM_PARAMS = 7
        NUM_OUTPUTS = 8
        DRIVER_NAME_LEN = 9
        STORE_COUNT = 10  # populated slots in the driver store.
        ACTIVE_SLOT = 11  # 0xFF = transient (RAM-only) driver.
        RUNNER_STATE = 12  # RunnerState enum.
        PROBE_RETRIES = 13  # Current probe-retry counter.
        DECIMATION_LO = 14  # sample decimation factor LE (0=off, 1=every sample, N=every Nth).
        DECIMATION_HI = 15
        CMD = 16  # Single-byte Cmd dispatch. Bit 0x80 is the optional doorbell — host may write `(opcode | 0x80)` to disambiguate `Cmd::LOAD` (opcode 0) from "no command pending". Firmware masks the high bit before dispatch.
        PROGRAM_SIZE_LO = 17  # Bytecode image size, LE low byte. A 2-byte announce opens the upload session: it requires `XFER_TYPE = 0` and no live session, arms `CMD_ERR_PENDING`, and resolves 0 (session open) or EBUSY / EPROTO / EFBIG — the host reads that edge before streaming a single chunk.
        PROGRAM_SIZE_HI = 18
        PARAM_SELECT = 19  # Chooses which param descriptor is exposed.
        PARAM_SET_VALUE = 20  # 4 bytes (u32 LE).
        STORE_SELECT = 24  # Slot for SAVE / DELETE. Under `XFER_TYPE = CALIB` it doubles as the read-back page index: the `PROGRAM_DATA` window then serves calibration-record bytes [page*32 .. page*32+31] of the live record.
        PROTO_VERSION = 25  # Read-only. Version of the register-map contract (currently `PROTO_VERSION_VALUE`).
        XFER_TYPE = 26  # Selects the consumer of `PROGRAM_DATA` writes: 0 = VM bytecode (default; NXS upload), 1 = DFU image bytes (forwarded to the DFU sink), 2 = config record (identity commissioning; `PROGRAM_DATA` carries a `CONFIG_RECORD_SIZE`-byte record and `Cmd::STORE_PERSIST` commits it), 3 = time-sync record (a `TIME_SYNC_RECORD_SIZE`-byte volatile record applied as its 20th byte lands; never persisted), 4 = calibration record (`Calibration.RECORD_SIZE` bytes staged in 32-byte chunks, applied by `Cmd::CALIB_APPLY`), 5 = build identity (read-only `git describe` string, `BUILD_INFO_SIZE` bytes). The `_regs[PROGRAM_DATA..]` window mirrors the current record for read-back in modes 2 and 3; modes 4 and 5 serve their record through the same window page-by-page via `STORE_SELECT`. A write during a live transfer session (an open upload or DFU push) is refused silently — the mode does not move, and the writer detects refusal by reading it back; sessions end at their terminator (`Cmd::LOAD`, `DFU_FINISH`), by `Cmd::XFER_ABORT`, or after `XFER_SESSION_STALE_MS` of holder silence.
        XFER_PHASE = 27  # Read-only transfer phase. During a DFU it is a `DfuPhase` the master polls to pace `DFU_BEGIN`'s erase and detect a rejected write (detail in `CMD_ERROR`). While an on-device calibration procedure runs (started by `Cmd::CAL_GYRO`/`CAL_MAG_START`) it is the `Calibration.CalState` (0 idle, 1 gyro-wait-still, 2 gyro-averaging, 3 mag-collect); the procedure completes when it returns to idle, its verdict in `CMD_ERROR`. DFU and calibration never run at once — a procedure claims the transfer session when its start command is accepted (`Cmd::CAL_GYRO`/`CAL_MAG_START` and their Cyphal equivalents answer `EBUSY` when a contender holds it) and releases it on the edge back to idle, so the contender's gate refuses the overlap. IDLE when neither is active.
        XFER_ACK = 28  # Read-only single-byte progress counter, untearable under an I²C poll. During a DFU it is the accepted-chunk counter (mod 256): 0 after `DFU_BEGIN`, +1 per committed `PROGRAM_DATA` chunk; the master paces its push on it. While a calibration procedure runs it is the detail: gyro percent averaged, or mag rotation coverage on a 0-14 scale, where 12 is the stopping point at which a host asks the solve (which can still answer EAGAIN and keep the collection open).
        CMD_ERROR = 29  # Read-only result of the most recent host command op: 0 = OK, otherwise the positive errno value. Owned by the command-dispatch path and, while it holds the transfer session, a completing calibration procedure — the `vm_status` heartbeat never writes it, the DFU data path never zeroes it, and a bystander's refused `XFER_TYPE` write is silent — so a pending result cannot be clobbered — with one narrow exception: a command *refused* EBUSY under a held calibration session latches here, and the procedure's verdict landing in the same ~100 ms comm pass overwrites it, reading as the procedure's outcome rather than the refusal (and `ERROR_CODE` stays with VM health). The async ops — `LOAD`, store SAVE / DELETE_SLOT / CLEAR_STORE / STORE_PERSIST / PEEK_SLOT, `DFU_BEGIN`, `XFER_ABORT`, and a `PROGRAM_SIZE` announce — read `CMD_ERR_PENDING` from enqueue until the deferred handler resolves them; a host polls its own edge. Refusals resolve here too: EBUSY (a transfer session is live), EPROTO (wrong mode for the op), EFBIG (announce exceeds staging), ENODATA (`LOAD` on a short stage), ENOEXEC (`LOAD` parse failure), EAGAIN (the command queue was full and the op was dropped before dispatch — retry).
        DESCRIPTOR_EPOCH = 30  # Read-only descriptor-set generation: 0 until the first driver load, then 1..255 (wrapping past 0), bumped on every driver (re)load. A host caches descriptors against this and re-reads when it changes.
        OUTPUT_SELECT = 31  # Writable. Chooses which output descriptor the SEL window exposes — and switches the window to the output view.
        PROGRAM_DATA = 32  # 32-byte incoming-bytecode window.
        SAMPLE_DATA = 64  # 128-byte latest-sample record window (0x40..0xBF): `latch_time_us u64 | timestamp_us u64 | seq u16 | data[SAMPLE_SIZE]`, all little-endian (offsets `SAMPLE_RECORD_*`). One latched read returns a coherent record; `latch_time_us` is stamped by the I²C target at first byte, so every poll doubles as a two-way time-sync observation.
        SEL_NAME_LEN = 192
        SEL_NAME = 193  # 16 bytes. Driver view: the loaded driver's name.
        SEL_TYPE = 209  # Param view: bits[3:0] param_type (0 = enum, 1 = range), bit[4] ParamKind (0 = reload, 1 = live). Output view: NXS field_type code.
        SEL_PARAM_DEFAULT = 210  # Param view: 4 bytes (u32 LE).
        SEL_DRIVER_NUM_PARAMS = 211  # Driver/peek view: declared parameter count.
        SEL_DRIVER_NUM_OUTPUTS = 212  # Driver/peek view: declared output count.
        SEL_DRIVER_SLOT = 213  # Peek view: the peeked slot; 0xFF in the live driver view.
        SEL_PARAM_CURRENT = 214  # Param view: 4 bytes (u32 LE).
        SEL_DRIVER_I2C_ADDR = 215  # Peek view: latched mikroBUS I2C address of the active driver (0 = none / stored slot).
        SEL_PARAM_NUM_VALS = 218
        SEL_OUTPUT_SCALE = 210  # Output view: 4 bytes (f32 LE) — the effective scale; full-precision f64 descriptors are served over Cyphal GetOutputInfo.
        SEL_OUTPUT_OFFSET = 214  # Output view: 4 bytes (f32 LE).
        SEL_OUTPUT_BYTE_ORDER = 218  # Output view: sample-data endianness: 0 = big, 1 = little.
        SEL_UNIT_LEN = 219
        SEL_UNIT = 220  # 8 bytes.
        SEL_VALUE_INDEX = 228  # Param view, writable: which declared value `SEL_VALUE` exposes (0..num_values-1). Resets to 0 on `PARAM_SELECT`. In the diag view: which `DiagCounter` `SEL_VALUE` exposes; resets to 0 on entering the view.
        SEL_OUTPUT_SEMANTIC = 228  # Output view: semantic category code (0 = generic).
        SEL_VALUE = 229  # Param view: 4 bytes (u32 LE) — values[SEL_VALUE_INDEX]. Diag view: the selected fault counter, u16 zero-extended to u32 LE. An out-of-range index reads 0 in both views.
        SEL_OUTPUT_COUNT = 229  # Output view: 2 bytes (u16 LE) — string payload width, 0 for numeric fields.
        SEL_OUTPUT_AT = 231  # Output view: the field's byte position within the sample (gaps are legal).
        DRIVER_SELECT = 233  # Writable. Writing `DRIVER_VIEW_DIAG` switches the SEL window to the diag view (fault counters via `SEL_VALUE_INDEX`/`SEL_VALUE`, indices in `DiagCounter`); any other write switches to the driver view (name via SEL_NAME_LEN/SEL_NAME). Echoes 1 in the live driver view, `DRIVER_VIEW_PEEK` after Cmd::PEEK_SLOT, and `DRIVER_VIEW_DIAG` in the diag view.
        DECIMATION_SELECT = 234  # Writable per-subject decimation selector: a SubjectBucket value (0..NUM_SUBJECT_BUCKETS-1). Commit is deferred — the host polls until the register echoes the written value; an out-of-range or unbound select echoes `SELECTOR_INACTIVE`. Selecting repaints `DECIMATION_VALUE` with that subject's live factor.
        DECIMATION_VALUE = 235  # 2 bytes (u16 LE), read/write: the selected subject's decimation factor (0 = off, 1 = every device-output sample, N = every Nth), thinning only the Cyphal SI fan-out — the register mirror of `aliensense.nxs.decimation.<subject>`. Both bytes must be written in one transaction. Volatile until `Cmd::STORE_PERSIST` snapshots the live factors.
        CAN_TERM = 237  # RW CAN split-termination selection: 0 = off (the uncommissioned default), 1 = on, 0xFF = revert to the default. A write stages the persisted selection (committed by `Cmd::STORE_PERSIST` / a Cyphal Save) and the comm loop live-applies it on boards with the termination pin; an unsaved change reverts at reboot. Reads mirror the effective state (0 or 1); an out-of-vocabulary write is ignored.
        SERIAL = 240  # Read-only 12-byte chip UID96 (STM32 unique ID), MSB-first, app-injected. 0 until set.
        FW_VERSION_MAJOR = 252  # Read-only firmware version major, app-injected from the build's VERSION file — the same value Cyphal serves in `GetInfo.software_version`. 0 until seeded.
        FW_VERSION_MINOR = 253  # Read-only firmware version minor. 0xFE is reserved for a future patch byte.

        _NAMES = {
            0: 'WHO_AM_I',
            1: 'STATUS',
            2: 'VM_STATE',
            3: 'ERROR_CODE',
            4: 'SAMPLE_COUNT_LO',
            5: 'SAMPLE_COUNT_HI',
            6: 'SAMPLE_SIZE',
            7: 'NUM_PARAMS',
            8: 'NUM_OUTPUTS',
            9: 'DRIVER_NAME_LEN',
            10: 'STORE_COUNT',
            11: 'ACTIVE_SLOT',
            12: 'RUNNER_STATE',
            13: 'PROBE_RETRIES',
            14: 'DECIMATION_LO',
            15: 'DECIMATION_HI',
            16: 'CMD',
            17: 'PROGRAM_SIZE_LO',
            18: 'PROGRAM_SIZE_HI',
            19: 'PARAM_SELECT',
            20: 'PARAM_SET_VALUE',
            24: 'STORE_SELECT',
            25: 'PROTO_VERSION',
            26: 'XFER_TYPE',
            27: 'XFER_PHASE',
            28: 'XFER_ACK',
            29: 'CMD_ERROR',
            30: 'DESCRIPTOR_EPOCH',
            31: 'OUTPUT_SELECT',
            32: 'PROGRAM_DATA',
            64: 'SAMPLE_DATA',
            192: 'SEL_NAME_LEN',
            193: 'SEL_NAME',
            209: 'SEL_TYPE',
            210: 'SEL_PARAM_DEFAULT',
            211: 'SEL_DRIVER_NUM_PARAMS',
            212: 'SEL_DRIVER_NUM_OUTPUTS',
            213: 'SEL_DRIVER_SLOT',
            214: 'SEL_PARAM_CURRENT',
            215: 'SEL_DRIVER_I2C_ADDR',
            218: 'SEL_PARAM_NUM_VALS',
            219: 'SEL_UNIT_LEN',
            220: 'SEL_UNIT',
            228: 'SEL_VALUE_INDEX',
            229: 'SEL_VALUE',
            231: 'SEL_OUTPUT_AT',
            233: 'DRIVER_SELECT',
            234: 'DECIMATION_SELECT',
            235: 'DECIMATION_VALUE',
            237: 'CAN_TERM',
            240: 'SERIAL',
            252: 'FW_VERSION_MAJOR',
            253: 'FW_VERSION_MINOR',
        }

    class Cmd:
        """
        Command codes written to `Reg::CMD`.
        """
        LOAD = 0  # Parse and activate the staged bytecode as the loaded driver (does not run it). Ends the upload session. Async like SAVE: arms `CMD_ERR_PENDING`, then `CMD_ERROR` reads 0 (driver staged) or +errno — ENODATA when the stage is shorter than the announce (look at the transport), ENOEXEC when the bytes are not a valid driver image (stale wheel, corruption — detail in the device log). "Uploaded" on the host means the device confirmed the parse.
        RUN = 1
        STOP = 2
        RESET = 3
        SAVE = 4  # Save RAM image → store[STORE_SELECT].
        DELETE_SLOT = 5  # Remove store[STORE_SELECT].
        CLEAR_STORE = 6  # Wipe all stored drivers.
        CYCLE = 7  # Force advance to next populated slot.
        DFU_BEGIN = 8  # Erase the secondary slot and open a DFU staging session.
        DFU_FINISH = 9  # Close the session, mark the swap pending, reboot to apply.
        REBOOT = 10  # Reboot the device (a pending swap applies on next boot).
        STORE_PERSIST = 11  # Validate the staged identity config (the `CONFIG_RECORD_SIZE`-byte record written to `PROGRAM_DATA` under `XFER_TYPE = CONFIG`) and commit it to NVS, together with the live decimation state and the running calibration record. Refuses EPROTO with no live stage — a host persisting a solve wants `Cmd::CALIB_PERSIST`. Async like SAVE: arms `CMD_ERR_PENDING`, then `CMD_ERROR` reads 0 (committed) or a positive errno (e.g. EINVAL for an out-of-range address). Identity changes take effect at the next reboot.
        IDENTIFY = 12  # Strobe the status LED (~10 s) so an operator can physically locate the unit.
        PEEK_SLOT = 13  # Peek store[STORE_SELECT] without loading it (STORE_SELECT = 0xFF peeks the active driver). Async like SAVE: arms `CMD_ERR_PENDING`, then `CMD_ERROR` reads 0 (peek view valid) or +errno — ENOENT empty/out-of-range slot, EBADF corrupt image, ENOTSUP foreign image version. On success the SEL window switches to the peek view: name via `SEL_NAME_LEN`/`SEL_NAME`, counts via `SEL_DRIVER_NUM_PARAMS`/`SEL_DRIVER_NUM_OUTPUTS`, `SEL_DRIVER_SLOT` echoes the peeked slot, `SEL_DRIVER_I2C_ADDR` the latched mikroBUS address (0 unless peeking the active driver), and `DRIVER_SELECT` echoes `DRIVER_VIEW_PEEK`. The view is a snapshot — re-issue after store mutations.
        ENTER_RECOVERY = 14  # Arm the MCUboot serial-recovery flag and cold-reset into it. The device then holds in the bootloader until a host completes an mcumgr image upload — there is no timed window to catch. Dispatched independently of the VM handler, like `REBOOT`, so it works on a device whose driver never loaded. The Cyphal equivalent is the `ENTER_RECOVERY` (0xA008) ExecuteCommand.
        XFER_ABORT = 15  # Deliberately close the open transfer session (bytecode upload or DFU push) and cancel any calibration procedure in progress, whichever transport started it. A cancelled procedure applies nothing; the previous record stays, and the engine's verdict (ECANCELED, visible in the Cyphal progress register) is consumed on this bus by the abort's own acknowledgement. `Cmd::CAL_MAG_STOP` is the other way out of a collection and it solves and applies, so it cannot express a cancel. Arms `CMD_ERR_PENDING`; resolves 0 when something was released, ENOENT when there was nothing to do. The abort path for a host giving up mid-transfer — a crashed host needs nothing: an idle session is reclaimable by the next contender after `XFER_SESSION_STALE_MS`.
        CALIB_APPLY = 16  # Validate the staged calibration record (`Calibration.RECORD_SIZE` bytes written to `PROGRAM_DATA` under `XFER_TYPE = CALIB`) and apply it to the running state; `Cmd::CALIB_PERSIST` persists it. Async like SAVE: arms `CMD_ERR_PENDING`, then `CMD_ERROR` reads 0 (applied), EBUSY while a DFU or a running procedure holds the status registers, or a positive errno (EINVAL for a malformed record).
        CAL_GYRO = 17  # Start the on-device gyro still-average: the device waits for stillness, averages the rates, and applies the bias to the running state (Save persists). `CMD_ERROR` reports acceptance (0; EBUSY during a mag collection or while a transfer session is live; EOPNOTSUPP when the active driver has no gyro vector; ENODEV with no driver); progress reads from `XFER_PHASE`/`XFER_ACK` under `XFER_TYPE = CALIB`, and the completion verdict lands in `CMD_ERROR` when the phase returns to idle.
        CAL_MAG_START = 18  # Start the on-device mag collection: rotate the vehicle while the device accumulates the ellipsoid fit; coverage progress reads from `XFER_ACK` under `XFER_TYPE = CALIB`. `CMD_ERROR` reports acceptance (0; EBUSY during a gyro procedure or while a transfer session is live; EOPNOTSUPP when the active driver has no mag vector; ENODEV with no driver).
        CAL_MAG_STOP = 19  # Close the mag collection: coverage-gate, solve, self-check, and apply on success. `CMD_ERROR` carries the verdict — 0 applied, EAGAIN insufficient coverage/points (the collection stays open and its give-up window restarts), ERANGE degenerate (non-ellipsoid) fit, EBADMSG failed the sphere self-check, EBUSY while a DFU holds the status registers; every failure keeps the previous calibration.
        CALIB_PERSIST = 20  # Commit the running calibration record to NVS, leaving identity untouched — the save a host performs after a solve, when it has no identity record to stage. Snapshots the same live state a full Save does (the calibration bank and the decimation factors), so both save forms persist one consistent set; the Cyphal equivalent is `COMMAND_STORE_PERSISTENT_STATES`. Async like SAVE: arms `CMD_ERR_PENDING`, then `CMD_ERROR` reads 0 (committed), EBUSY while a DFU or a running procedure holds the status registers, ENODEV on a device with no calibration bank, or a positive errno on a validation or write failure.

        _NAMES = {
            0: 'LOAD',
            1: 'RUN',
            2: 'STOP',
            3: 'RESET',
            4: 'SAVE',
            5: 'DELETE_SLOT',
            6: 'CLEAR_STORE',
            7: 'CYCLE',
            8: 'DFU_BEGIN',
            9: 'DFU_FINISH',
            10: 'REBOOT',
            11: 'STORE_PERSIST',
            12: 'IDENTIFY',
            13: 'PEEK_SLOT',
            14: 'ENTER_RECOVERY',
            15: 'XFER_ABORT',
            16: 'CALIB_APPLY',
            17: 'CAL_GYRO',
            18: 'CAL_MAG_START',
            19: 'CAL_MAG_STOP',
            20: 'CALIB_PERSIST',
        }

    class DfuPhase:
        """
        DFU transfer phase, reported in `Reg::XFER_PHASE`.

        The I²C master paces a DFU push on two single-byte registers: it
        writes `Cmd::DFU_BEGIN`, polls `XFER_PHASE` until `READY` (the
        begin-time slot erase has finished), streams chunks to
        `PROGRAM_DATA` while watching `XFER_ACK` count them, writes
        `Cmd::DFU_FINISH`, then reads the terminal phase. `ERROR` (with
        the detail in `CMD_ERROR`) is cleared by the next successful
        write or `DFU_BEGIN`. Both registers are one byte wide, so a poll
        is served in a single ISR transaction and can never observe a
        torn value.
        """
        IDLE = 0
        ERASING = 1
        READY = 2
        WRITING = 3
        FINISHING = 4
        ERROR = 5

        _NAMES = {
            0: 'IDLE',
            1: 'ERASING',
            2: 'READY',
            3: 'WRITING',
            4: 'FINISHING',
            5: 'ERROR',
        }

    class DiagCounter:
        """
        Index namespace of the SEL diag view: `SEL_VALUE_INDEX` picks the counter, `SEL_VALUE` serves it.

        Entered by writing `DRIVER_VIEW_DIAG` to `Reg::DRIVER_SELECT`. Each
        counter is cumulative since boot and saturates at 65535 — these are
        should-be-zero fault counters, so a saturated value stays sticky
        evidence instead of wrapping back to a healthy-looking small number.
        The host reads a delta across its observation window. The served
        value is a snapshot taken at the selector write — it never repaints
        under the reader, so an awaited echo guarantees a tear-free read;
        re-write the index to refresh.
        """
        DRDY_COALESCED = 0  # DRDY edges that arrived while the previous wake was still pending — the VM was busy, that interval's sample was never taken. Counts from the driver load, and only once the loaded driver has begun waiting for data-ready: an idle line's edges miss nothing and never register. Sourced from `vm_status`, so it refreshes at the ~100 ms status cadence.
        INGRESS_REJECTS = 1  # Host commands rejected by the transport arbiter because a session was live on another transport. The reject is silent on the losing bus (its writes are dropped before any register logic); this counter is the wire-visible evidence. I²C reads and the diagnostic-view selectors both bypass the arbiter claim, so the losing I²C host can select the diag view and read it mid-rejection.
        I2C_CMD_QUEUE_OVERFLOWS = 2  # Host writes dropped because the I²C command queue was full (the host sees EAGAIN in `CMD_ERROR` when the dropped item had armed it).
        VM_IO_ERRORS = 3  # Measure-loop I/O errors the VM absorbed — the diag-view copy of `vm_status.vm_io_err_count`, refreshed at the ~100 ms status cadence.
        PROBE_FAILURES = 4  # Probe give-ups — the diag-view copy of `vm_status.probe_failed_count`, refreshed at the ~100 ms status cadence.

        _NAMES = {
            0: 'DRDY_COALESCED',
            1: 'INGRESS_REJECTS',
            2: 'I2C_CMD_QUEUE_OVERFLOWS',
            3: 'VM_IO_ERRORS',
            4: 'PROBE_FAILURES',
        }

    STATUS_SAMPLE_READY = 1  # `Reg::STATUS` bit: a fresh sample has landed in `Reg::SAMPLE_DATA`.

    STATUS_ERROR = 2  # `Reg::STATUS` bit: the VM is in an error state (`Reg::ERROR_CODE` carries the detail).

    STATUS_RUNNING = 128  # `Reg::STATUS` bit: the VM is currently in the measuring loop.

    WHO_AM_I_VALUE = 171  # Expected value at `Reg::WHO_AM_I`; lets host tooling probe a bus for an NXS target.

    SERIAL_LEN = 12  # Bytes in the `Reg::SERIAL` window (STM32 UID96 = 96 bits).

    DFU_ACK_TIMEOUT_MS = 1000  # Host-side wait for one DFU chunk's `XFER_ACK` edge before resending.

    XFER_SESSION_STALE_MS = 2000  # Transfer-session inactivity after which the device treats the holder as gone.

    PROGRAM_CHUNK_SIZE = 32  # Bytes per `Reg::PROGRAM_DATA` write window; the upload protocol assumes this matches the master's chunk size.

    SAMPLE_RECORD_LATCH_TIME_OFF = 0  # `Reg::SAMPLE_DATA` record: offset of `latch_time_us` (u64 LE) — the device clock at the I²C latch.

    SAMPLE_RECORD_TIMESTAMP_OFF = 8  # `Reg::SAMPLE_DATA` record: offset of the sample's acquisition `timestamp_us` (u64 LE, device clock).

    SAMPLE_RECORD_SEQ_OFF = 16  # `Reg::SAMPLE_DATA` record: offset of the sample sequence number (u16 LE, wraps at 2^16).

    SAMPLE_RECORD_DATA_OFF = 18  # `Reg::SAMPLE_DATA` record: offset of the sample bytes; `Reg::SAMPLE_SIZE` bytes follow.

    SAMPLE_RECORD_DATA_MAX = 110  # Max sample bytes the record window carries (128-byte window minus the 18-byte header); a driver's I²C-readable sample is capped here.

    PROTO_VERSION_VALUE = 1  # Current version of the register-map contract, exposed at `Reg::PROTO_VERSION`.

    XFER_TYPE_VM_BYTECODE = 0  # `Reg::XFER_TYPE` value: `Reg::PROGRAM_DATA` writes carry VM bytecode (default).

    XFER_TYPE_DFU_IMAGE = 1  # `Reg::XFER_TYPE` value: `Reg::PROGRAM_DATA` writes carry DFU image bytes.

    XFER_TYPE_CONFIG = 2  # `Reg::XFER_TYPE` value: `Reg::PROGRAM_DATA` carries the identity-config record (commission node/subject addresses, commit with `Cmd::STORE_PERSIST`).

    XFER_TYPE_TIME_SYNC = 3  # `Reg::XFER_TYPE` value: `Reg::PROGRAM_DATA` carries the volatile time-sync record — offset i64 LE, bound u32 LE, rate i32 LE, then validity window u32 LE; the 20th byte applies the discipline immediately (no persist). Reads mirror the live state at the `TIME_SYNC_RECORD_*` offsets.

    TIME_SYNC_RECORD_SIZE = 22  # Time-sync record: 8-byte offset + 4-byte bound + 4-byte rate + 4-byte validity window written by the host; the read mirror appends source and valid bytes.

    TIME_SYNC_RECORD_OFFSET_OFF = 0  # Time-sync record: offset of the i64 LE synced-time offset (µs).

    TIME_SYNC_RECORD_BOUND_OFF = 8  # Time-sync record: offset of the u32 LE error bound (µs).

    TIME_SYNC_RECORD_RATE_OFF = 12  # Time-sync record: offset of the i32 LE clock-rate correction (ppb).

    TIME_SYNC_RECORD_VALID_FOR_OFF = 16  # Time-sync record: offset of the u32 LE validity window (µs). The discipline expires this long after the push applies; the host tools push ten times their refresh cadence. 0 is malformed and leaves the discipline unset.

    TIME_SYNC_RECORD_SOURCE_OFF = 20  # Time-sync record mirror: the discipline source (0 none/stale, 1 host, 2 ring master, 3 GNSS). Ignored on write.

    TIME_SYNC_RECORD_VALID_OFF = 21  # Time-sync record mirror: 1 while a fresh discipline is in effect. Ignored on write.

    XFER_TYPE_CALIB = 4  # `Reg::XFER_TYPE` value: `Reg::PROGRAM_DATA` carries the calibration record (stage in 32-byte chunks, apply with `Cmd::CALIB_APPLY`, read back page-by-page via `Reg::STORE_SELECT`).

    XFER_TYPE_BUILD_INFO = 5  # `Reg::XFER_TYPE` value: the `PROGRAM_DATA` window serves the firmware build identity, read-only.

    BUILD_INFO_SIZE = 64  # Bytes in the build-identity record served through the `PROGRAM_DATA` window under `XFER_TYPE = BUILD_INFO` (two 32-byte pages).

    CONFIG_RECORD_SIZE = 28  # Bytes in the identity-config record exchanged through the `PROGRAM_DATA` window under `XFER_TYPE = CONFIG`.

    CMD_DOORBELL_BIT = 128  # CMD doorbell bit — host ORs this into the opcode so firmware distinguishes `Cmd::LOAD` (0) from "no command pending".

    CMD_OPCODE_MASK = 127  # Mask applied to `Reg::CMD` before dispatch, stripping `CMD_DOORBELL_BIT`.

    CMD_ERR_PENDING = 255  # `Reg::CMD_ERROR` value while an asynchronous store command is in flight.

    SELECTOR_INACTIVE = 255  # Echo value for the inactive selector (`PARAM_SELECT` / `OUTPUT_SELECT`); 0xFF is unambiguous since no driver declares 0xFF params/outputs.

    DRIVER_VIEW_PEEK = 2  # `DRIVER_SELECT` echo while the SEL window shows a peeked stored slot (the live driver view echoes 1).

    DRIVER_VIEW_DIAG = 3  # `DRIVER_SELECT` write/echo value selecting the diag view (fault counters via `SEL_VALUE_INDEX`/`SEL_VALUE`).

    DIAG_COUNTER_COUNT = 5  # Number of `DiagCounter` indices the diag view serves.

    SEL_TYPE_PARAM_TYPE_MASK = 15  # Param view: mask isolating `param_type` (0 = enum, 1 = range) in the low nibble of `Reg::SEL_TYPE`.

    SEL_TYPE_KIND_SHIFT = 4  # Param view: bit position of `ParamKind` (0 = reload, 1 = live) within `Reg::SEL_TYPE`; the descriptor window is full, so kind rides a spare bit of `SEL_TYPE` rather than its own register.

class Calibration:

    class Rotation:
        """
        Mounting-orientation codes for the calibration record.

        Names the physical rotation of the module relative to the vehicle
        body, as the composition R = Rz(yaw) * Ry(pitch) * Rx(roll) with
        each angle in {0, 90, 180, 270} degrees; the 24 distinct proper
        axis-aligned rotations, canonically named with the fewest terms.
        The device recovers body-frame vectors from sensor-frame ones as
        v_body = R * v_sensor, composing R with each vector bucket's
        calibration affine at record-change time. Every value carries its
        row-major 3x3 matrix (entries in {-1, 0, 1}); the generator
        validates det = +1 and emits the ROTATION_MATRIX table for the
        firmware and host appliers. Append only - never renumber.
        """
        NONE = 0
        YAW_90 = 1
        YAW_180 = 2
        YAW_270 = 3
        PITCH_90 = 4
        PITCH_180 = 5
        PITCH_270 = 6
        ROLL_90 = 7
        ROLL_180 = 8
        ROLL_270 = 9
        PITCH_90_YAW_90 = 10
        PITCH_90_YAW_180 = 11
        PITCH_90_YAW_270 = 12
        PITCH_180_YAW_90 = 13
        PITCH_180_YAW_270 = 14
        PITCH_270_YAW_90 = 15
        PITCH_270_YAW_180 = 16
        PITCH_270_YAW_270 = 17
        ROLL_90_YAW_90 = 18
        ROLL_90_YAW_180 = 19
        ROLL_90_YAW_270 = 20
        ROLL_90_PITCH_180 = 21
        ROLL_270_YAW_90 = 22
        ROLL_270_YAW_270 = 23

        _NAMES = {
            0: 'NONE',
            1: 'YAW_90',
            2: 'YAW_180',
            3: 'YAW_270',
            4: 'PITCH_90',
            5: 'PITCH_180',
            6: 'PITCH_270',
            7: 'ROLL_90',
            8: 'ROLL_180',
            9: 'ROLL_270',
            10: 'PITCH_90_YAW_90',
            11: 'PITCH_90_YAW_180',
            12: 'PITCH_90_YAW_270',
            13: 'PITCH_180_YAW_90',
            14: 'PITCH_180_YAW_270',
            15: 'PITCH_270_YAW_90',
            16: 'PITCH_270_YAW_180',
            17: 'PITCH_270_YAW_270',
            18: 'ROLL_90_YAW_90',
            19: 'ROLL_90_YAW_180',
            20: 'ROLL_90_YAW_270',
            21: 'ROLL_90_PITCH_180',
            22: 'ROLL_270_YAW_90',
            23: 'ROLL_270_YAW_270',
        }

        # Row-major 3x3 matrix per code (v_out = M * v_in).
        MATRIX = {
            0: (1, 0, 0, 0, 1, 0, 0, 0, 1),  # NONE
            1: (0, -1, 0, 1, 0, 0, 0, 0, 1),  # YAW_90
            2: (-1, 0, 0, 0, -1, 0, 0, 0, 1),  # YAW_180
            3: (0, 1, 0, -1, 0, 0, 0, 0, 1),  # YAW_270
            4: (0, 0, 1, 0, 1, 0, -1, 0, 0),  # PITCH_90
            5: (-1, 0, 0, 0, 1, 0, 0, 0, -1),  # PITCH_180
            6: (0, 0, -1, 0, 1, 0, 1, 0, 0),  # PITCH_270
            7: (1, 0, 0, 0, 0, -1, 0, 1, 0),  # ROLL_90
            8: (1, 0, 0, 0, -1, 0, 0, 0, -1),  # ROLL_180
            9: (1, 0, 0, 0, 0, 1, 0, -1, 0),  # ROLL_270
            10: (0, -1, 0, 0, 0, 1, -1, 0, 0),  # PITCH_90_YAW_90
            11: (0, 0, -1, 0, -1, 0, -1, 0, 0),  # PITCH_90_YAW_180
            12: (0, 1, 0, 0, 0, -1, -1, 0, 0),  # PITCH_90_YAW_270
            13: (0, -1, 0, -1, 0, 0, 0, 0, -1),  # PITCH_180_YAW_90
            14: (0, 1, 0, 1, 0, 0, 0, 0, -1),  # PITCH_180_YAW_270
            15: (0, -1, 0, 0, 0, -1, 1, 0, 0),  # PITCH_270_YAW_90
            16: (0, 0, 1, 0, -1, 0, 1, 0, 0),  # PITCH_270_YAW_180
            17: (0, 1, 0, 0, 0, 1, 1, 0, 0),  # PITCH_270_YAW_270
            18: (0, 0, 1, 1, 0, 0, 0, 1, 0),  # ROLL_90_YAW_90
            19: (-1, 0, 0, 0, 0, 1, 0, 1, 0),  # ROLL_90_YAW_180
            20: (0, 0, -1, -1, 0, 0, 0, 1, 0),  # ROLL_90_YAW_270
            21: (-1, 0, 0, 0, 0, -1, 0, -1, 0),  # ROLL_90_PITCH_180
            22: (0, 0, -1, 1, 0, 0, 0, -1, 0),  # ROLL_270_YAW_90
            23: (0, 0, 1, -1, 0, 0, 0, -1, 0),  # ROLL_270_YAW_270
        }

    class CalState:
        """
        On-device calibration procedure state.

        The procedure state exposed to hosts: the I2C `XFER_PHASE` register
        under `XFER_TYPE = CALIB`, and the first byte of the Cyphal
        `calibration.progress` register. The companion detail (I2C
        `XFER_ACK`) carries the gyro percent averaged or the mag rotation
        coverage on the `MAG_COVERAGE_FULL` scale, `MAG_COVERAGE_ENOUGH`
        being the stopping point; a completed procedure's verdict rides
        `CMD_ERROR` on I2C and the progress record's third byte on Cyphal.
        """
        IDLE = 0
        GYRO_WAIT_STILL = 1  # Waiting for the unit to be held still.
        GYRO_AVERAGING = 2  # Accumulating the at-rest rates.
        MAG_COLLECT = 3  # Accumulating the rotation cloud.

        _NAMES = {
            0: 'IDLE',
            1: 'GYRO_WAIT_STILL',
            2: 'GYRO_AVERAGING',
            3: 'MAG_COLLECT',
        }

    class BucketGuard:
        """
        How a stored bucket relates to the running sensor.

        The safety verdict of the whole feature: every applier, on the
        device and on the host, asks the record for it rather than
        re-deriving the tag comparison. A zero tag is the deliberate
        escape hatch for a hand-written record - applied, but nothing
        verifies it belongs to this sensor. The host renders the names
        lowercased in `calibrate show` and the suite status column.
        """
        BOUND = 0  # Solved against this sensor; apply it.
        UNGUARDED = 1  # Solved with no identity recorded; apply it, unverified.
        STALE = 2  # Solved against a different sensor; do not apply it.

        _NAMES = {
            0: 'BOUND',
            1: 'UNGUARDED',
            2: 'STALE',
        }

    CAL_RESULT_NONE = 255  # Progress-result byte meaning no procedure has completed since boot.

    MAG_COVERAGE_ENOUGH = 12  # Mag coverage at which a host stops the collection and asks.

    MAG_COVERAGE_FULL = 14  # Top of the mag coverage scale reported in `XFER_ACK`.

    RECORD_VERSION = 1  # Layout version of the calibration record (flash and wire).

    NUM_VECTORS = 3  # Vector buckets carrying a full affine (accel, gyro, mag).

    RECORD_SIZE = 168  # Packed size in bytes of the calibration record.

class CyphalDefaults:

    DEFAULT_NODE_ID = 125  # Compiled-default Cyphal node-ID, used when the device has no commissioned address in NVS.

    HOST_NODE_ID = 127  # Node-ID the host tooling (nxs / yakut / yukon) claims for its own Cyphal node.

    TEST_REF_NODE_ID = 126  # Node-ID the factory Ref emulator claims for its CAN host-mimic node.

    SAMPLE_SUBJECT_ID = 6144  # Default subject-ID for the sample stream (aliensense.nxs.RawSample), surfaced as uavcan.pub.sample.id.

    STATUS_SUBJECT_ID = 6145  # Default subject-ID for the aliensense.nxs.Status snapshot (uavcan.pub.status.id).

    ACCEL_SUBJECT_ID = 6146  # Default subject-ID for uavcan.si.sample.acceleration.Vector3 (uavcan.pub.acceleration.id). 0 disables the projection.

    GYRO_SUBJECT_ID = 6147  # Default subject-ID for uavcan.si.sample.angular_velocity.Vector3 (uavcan.pub.angular_velocity.id). 0 disables the projection.

    TEMPERATURE_SUBJECT_ID = 6148  # Default subject-ID for uavcan.si.sample.temperature.Scalar (uavcan.pub.temperature.id). 0 disables the projection.

    PRESSURE_SUBJECT_ID = 6149  # Default subject-ID for uavcan.si.sample.pressure.Scalar (uavcan.pub.pressure.id). 0 disables the projection.

    GNSS_SUBJECT_ID = 6150  # Default subject-ID for reg.udral.physics.kinematics.geodetic.PointStateVarTs (uavcan.pub.gnss.id). 0 disables the projection.

    MAGNETIC_FIELD_SUBJECT_ID = 6151  # Default subject-ID for uavcan.si.sample.magnetic_field_strength.Vector3 (uavcan.pub.magnetic_field.id). 0 disables the projection.

    SCALAR_SUBJECT_BASE = 6152  # Base subject-ID for the scalar uavcan.si.sample.*.Scalar block (uavcan.pub.scalar.id).

    CAN_BITRATE_DEFAULT = 1000000  # Default CAN arbitration bitrate in bit/s, reported by uavcan.can.bitrate when no profile is commissioned.

    CAN_BITRATE_DATA_DEFAULT = 4000000  # Default CAN-FD data-phase bitrate in bit/s, reported by uavcan.can.bitrate when no profile is commissioned.

class NxsDevices:

    class RBDevice:
        """
        System-wide device address registry.

        Device addresses in a pocket clear of common camera /
        GPIO-expander / serializer / EEPROM I²C addresses. One address per
        board, used on every host link (I²C target and frame address).
        Convert via `static_cast<DeviceAddress>(RBDevice::NXS)` for frame
        fields.
        """
        NXS = 48
        COMPUTE = 49
        RESERVED_0X32 = 50
        RESERVED_0X33 = 51

        _NAMES = {
            48: 'NXS',
            49: 'COMPUTE',
            50: 'RESERVED_0X32',
            51: 'RESERVED_0X33',
        }

    DEVICE_ADDRESS_BROADCAST = 255  # Wildcard destination — frames addressed to BROADCAST are accepted by every device on the bus.

class NxsDriverImage:

    class ParamKind:
        """
        How a parameter set is applied at runtime.

        `RELOAD` patches the bytecode operand and restarts the VM
        (re-running probe + configure — a sensor re-init); the value is
        baked into the bytecode. `LIVE` applies without a reload: the
        value lives in `Param::current_value` (no bytecode patch), and a
        consumer re-reads it when the param-change counter advances.
        """
        RELOAD = 0
        LIVE = 1

        _NAMES = {
            0: 'RELOAD',
            1: 'LIVE',
        }

    class BusKind:
        """
        Physical bus the driver runs on.

        Tags each per-bus profile in the trailing `bus_config` block, and
        is the value set of the runtime `bus` parameter (I2C vs SPI for
        register drivers, decided at upload time). The image carries one
        profile per bus the chip supports; firmware applies the one whose
        kind matches the selected `bus`. The `BUS_KIND_NONE` sentinel sits
        outside this enum because it is not a valid bus, only an "absent
        trailer" marker.
        """
        I2C = 0
        SPI = 1
        UART = 2

        _NAMES = {
            0: 'I2C',
            1: 'SPI',
            2: 'UART',
        }

    class AutoInc:
        """
        Register auto-increment scheme for burst access.

        `IMPLICIT` — the chip advances its register pointer on continued
        clocking (most parts); a burst just reads N bytes. `MSB` — the
        controller sets the address MSB to enable multi-byte access (ST
        LIS/LSM parts set the I²C sub-address bit 7). `NONE` — the part has
        no auto-increment; multi-byte register access is unsupported.
        """
        IMPLICIT = 0
        MSB = 1
        NONE = 2

        _NAMES = {
            0: 'IMPLICIT',
            1: 'MSB',
            2: 'NONE',
        }

    class Pec:
        """
        I²C packet error checking (SMBus PEC).

        `NONE` — no CRC byte. `CRC8` — an SMBus PEC byte (CRC-8, poly 0x07)
        trails each transaction; the controller appends it on write and
        verifies it on read.
        """
        NONE = 0
        CRC8 = 1

        _NAMES = {
            0: 'NONE',
            1: 'CRC8',
        }

    NXS_MAJOR = 1  # NXS format major version (incompatible-change counter).

    NXS_MINOR = 0  # NXS format minor version (highest backward-compatible minor firmware implements).

    VM_MAX_PROGRAM_SIZE = 4096  # Max bytecode bytes one driver program may carry.

    MAX_DRIVER_IMAGE_SIZE = 6144  # Max serialized `.nxs` image size (header + metadata + bytecode).

    BUS_KIND_NONE = 255  # `BusConfig::kind` value for a default-constructed (unset) profile. A parsed profile always carries a real bus kind; absence of profiles is `num_bus_profiles == 0`, not this value on the wire.

    MAX_BUS_PROFILES = 3  # Max per-bus communication profiles in one image (i2c + spi, or uart).

    MAX_PARAMS = 8  # Max configurable parameters one driver image can declare.

    MAX_PATCH_SITES = 2  # Max bytecode sites one parameter may patch on a runtime set.

    MAX_PARAM_VALUES = 16  # Max enum values (or the [min, max] pair) a single parameter declares.

    MAX_OUTPUTS = 16  # Max output fields one driver image can declare.

class FieldSemantics:

    class FieldSemantic:
        """
        Output-field semantic category codes.

        Carried per output field in the NXS image and exposed through both
        host interfaces (the Cyphal GetOutputInfo service and the I²C
        descriptor window). Lets a consumer categorize a field — "this is the
        accelerometer X axis" — without parsing its name. GENERIC (0) is the
        fallback for anything unrecognized. Append only — never renumber.

        Per-value routing metadata is the single source for the SI
        projection: `si_unit` is the canonical unit the compiler enforces
        (omitted = authored unit, no enforcement), `bucket`/`slot` place the
        field on its standard subject (omitted = NONE, RawSample-only), and
        `group: geodetic` marks the fields the mapper assembles into one
        completeness-gated PointStateVarTs instead of routing by bucket.
        """
        GENERIC = 0
        ACCEL_X = 1
        ACCEL_Y = 2
        ACCEL_Z = 3
        GYRO_X = 4
        GYRO_Y = 5
        GYRO_Z = 6
        MAG_X = 7
        MAG_Y = 8
        MAG_Z = 9
        TEMPERATURE = 10
        PRESSURE = 11
        HUMIDITY = 12
        NMEA = 13
        LATITUDE = 14
        LONGITUDE = 15
        ALTITUDE = 16
        VEL_NORTH = 17
        VEL_EAST = 18
        VEL_DOWN = 19
        POS_H_ACC = 20
        POS_V_ACC = 21
        VEL_S_ACC = 22
        ANGLE = 23
        VOLTAGE = 24
        CURRENT = 25
        DISTANCE = 26
        FORCE = 27
        FREQUENCY = 28
        LUMINANCE = 29
        MASS = 30
        TORQUE = 31
        SPEED = 32
        FLOW = 33
        TIME_OF_WEEK = 34
        FIX_TYPE = 35

        _NAMES = {
            0: 'GENERIC',
            1: 'ACCEL_X',
            2: 'ACCEL_Y',
            3: 'ACCEL_Z',
            4: 'GYRO_X',
            5: 'GYRO_Y',
            6: 'GYRO_Z',
            7: 'MAG_X',
            8: 'MAG_Y',
            9: 'MAG_Z',
            10: 'TEMPERATURE',
            11: 'PRESSURE',
            12: 'HUMIDITY',
            13: 'NMEA',
            14: 'LATITUDE',
            15: 'LONGITUDE',
            16: 'ALTITUDE',
            17: 'VEL_NORTH',
            18: 'VEL_EAST',
            19: 'VEL_DOWN',
            20: 'POS_H_ACC',
            21: 'POS_V_ACC',
            22: 'VEL_S_ACC',
            23: 'ANGLE',
            24: 'VOLTAGE',
            25: 'CURRENT',
            26: 'DISTANCE',
            27: 'FORCE',
            28: 'FREQUENCY',
            29: 'LUMINANCE',
            30: 'MASS',
            31: 'TORQUE',
            32: 'SPEED',
            33: 'FLOW',
            34: 'TIME_OF_WEEK',
            35: 'FIX_TYPE',
        }

    class SubjectBucket:
        """
        The standard subject a semantic output field maps onto.

        The vector and named-scalar spine quantities get a dedicated bucket
        each; the long tail of single-value SI quantities shares SCALAR,
        whose slot is the scalar-kind index (semantic − SCALAR_SEM_FIRST).
        Indexes the device-wide per-subject decimation array (CommThread,
        ConfigStore) — append only, never renumber.
        """
        NONE = 0  # No standard subject: GENERIC, non-SI, and the geodetic group
        ACCELERATION = 1
        ANGULAR_VELOCITY = 2
        MAGNETIC_FIELD = 3
        TEMPERATURE = 4
        PRESSURE = 5
        SCALAR = 6

        _NAMES = {
            0: 'NONE',
            1: 'ACCELERATION',
            2: 'ANGULAR_VELOCITY',
            3: 'MAGNETIC_FIELD',
            4: 'TEMPERATURE',
            5: 'PRESSURE',
            6: 'SCALAR',
        }

    NUM_FIELD_SEMANTICS = 36
    NUM_SUBJECT_BUCKETS = 7
    SCALAR_SEM_FIRST = 23
    SCALAR_SEM_LAST = 33
    NUM_SCALAR_KINDS = 11

    # Canonical SI unit per semantic code; a semantic absent here carries an
    # authored unit (no enforcement).
    FIELD_SEMANTIC_UNIT = {
        1: 'm/s^2',  # ACCEL_X
        2: 'm/s^2',  # ACCEL_Y
        3: 'm/s^2',  # ACCEL_Z
        4: 'rad/s',  # GYRO_X
        5: 'rad/s',  # GYRO_Y
        6: 'rad/s',  # GYRO_Z
        7: 'tesla',  # MAG_X
        8: 'tesla',  # MAG_Y
        9: 'tesla',  # MAG_Z
        10: 'kelvin',  # TEMPERATURE
        11: 'pascal',  # PRESSURE
        14: 'rad',  # LATITUDE
        15: 'rad',  # LONGITUDE
        16: 'm',  # ALTITUDE
        17: 'm/s',  # VEL_NORTH
        18: 'm/s',  # VEL_EAST
        19: 'm/s',  # VEL_DOWN
        20: 'm',  # POS_H_ACC
        21: 'm',  # POS_V_ACC
        22: 'm/s',  # VEL_S_ACC
        23: 'rad',  # ANGLE
        24: 'V',  # VOLTAGE
        25: 'A',  # CURRENT
        26: 'm',  # DISTANCE
        27: 'N',  # FORCE
        28: 'Hz',  # FREQUENCY
        29: 'cd/m^2',  # LUMINANCE
        30: 'kg',  # MASS
        31: 'N*m',  # TORQUE
        32: 'm/s',  # SPEED
        33: 'm^3/s',  # FLOW
        34: 's',  # TIME_OF_WEEK
    }

    # SubjectBucket value per semantic code.
    FIELD_SEMANTIC_BUCKET = {
        0: 0,  # GENERIC -> NONE
        1: 1,  # ACCEL_X -> ACCELERATION
        2: 1,  # ACCEL_Y -> ACCELERATION
        3: 1,  # ACCEL_Z -> ACCELERATION
        4: 2,  # GYRO_X -> ANGULAR_VELOCITY
        5: 2,  # GYRO_Y -> ANGULAR_VELOCITY
        6: 2,  # GYRO_Z -> ANGULAR_VELOCITY
        7: 3,  # MAG_X -> MAGNETIC_FIELD
        8: 3,  # MAG_Y -> MAGNETIC_FIELD
        9: 3,  # MAG_Z -> MAGNETIC_FIELD
        10: 4,  # TEMPERATURE -> TEMPERATURE
        11: 5,  # PRESSURE -> PRESSURE
        12: 0,  # HUMIDITY -> NONE
        13: 0,  # NMEA -> NONE
        14: 0,  # LATITUDE -> NONE
        15: 0,  # LONGITUDE -> NONE
        16: 0,  # ALTITUDE -> NONE
        17: 0,  # VEL_NORTH -> NONE
        18: 0,  # VEL_EAST -> NONE
        19: 0,  # VEL_DOWN -> NONE
        20: 0,  # POS_H_ACC -> NONE
        21: 0,  # POS_V_ACC -> NONE
        22: 0,  # VEL_S_ACC -> NONE
        23: 6,  # ANGLE -> SCALAR
        24: 6,  # VOLTAGE -> SCALAR
        25: 6,  # CURRENT -> SCALAR
        26: 6,  # DISTANCE -> SCALAR
        27: 6,  # FORCE -> SCALAR
        28: 6,  # FREQUENCY -> SCALAR
        29: 6,  # LUMINANCE -> SCALAR
        30: 6,  # MASS -> SCALAR
        31: 6,  # TORQUE -> SCALAR
        32: 6,  # SPEED -> SCALAR
        33: 6,  # FLOW -> SCALAR
        34: 0,  # TIME_OF_WEEK -> NONE
        35: 0,  # FIX_TYPE -> NONE
    }

    # Slot within the routed subject (vector axis / scalar-kind index).
    FIELD_SEMANTIC_SLOT = {
        0: 0,  # GENERIC
        1: 0,  # ACCEL_X
        2: 1,  # ACCEL_Y
        3: 2,  # ACCEL_Z
        4: 0,  # GYRO_X
        5: 1,  # GYRO_Y
        6: 2,  # GYRO_Z
        7: 0,  # MAG_X
        8: 1,  # MAG_Y
        9: 2,  # MAG_Z
        10: 0,  # TEMPERATURE
        11: 0,  # PRESSURE
        12: 0,  # HUMIDITY
        13: 0,  # NMEA
        14: 0,  # LATITUDE
        15: 0,  # LONGITUDE
        16: 0,  # ALTITUDE
        17: 0,  # VEL_NORTH
        18: 0,  # VEL_EAST
        19: 0,  # VEL_DOWN
        20: 0,  # POS_H_ACC
        21: 0,  # POS_V_ACC
        22: 0,  # VEL_S_ACC
        23: 0,  # ANGLE
        24: 1,  # VOLTAGE
        25: 2,  # CURRENT
        26: 3,  # DISTANCE
        27: 4,  # FORCE
        28: 5,  # FREQUENCY
        29: 6,  # LUMINANCE
        30: 7,  # MASS
        31: 8,  # TORQUE
        32: 9,  # SPEED
        33: 10,  # FLOW
        34: 0,  # TIME_OF_WEEK
        35: 0,  # FIX_TYPE
    }

    # DSDL type and wire shape per SubjectBucket value.
    SUBJECT_BUCKET_TYPE = {
        1: 'uavcan.si.sample.acceleration.Vector3.1.0',  # ACCELERATION
        2: 'uavcan.si.sample.angular_velocity.Vector3.1.0',  # ANGULAR_VELOCITY
        3: 'uavcan.si.sample.magnetic_field_strength.Vector3.1.0',  # MAGNETIC_FIELD
        4: 'uavcan.si.sample.temperature.Scalar.1.0',  # TEMPERATURE
        5: 'uavcan.si.sample.pressure.Scalar.1.0',  # PRESSURE
        6: 'uavcan.si.sample.<quantity>.Scalar.1.0',  # SCALAR
    }
    SUBJECT_BUCKET_WIRE_SHAPE = {
        1: 'vector3',  # ACCELERATION
        2: 'vector3',  # ANGULAR_VELOCITY
        3: 'vector3',  # MAGNETIC_FIELD
        4: 'scalar',  # TEMPERATURE
        5: 'scalar',  # PRESSURE
        6: 'scalar_block',  # SCALAR
    }

class GpsTime:

    GPS_EPOCH_UNIX_S = 315964800  # Unix seconds of the GPS epoch (1980-01-06 00:00:00 UTC).

    GPS_UTC_LEAP_S = 18  # GPS−UTC leap-second offset; bump on the next announced leap second.

    GPS_WEEK_S = 604800  # Seconds per GPS week (iTOW rolls over at this bound).

class RunnerStates:

    class RunnerState:
        """
        Driver lifecycle state.

        Sits above `VmState` (IDLE/RUNNING/ERROR). Happy path:
        NO_DRIVER → PROBING → MEASURING.
        """
        NO_DRIVER = 0  # No driver loaded into the VM.
        LOADING = 1  # Driver loaded, VM not running (post-STOP).
        PROBING = 2  # VM running probe + configure; no samples yet.
        MEASURING = 3  # VM in measure loop; sample_count > 0.
        PROBE_FAILED = 5  # N consecutive probe failures; advance slot.

        _NAMES = {
            0: 'NO_DRIVER',
            1: 'LOADING',
            2: 'PROBING',
            3: 'MEASURING',
            5: 'PROBE_FAILED',
        }
