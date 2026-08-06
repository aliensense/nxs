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
        PROGRAM_SIZE_LO = 17
        PROGRAM_SIZE_HI = 18
        PARAM_SELECT = 19  # Chooses which param descriptor is exposed.
        PARAM_SET_VALUE = 20  # 4 bytes (u32 LE).
        STORE_SELECT = 24  # Slot for SAVE / DELETE.
        PROTO_VERSION = 25  # Read-only. Version of the register-map contract (currently `PROTO_VERSION_VALUE`).
        XFER_TYPE = 26  # Selects the consumer of `PROGRAM_DATA` writes: 0 = VM bytecode (default; NXS upload), 1 = DFU image bytes (forwarded to the DFU sink), 2 = config record (identity commissioning; `PROGRAM_DATA` carries a `CONFIG_RECORD_SIZE`-byte record and `Cmd::STORE_PERSIST` commits it), 3 = time-sync record (a `TIME_SYNC_RECORD_SIZE`-byte volatile record applied as its 20th byte lands; never persisted). The `_regs[PROGRAM_DATA..]` window mirrors the current record for read-back in modes 2 and 3.
        XFER_PHASE = 27  # Read-only `DfuPhase`. The master polls this to pace `DFU_BEGIN`'s erase and to detect a rejected write (detail in `CMD_ERROR`). IDLE when XFER_TYPE=0.
        XFER_ACK = 28  # Read-only accepted-chunk counter, mod 256: 0 after `DFU_BEGIN`, +1 per committed `PROGRAM_DATA` chunk. Single-byte so an I²C poll can never read it torn; the master paces its push on it.
        CMD_ERROR = 29  # Read-only result of the most recent host command op — a DFU operation or a store SAVE / DELETE_SLOT / CLEAR_STORE: 0 = OK, otherwise the positive errno value. Owned by the command-dispatch path alone; the `vm_status` heartbeat never writes it, so a result can't be clobbered (and `ERROR_CODE` stays with VM health). A store command is dispatched asynchronously, so this reads `CMD_ERR_PENDING` from enqueue until the deferred handler runs — a host polls until it changes. Cleared by `DFU_BEGIN`, a successful DFU write, or `XFER_TYPE = 0`.
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
        SEL_VALUE_INDEX = 228  # Param view, writable: which declared value `SEL_VALUE` exposes (0..num_values-1). Resets to 0 on `PARAM_SELECT`.
        SEL_OUTPUT_SEMANTIC = 228  # Output view: semantic category code (0 = generic).
        SEL_VALUE = 229  # Param view: 4 bytes (u32 LE) — values[SEL_VALUE_INDEX].
        SEL_OUTPUT_COUNT = 229  # Output view: 2 bytes (u16 LE) — string payload width, 0 for numeric fields.
        SEL_OUTPUT_AT = 231  # Output view: the field's byte position within the sample (gaps are legal).
        DRIVER_SELECT = 233  # Writable. Any write switches the SEL window to the driver view (name via SEL_NAME_LEN/SEL_NAME); echoes 1 in the live view and `DRIVER_VIEW_PEEK` after Cmd::PEEK_SLOT.
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
        LOAD = 0
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
        STORE_PERSIST = 11  # Validate the staged identity config (the `CONFIG_RECORD_SIZE`-byte record written to `PROGRAM_DATA` under `XFER_TYPE = CONFIG`) and commit it to NVS. Async like SAVE: arms `CMD_ERR_PENDING`, then `CMD_ERROR` reads 0 (committed) or a positive errno (e.g. EINVAL for an out-of-range address). Identity changes take effect at the next reboot.
        IDENTIFY = 12  # Strobe the status LED (~10 s) so an operator can physically locate the unit.
        PEEK_SLOT = 13  # Peek store[STORE_SELECT] without loading it (STORE_SELECT = 0xFF peeks the active driver). Async like SAVE: arms `CMD_ERR_PENDING`, then `CMD_ERROR` reads 0 (peek view valid) or +errno — ENOENT empty/out-of-range slot, EBADF corrupt image, ENOTSUP foreign image version. On success the SEL window switches to the peek view: name via `SEL_NAME_LEN`/`SEL_NAME`, counts via `SEL_DRIVER_NUM_PARAMS`/`SEL_DRIVER_NUM_OUTPUTS`, `SEL_DRIVER_SLOT` echoes the peeked slot, `SEL_DRIVER_I2C_ADDR` the latched mikroBUS address (0 unless peeking the active driver), and `DRIVER_SELECT` echoes `DRIVER_VIEW_PEEK`. The view is a snapshot — re-issue after store mutations.
        ENTER_RECOVERY = 14  # Arm the MCUboot serial-recovery flag and cold-reset into it. The device then holds in the bootloader until a host completes an mcumgr image upload — there is no timed window to catch. Dispatched independently of the VM handler, like `REBOOT`, so it works on a device whose driver never loaded. The Cyphal equivalent is the `ENTER_RECOVERY` (0xA008) ExecuteCommand.

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

    STATUS_SAMPLE_READY = 1  # `Reg::STATUS` bit: a fresh sample has landed in `Reg::SAMPLE_DATA`.

    STATUS_ERROR = 2  # `Reg::STATUS` bit: the VM is in an error state (`Reg::ERROR_CODE` carries the detail).

    STATUS_RUNNING = 128  # `Reg::STATUS` bit: the VM is currently in the measuring loop.

    WHO_AM_I_VALUE = 171  # Expected value at `Reg::WHO_AM_I`; lets host tooling probe a bus for an NXS target.

    SERIAL_LEN = 12  # Bytes in the `Reg::SERIAL` window (STM32 UID96 = 96 bits).

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

    CONFIG_RECORD_SIZE = 28  # Bytes in the identity-config record exchanged through the `PROGRAM_DATA` window under `XFER_TYPE = CONFIG`.

    CMD_DOORBELL_BIT = 128  # CMD doorbell bit — host ORs this into the opcode so firmware distinguishes `Cmd::LOAD` (0) from "no command pending".

    CMD_OPCODE_MASK = 127  # Mask applied to `Reg::CMD` before dispatch, stripping `CMD_DOORBELL_BIT`.

    CMD_ERR_PENDING = 255  # `Reg::CMD_ERROR` value while an asynchronous store command is in flight.

    SELECTOR_INACTIVE = 255  # Echo value for the inactive selector (`PARAM_SELECT` / `OUTPUT_SELECT`); 0xFF is unambiguous since no driver declares 0xFF params/outputs.

    DRIVER_VIEW_PEEK = 2  # `DRIVER_SELECT` echo while the SEL window shows a peeked stored slot (the live driver view echoes 1).

    SEL_TYPE_PARAM_TYPE_MASK = 15  # Param view: mask isolating `param_type` (0 = enum, 1 = range) in the low nibble of `Reg::SEL_TYPE`.

    SEL_TYPE_KIND_SHIFT = 4  # Param view: bit position of `ParamKind` (0 = reload, 1 = live) within `Reg::SEL_TYPE`; the descriptor window is full, so kind rides a spare bit of `SEL_TYPE` rather than its own register.

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

    NUM_FIELD_SEMANTICS = 34
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

class CmdAck:

    class Ack:
        """
        Result codes carried in the framed command acknowledgement.

        Negative on failure, 0 on success. The framed command path
        answers every request with an acknowledgement carrying one of
        these. The I²C register path reports the equivalent state in
        `Reg::STATUS` and `Reg::ERROR_CODE` instead.
        """
        OK = 0
        ERR_MALFORMED = -1  # Frame too short or a field failed validation.
        ERR_NO_HANDLER = -2  # No backend handler is attached for this command.
        ERR_UPLOAD_STATE = -3  # Upload command arrived out of order (no open session).
        ERR_UPLOAD_SIZE = -4  # Chunk overruns the declared total size.
        ERR_UPLOAD_CRC = -5  # Committed image failed its CRC check.
        ERR_INVALID = -6  # Handler rejected an otherwise well-formed request.

        _NAMES = {
            0: 'OK',
            -1: 'ERR_MALFORMED',
            -2: 'ERR_NO_HANDLER',
            -3: 'ERR_UPLOAD_STATE',
            -4: 'ERR_UPLOAD_SIZE',
            -5: 'ERR_UPLOAD_CRC',
            -6: 'ERR_INVALID',
        }
