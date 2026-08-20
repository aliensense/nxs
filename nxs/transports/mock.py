"""In-memory NxsClient returning synthetic samples for tests and board-free runs."""
import logging
import struct
import time
from dataclasses import replace

from nxs.client import (
    CalibrationRecord, NxsClient, SupportsBitTiming, SupportsCalibration,
    SupportsCanTermination, SupportsCommissioning, SupportsIdentify,
    SupportsRecovery, SupportsSlotPeek, SupportsTimeSync,
    validate_can_bitrate, validate_can_term, validate_commission)

log = logging.getLogger("nxs.mock")


class MockTransport(NxsClient, SupportsCommissioning, SupportsIdentify,
                    SupportsRecovery, SupportsSlotPeek, SupportsBitTiming,
                    SupportsCanTermination, SupportsTimeSync,
                    SupportsCalibration):
    """In-memory `NxsClient` returning synthetic IMU samples — for
    tests and board-free runs. Simulates a 14-byte sample
    (7 × int16 BE: accel xyz, temp, gyro xyz) and carries the full
    management surface (identity, store peek, decimation, commissioning,
    recovery) so board-free flows exercise the same code paths."""

    SERIAL = bytes(range(0xB0, 0xB0 + 12))
    FW_VERSION = "v1.0.0-2-gabcdef1"

    def __init__(self):
        super().__init__()
        self._loaded = None
        self._loaded_name = ""
        self._run = False
        self._tick = 0
        self._sample_size = 20
        self._slots: dict = {}
        self._active_slot = 0xFF
        self._identify_calls = 0
        self._recover_calls = 0
        self._decimation: dict = {None: 1}
        self._identity = {"node_addr": 125, "topics": {}}
        self._can_bitrate = (1000000, 4000000)
        self._can_term = 0
        self._ts_offset = 0
        self._ts_bound = 0
        self._ts_rate = 0
        self._ts_valid_for = 0
        self._ts_source = 0
        self._device_epoch = 0
        self._calibration = CalibrationRecord()
        self._calibration_persisted = False
        # Injectable raw-count offsets so a calibrate run has a real bias
        # to solve for (added to the synthetic counts before packing).
        self._gyro_bias_counts = (0, 0, 0)
        # Simulated on-device procedure surface, driven per progress poll.
        self._cal_script = None
        self._cal_progress = (0, 0, 0xFF)
        self._cal_stop_result = 0
        self._cal_epoch = 1
        self._cal_script_applies = None
        self.cal_aborted = False

    def probe(self) -> bool:
        return True

    def identify(self):
        self._identify_calls += 1

    def read_serial(self):
        return self.SERIAL

    def read_fw_version(self):
        return self.FW_VERSION

    def recover(self) -> int:
        self._recover_calls += 1
        return 0

    def read_decimation(self, subject=None) -> int:
        return self._decimation.get(subject, 1)

    def write_decimation(self, value: int, subject=None) -> None:
        self._decimation[subject] = value

    def read_identity(self) -> dict:
        return {"node_addr": self._identity["node_addr"],
                "topics": dict(self._identity["topics"])}

    def commission(self, node_addr=None, topics=None, can_bitrate=None,
                   can_term=None) -> None:
        validate_commission(node_addr, topics)
        if can_bitrate is not None:
            self.write_can_bitrate(*can_bitrate)
        if can_term is not None:
            self.write_can_term(can_term)
        if node_addr is not None:
            self._identity["node_addr"] = node_addr
        if topics:
            self._identity["topics"].update(topics)

    def read_slot_info(self, slot: int):
        """SupportsSlotPeek over the in-memory store."""
        from types import SimpleNamespace

        from nxs.image import deserialize

        image = (self._loaded if slot == 0xFF and self._active_slot == 0xFF
                 else self._slots.get(self._active_slot if slot == 0xFF else slot))
        if image is None:
            return None
        try:
            compiled = deserialize(image)
        except Exception:
            return None
        return SimpleNamespace(name=compiled.name,
                               num_params=len(compiled.params),
                               num_outputs=len(compiled.outputs),
                               i2c_addr=0)

    @property
    def identify_calls(self) -> int:
        return self._identify_calls

    # ── CAN bit timing ────────────────────────────────────
    def read_can_bitrate(self) -> tuple:
        return self._can_bitrate

    def write_can_bitrate(self, nominal: int, data: int) -> None:
        validate_can_bitrate(nominal, data)
        self._can_bitrate = ((1000000, 4000000) if nominal == 0
                             else (nominal, data))

    # ── CAN termination ───────────────────────────────────
    def read_can_term(self) -> int:
        return self._can_term

    def write_can_term(self, value: int) -> None:
        validate_can_term(value)
        self._can_term = 0 if value == 0xFFFF else value

    # ── Calibration ───────────────────────────────────────
    GYRO_SCALE = 1e-3   # the served descriptor's gyro LSB, rad/s

    def cal_gyro(self) -> None:
        # Simulated on-device procedure: two averaging polls, then done.
        self._cal_script = [(2, 40, 0xFF), (2, 100, 0xFF), (0, 0, 0)]
        self._cal_script_applies = 'gyro'

    def cal_mag_start(self) -> None:
        # Coverage climbs over a few polls, then holds past the CLI's
        # stopping point until stop.
        from nxs._generated_constants import Calibration
        stop_at = Calibration.MAG_COVERAGE_ENOUGH + 1
        self._cal_script = [(3, 4, 0xFF), (3, 9, 0xFF), (3, stop_at, 0xFF)]

    def cal_mag_stop(self) -> None:
        if self._cal_stop_result:
            # A coverage refusal keeps the collection open and shows no
            # verdict, exactly like the firmware — the progress tuple
            # stays wherever the script left it.
            from nxs.client import CALIB_ERR_REASON, DeviceRefused, err_reason
            raise DeviceRefused(
                    self._cal_stop_result,
                    err_reason(self._cal_stop_result, CALIB_ERR_REASON))
        self._cal_script = None
        self._cal_progress = (0, 0, 0)
        # The real stop applies the solved affine atomically; report the
        # same observable change so a mock-run test cannot pass while
        # seeing no calibration move.
        rec = self.read_calibration()
        rec.m = (rec.m[0], rec.m[1], (0.9, 0.0, 0.0, 0.0, 0.9, 0.0,
                                      0.0, 0.0, 0.9))
        rec.b = (rec.b[0], rec.b[1], (1e-6, -2e-6, 3e-6))
        self.write_calibration(rec, persist=False)

    def cal_abort(self) -> None:
        from nxs.client import ERRNO_ECANCELED
        self._cal_script = None
        self._cal_progress = (0, 0, ERRNO_ECANCELED)
        self.cal_aborted = True

    def read_cal_progress(self):
        # A drained script holds its last tuple (mag: coverage until stop).
        if self._cal_script:
            self._cal_progress = self._cal_script.pop(0)
            if (not self._cal_script and self._cal_progress == (0, 0, 0)
                    and self._cal_script_applies == 'gyro'):
                # The completing pass applies the solved bias, exactly
                # like the device: still rates decode to ~0 afterwards.
                rec = self.read_calibration()
                rec.b = (rec.b[0],
                         tuple(-c * self.GYRO_SCALE
                               for c in self._gyro_bias_counts),
                         rec.b[2])
                self.write_calibration(rec, persist=False)
                self._cal_script_applies = None
        return self._cal_progress

    def set_cal_stop_result(self, errno_value: int) -> None:
        """Force the next cal_mag_stop to refuse with `errno_value`."""
        self._cal_stop_result = errno_value

    def save_calibration(self) -> None:
        self._calibration_persisted = True

    def read_calibration(self) -> CalibrationRecord:
        # Defensive copy, mirroring a real transport's pack/unpack value
        # semantics: mutating a read record must not change device state
        # until write_calibration.
        return replace(self._calibration)

    def write_calibration(self, record: CalibrationRecord,
                          persist: bool = True) -> None:
        self._calibration = replace(record)
        self._calibration_persisted = persist
        self._cal_epoch = (self._cal_epoch + 1) & 0xFF

    def read_cal_epoch(self) -> int:
        return self._cal_epoch

    def set_orientation(self, rotation: int, persist: bool = True) -> None:
        record = self.read_calibration()
        record.orientation = rotation
        self.write_calibration(record, persist)

    @property
    def calibration_persisted(self) -> bool:
        return self._calibration_persisted

    def set_gyro_bias_counts(self, bias) -> None:
        """Inject a constant raw-count gyro offset into the synthetic stream."""
        self._gyro_bias_counts = tuple(bias)

    # ── Driver lifecycle ──────────────────────────────────
    def upload_image(self, image: bytes):
        self._loaded = image
        self._tick = 0
        self._run = False
        # A real device serves the stored driver's name; parse it so
        # name-based checks behave the same against the mock.
        try:
            from nxs.image import deserialize
            self._loaded_name = deserialize(image).name
        except Exception:
            self._loaded_name = "mock"
        # Locate the SET_SAMPLE_SIZE opcode in the embedded bytecode —
        # unique enough to find without decoding the NXS header.
        self._sample_size = 20
        for i in range(len(image) - 1):
            if image[i] == 0x61:  # OP_SET_SAMPLE_SIZE
                self._sample_size = image[i + 1]
                break
        log.info("Mock: uploaded %d bytes", len(image))

    def vm_run(self):
        self._run = True

    def vm_stop(self):
        self._run = False

    def vm_reset(self):
        self._run = False
        self._loaded = None

    def push_image(self, bin_path, chunk_size: int = 32, progress_cb=None) -> int:
        """Simulated DFU: reports the image size, flashes nothing."""
        import os
        return os.path.getsize(bin_path)

    # ── Parameters ────────────────────────────────────────
    def read_capabilities(self) -> list:
        return []

    def set_param(self, name: str, value: int):
        pass

    # ── Driver store ──────────────────────────────────────
    # `_slots` maps a stable slot NUMBER → image, like real firmware —
    # not a list, so a non-contiguous `save_slot(3)` keeps slot 3.
    def save_slot(self, slot: int):
        if self._loaded is None:
            log.warning("Mock save: no image uploaded yet")
            return
        self._slots[slot] = self._loaded
        # First populated slot becomes active; later saves don't steal
        # the active pointer (the runner owns it on real firmware).
        if self._active_slot == 0xFF:
            self._active_slot = slot

    def delete_slot(self, slot: int):
        self._slots.pop(slot, None)
        if self._active_slot == slot:
            self._active_slot = min(self._slots) if self._slots else 0xFF

    def clear_store(self):
        self._slots.clear()
        self._active_slot = 0xFF

    def cycle(self):
        # Advance to the next populated slot number, wrapping.
        occupied = sorted(self._slots)
        if occupied:
            nxt = next((s for s in occupied if s > self._active_slot),
                       occupied[0])
            self._active_slot = nxt

    # ── Status / metadata ─────────────────────────────────
    def read_driver_name(self) -> str:
        # Model the real transports: report the ACTIVE stored slot's driver,
        # falling back to the RAM-loaded driver when nothing is saved yet
        # (the transient 0xFF active slot).
        if self._active_slot in self._slots:
            from nxs.image import deserialize
            return deserialize(self._slots[self._active_slot]).name
        return self._loaded_name if self._loaded else ""

    def read_sample_size(self) -> int:
        return self._sample_size

    def read_sample_count(self) -> int:
        return self._tick

    def read_status(self) -> int:
        return 0x81 if self._run else 0x00  # bit7 running, bit0 sample_ready

    def read_vm_state(self) -> int:
        return 1 if self._run else 0

    def read_error_code(self) -> int:
        return 0

    def read_store_count(self) -> int:
        return len(self._slots)

    def read_active_slot(self) -> int:
        return self._active_slot

    def read_runner_state(self) -> int:
        return 3 if self._run else 0  # 3 = MEASURING

    def read_probe_retries(self) -> int:
        return 0

    # ── Output descriptors ────────────────────────────────
    def read_outputs(self) -> list:
        # One entry per field of `_fake_sample()`'s packed layout
        # (`>3hh3h3h`, big-endian int16): decoding these descriptors over
        # the streamed bytes yields SI values — az 4096 counts ~ 9.81
        # m/s^2, temp 25 degC, |B| ~ 46 uT.
        def f(name, sem, off, scale, offset=0.0):
            return {'name': name, 'semantic': sem, 'type': 'int16',
                    'byte_order': 'big', 'byte_off': off,
                    'scale': scale, 'offset': offset}

        accel = 9.80665 / 4096          # counts at FS=8g to m/s^2
        return [
            f('accel_x', 1, 0, accel), f('accel_y', 2, 2, accel),
            f('accel_z', 3, 4, accel),
            f('temp', 10, 6, 1 / 326.8, 25.0),
            f('gyro_x', 4, 8, 1e-3), f('gyro_y', 5, 10, 1e-3),
            f('gyro_z', 6, 12, 1e-3),
            f('mag_x', 7, 14, 1e-7), f('mag_y', 8, 16, 1e-7),
            f('mag_z', 9, 18, 1e-7),
        ]

    def _descriptor_token(self) -> int:
        return 1 if self._loaded else 0

    # ── Streaming primitives ──────────────────────────────
    def _arm_stream(self, every_nth: int):
        pass

    def _disarm_stream(self):
        pass

    def _next_raw(self, timeout: float):
        if not self._run:
            if timeout:
                time.sleep(timeout)
            return None
        if timeout:
            time.sleep(min(timeout, 0.005))  # pace blocking streams
        self._tick += 1
        return self._tick, self._fake_sample(), None

    def _fake_sample(self) -> bytes:
        import random
        ax = random.randint(-20, 20)
        ay = random.randint(-20, 20)
        az = 4096 + random.randint(-10, 10)  # ~1g at FS=8g
        temp = 0  # 0 * (1/326.8) + 25 = 25°C
        gx = random.randint(-5, 5) + self._gyro_bias_counts[0]
        gy = random.randint(-5, 5) + self._gyro_bias_counts[1]
        gz = random.randint(-5, 5) + self._gyro_bias_counts[2]
        mx = 200 + random.randint(-2, 2)   # x 0.1 uT/LSB: |B| ~ 46 uT
        my = -100 + random.randint(-2, 2)
        mz = 400 + random.randint(-2, 2)
        sample = struct.pack('>3hh3h3h', ax, ay, az, temp, gx, gy, gz,
                             mx, my, mz)
        return sample[:self._sample_size]

    # ── Time sync fake ────────────────────────────────────
    def read_device_time_us(self):
        self._device_epoch += 1000
        return self._device_epoch

    def push_time_sync(self, offset_us: int, bound_us: int,
                       rate_ppb: int = 0,
                       valid_for_us: int = 0) -> None:
        if valid_for_us == 0:
            return
        self._ts_offset = offset_us
        self._ts_bound = bound_us
        self._ts_rate = rate_ppb
        self._ts_valid_for = valid_for_us
        self._ts_source = 1

    def read_time_sync(self) -> tuple:
        return (self._ts_offset, self._ts_bound, self._ts_rate,
                self._ts_valid_for, self._ts_source, self._ts_source != 0)
