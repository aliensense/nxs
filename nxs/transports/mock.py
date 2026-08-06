"""In-memory NxsClient returning synthetic samples for tests and board-free runs."""
import logging
import struct
import time

from nxs.client import (
    NxsClient, SupportsBitTiming, SupportsCanTermination,
    SupportsCommissioning, SupportsIdentify, SupportsRecovery,
    SupportsSlotPeek, SupportsTimeSync, validate_can_bitrate, validate_can_term,
    validate_commission)

log = logging.getLogger("nxs.mock")

class MockTransport(NxsClient, SupportsCommissioning, SupportsIdentify,
                    SupportsRecovery, SupportsSlotPeek, SupportsBitTiming,
                    SupportsCanTermination, SupportsTimeSync):
    """In-memory `NxsClient` returning synthetic IMU samples — for
    tests and board-free runs. Simulates a 14-byte sample
    (7 × int16 BE: accel xyz, temp, gyro xyz) and carries the full
    management surface (identity, store peek, decimation, commissioning,
    recovery) so board-free flows exercise the same code paths."""

    SERIAL = bytes(range(0xB0, 0xB0 + 12))
    FW_VERSION = "1.0"

    def __init__(self):
        super().__init__()
        self._loaded = None
        self._loaded_name = ""
        self._run = False
        self._tick = 0
        self._sample_size = 14
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
        self._sample_size = 14
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
        return []

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
        gx = random.randint(-5, 5)
        gy = random.randint(-5, 5)
        gz = random.randint(-5, 5)
        sample = struct.pack('>3hh3h', ax, ay, az, temp, gx, gy, gz)
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
