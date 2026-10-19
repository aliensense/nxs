# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The camera node of one link, `python -m nxs.ros2_camera_node <port>
<link>`: a frame session on the link's capture node through the library,
each frame published as `sensor_msgs/Image` (`mono8`, the luma plane) and
`sensor_msgs/CameraInfo`, stamped from the sensor's start of frame and the
link's recorded timing. nxsd runs one per link; the lines it prints go to
the journal."""

from __future__ import annotations

import argparse
import array
import dataclasses
import signal
import sys
import time
from typing import Callable, List, Optional, Sequence

from nxs import ros2_camera_info
from nxs._libnxs_frames import TIMING_EXPOSURE, TIMING_READOUT_DELAY

#: The line a camera node prints once its session runs (`nxsd` restarts
#: the port's output under the port's first one when it reads it).
STARTED = "Started stream"
#: How long one wait for a frame lasts, and how many waits in a row end the node: frames come at the
#: link's rate, so a stream silent this long has ended, and nxsd restarts the node.
FRAME_WAIT_NS = 2_000_000_000
SILENT_WAITS = 3
#: Seconds between the lines that name the stamp's lag and the rate.
REPORT_S = 60.0
#: The frame of the camera topics, under the link's name.
OPTICAL_FRAME = "{port}_{link}_optical"


@dataclasses.dataclass(frozen=True)
class StampLaw:
    """Where a frame's stamp sits, from the link's recorded timing.

    `readout_delay_ns` is taken off the start-of-frame stamp on a triggered
    link whose personality states it, `exposure_ns` is halved off that on a
    link whose record states the exposure; `stack_exposure` halves the
    capture stack's own per-frame exposure off instead, on a free-running
    link. `note` is the line the node prints once."""

    readout_delay_ns: int = 0
    exposure_ns: int = 0
    stack_exposure: bool = False
    note: str = ""


def stamp_law(timing) -> StampLaw:
    """The stamp law of a link's `CaptureTiming`."""
    if not timing.triggered:
        return StampLaw(stack_exposure=True,
                        note="free-running: frames are stamped at mid-exposure from the capture "
                             "stack's exposure")
    if not timing.stated & TIMING_READOUT_DELAY:
        return StampLaw(note="the personality states no readout delay: frames are stamped at "
                             "the start of frame")
    if not timing.stated & TIMING_EXPOSURE:
        return StampLaw(int(timing.readout_delay_ns),
                        note="the hub's datasheet states no pulse width: frames are stamped at "
                             "the exposure's end, up to half the exposure after its middle")
    return StampLaw(int(timing.readout_delay_ns), int(timing.exposure_ns))


def frame_stamp_ns(law: StampLaw, timestamp_ns: int, stack_exposure_ns: int) -> int:
    """A frame's stamp on the monotonic clock: its start-of-frame stamp
    moved back by the law."""
    exposure = stack_exposure_ns if law.stack_exposure else law.exposure_ns
    return int(timestamp_ns) - law.readout_delay_ns - exposure // 2


def realtime_ns(mono_ns: int, now_realtime_ns: int, now_monotonic_ns: int) -> int:
    """A monotonic stamp on the realtime clock, by the clocks' difference
    sampled as the frame came to hand."""
    return mono_ns + (now_realtime_ns - now_monotonic_ns)


def fill_header(header, stamp_realtime_ns: int, frame_id: str) -> None:
    header.stamp.sec = stamp_realtime_ns // 1_000_000_000
    header.stamp.nanosec = stamp_realtime_ns % 1_000_000_000
    header.frame_id = frame_id


def fill_image(msg, frame, luma, stamp_realtime_ns: int, frame_id: str) -> None:
    """A frame's luma plane as a `mono8` image: `step` is the buffer's pitch,
    `data` the `pitch × height` bytes copied out of it, an `array.array('B')`
    the generated message keeps as it is. Any other sequence the message
    checks and copies value by value, a few frames a second at 2112x1552."""
    fill_header(msg.header, stamp_realtime_ns, frame_id)
    msg.height = int(frame.height)
    msg.width = int(frame.width)
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = int(frame.pitch)
    msg.data = luma


def fill_info(msg, calibration: ros2_camera_info.Calibration, stamp_realtime_ns: int,
              frame_id: str) -> None:
    fill_header(msg.header, stamp_realtime_ns, frame_id)
    msg.height = calibration.height
    msg.width = calibration.width
    msg.distortion_model = calibration.distortion_model
    msg.d = list(calibration.d)
    msg.k = list(calibration.k)
    msg.r = list(calibration.r)
    msg.p = list(calibration.p)


class _Clock:
    """The node's clocks, one object a test replaces."""

    @staticmethod
    def monotonic_ns() -> int:
        return time.monotonic_ns()

    @staticmethod
    def realtime_ns() -> int:
        return time.time_ns()


def run(frames, law: StampLaw, publish: Callable[[int, object, object], None], log: Callable[[str], None],
        stop: Callable[[], bool], clock=_Clock(), copy=None) -> int:
    """The frame loop: each frame stamped, its luma copied out and
    published; the lag and the rate named every `REPORT_S`.

    `publish(stamp_realtime_ns, frame, luma)` sends one frame; `copy(frame,
    luma)` fills the buffer (the session's own without one). Returns 1 once
    the stream ended or stayed silent for `SILENT_WAITS` waits, 0 when
    `stop()` asked for the end."""
    copy = frames.copy if copy is None else copy
    luma = None
    silent = 0
    count = 0
    lag_total = 0
    reported = clock.monotonic_ns()
    while not stop():
        try:
            frame = frames.next(FRAME_WAIT_NS)
        except TimeoutError:
            silent += 1
            if silent >= SILENT_WAITS:
                log(f"no frame for {SILENT_WAITS * FRAME_WAIT_NS // 1_000_000_000} s: the stream "
                    f"ended")
                return 1
            continue
        except BrokenPipeError as exc:
            log(f"the capture stack ended the stream ({exc})")
            return 1
        except OSError as exc:
            log(f"a frame was not handed over ({exc})")
            return 1
        silent = 0
        now_mono = clock.monotonic_ns()
        now_real = clock.realtime_ns()
        stamp = frame_stamp_ns(law, frame.timestamp_ns, frame.exposure_ns)
        size = int(frame.pitch) * int(frame.height)
        if luma is None or len(luma) != size:
            luma = array.array("B", bytes(size))
        try:
            copy(frame, luma)
        except OSError as exc:
            log(f"a frame was not read ({exc})")
            return 1
        publish(realtime_ns(stamp, now_real, now_mono), frame, luma)
        count += 1
        lag_total += now_mono - stamp
        if now_mono - reported >= REPORT_S * 1e9:
            seconds = (now_mono - reported) / 1e9
            log(f"stamp lag {lag_total / count / 1e6:.1f} ms, {count / seconds:.1f} fps")
            reported = now_mono
            count = 0
            lag_total = 0
    return 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="python -m nxs.ros2_camera_node",
                                     description="the camera node of one link")
    parser.add_argument("port")
    parser.add_argument("link")
    parser.add_argument("--topic-base", default="nxs",
                        help="the topics' first element (default: nxs)")
    parser.add_argument("--camera-info", default=None,
                        help="the link's calibration, a camera_info_manager YAML (a path or a "
                             "file:// URL)")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    from nxs import _libnxs_frames

    args = parse_args(argv)
    name = f"{args.port}/{args.link}"

    def log(line: str) -> None:
        print(line, flush=True)

    if not _libnxs_frames.supported():
        log("no capture-stack backend in this library")
        return 1
    try:
        capture, pair = _libnxs_frames.capture_pair_of(args.port, args.link)
        timing = _libnxs_frames.timing_of(args.port, args.link)
    except OSError as exc:
        log(str(exc))
        return 1
    calibration, notes = ros2_camera_info.load(args.camera_info, int(capture.width),
                                               int(capture.height))
    law = stamp_law(timing)

    import rclpy
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node(f"{args.port}_{args.link}")
    namespace = f"/{args.topic_base}/{args.port}/{args.link}"
    # Reliable matches every subscriber; a best-effort publisher never
    # reaches a reliable one.
    images = node.create_publisher(Image, f"{namespace}/image_raw", 10)
    infos = node.create_publisher(CameraInfo, f"{namespace}/camera_info", 10)
    frame_id = OPTICAL_FRAME.format(port=args.port, link=args.link)
    image, info = Image(), CameraInfo()
    stopping: List[bool] = []

    def on_signal(_signum, _frame) -> None:
        stopping.append(True)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def publish(stamp: int, frame, luma) -> None:
        fill_image(image, frame, luma, stamp, frame_id)
        fill_info(info, calibration, stamp, frame_id)
        images.publish(image)
        infos.publish(info)

    try:
        frames = _libnxs_frames.Frames(capture, pair, name)
    except OSError as exc:
        log(f"the session did not open ({exc})")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    log(STARTED)
    for line in notes:
        log(line)
    if law.note:
        log(law.note)
    try:
        return run(frames, law, publish, log, lambda: bool(stopping))
    finally:
        frames.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
