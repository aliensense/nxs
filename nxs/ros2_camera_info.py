# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A link's calibration for its camera topic: the camera_info_manager YAML
the calibration guide writes under `/etc/aliensense/camera_info/`, read
into the fields of `sensor_msgs/CameraInfo`."""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Tuple

import yaml

#: The distortion model an uncalibrated link publishes under.
PLUMB_BOB = "plumb_bob"


@dataclasses.dataclass(frozen=True)
class Calibration:
    """The `CameraInfo` fields a calibration file carries."""

    width: int
    height: int
    distortion_model: str
    d: Tuple[float, ...]
    k: Tuple[float, ...]
    r: Tuple[float, ...]
    p: Tuple[float, ...]


def uncalibrated(width: int, height: int) -> Calibration:
    """The zeros an uncalibrated link publishes: every consumer reads an
    all-zero `k` as no calibration."""
    return Calibration(width, height, PLUMB_BOB, (0.0,) * 5, (0.0,) * 9, (0.0,) * 9, (0.0,) * 12)


def _matrix(doc: dict, key: str, count: int) -> Tuple[float, ...]:
    """The `count` numbers of the matrix under `key`.

    Raises:
        ValueError: No data list, a value missing (`~`) or not a number, or
            another count, each naming the key.
    """
    block = doc.get(key)
    if not isinstance(block, dict) or not isinstance(block.get("data"), list):
        raise ValueError(f"{key}: no data list")
    data = []
    for index, value in enumerate(block["data"]):
        if value is None:
            raise ValueError(f"{key}: value {index} is missing")
        try:
            data.append(float(value))
        except (TypeError, ValueError):
            raise ValueError(f"{key}: value {index} is not a number ({value!r})") from None
    if len(data) != count:
        raise ValueError(f"{key}: {len(data)} values, not {count}")
    return tuple(data)


def parse(text: str) -> Calibration:
    """A camera_info_manager YAML document as a calibration.

    Raises:
        ValueError: A document without the matrices, with one of the wrong
            size, or with a value missing or not a number in one.
    """
    doc = yaml.safe_load(text)
    if not isinstance(doc, dict):
        raise ValueError("not a calibration document")
    try:
        width, height = int(doc["image_width"]), int(doc["image_height"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("image_width and image_height are required") from exc
    distortion = doc.get("distortion_coefficients")
    count = len(distortion["data"]) if isinstance(distortion, dict) \
        and isinstance(distortion.get("data"), list) else 5
    return Calibration(width, height, str(doc.get("distortion_model") or PLUMB_BOB),
                       _matrix(doc, "distortion_coefficients", count),
                       _matrix(doc, "camera_matrix", 9), _matrix(doc, "rectification_matrix", 9),
                       _matrix(doc, "projection_matrix", 12))


def load(path: Optional[str], width: int, height: int) -> Tuple[Calibration, List[str]]:
    """The calibration at `path` (a `file://` URL or a path), else the zeros,
    with the notes the node logs once: the file absent, unreadable, or
    sized for another geometry."""
    if not path:
        return uncalibrated(width, height), []
    name = path[len("file://"):] if path.startswith("file://") else path
    try:
        with open(name, encoding="utf-8") as fh:
            calibration = parse(fh.read())
    except FileNotFoundError:
        return uncalibrated(width, height), [f"no calibration at {name}: camera_info carries zeros"]
    except (OSError, ValueError, yaml.YAMLError) as exc:
        return uncalibrated(width, height), [f"{name}: {exc}: camera_info carries zeros"]
    notes = []
    if (calibration.width, calibration.height) != (width, height):
        notes.append(f"{name}: calibrated at {calibration.width}x{calibration.height}, "
                     f"the link runs {width}x{height}")
    return calibration, notes
