# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""A live knob on a head that serves the camera map, a pod's or a smart
camera's: one row committed on the link's camera map, in the row's unit
(Interface Description §13). The host's forms of a standard row are the
exposure as a time, the gain in decibels and the rate in frames a second;
any other knob is the row of that name, which the map judges."""

from __future__ import annotations

import math

from nxs._generated_constants import NxsMap
from nxs.cam.contracts import InfeasibleConfig
from nxs.cam.plan import RawConfig

ParamId = NxsMap.ParamId

#: The standard row each host form of a knob commits.
FORMS = {"gain": ParamId.GAIN_MDB, "gain_db": ParamId.GAIN_MDB, "exposure": ParamId.EXPOSURE_US,
         "fps": ParamId.FRAME_RATE_MFPS}


def parse_us(value: str) -> float:
    """A time in microseconds: `4ms`, `4000us`, or a bare number of
    microseconds."""
    v = value.strip().lower()
    if v.endswith("ms"):
        return float(v[:-2]) * 1000.0
    if v.endswith("us"):
        return float(v[:-2])
    return float(v)


def _number(knob: str, value: str, parse, takes: str, alternatives) -> int:
    """`value` as the row's number, or the refusal naming what the knob takes:
    a value that is no number, or none a finite cell holds."""
    try:
        return int(parse(value))
    except (ValueError, ZeroDivisionError, OverflowError):
        raise InfeasibleConfig(f"{knob} takes {takes}, not {value!r}", alternatives=alternatives)


def rate(value: str) -> float:
    """A rate in frames a second, or the refusal naming what `fps` takes."""
    try:
        fps = float(value)
    except ValueError:
        raise InfeasibleConfig(f"fps takes frames a second, not {value!r}", alternatives=["fps <frames a second>"])
    if not math.isfinite(fps) or fps <= 0:
        raise InfeasibleConfig(f"fps takes frames a second, not {value!r}", alternatives=["fps <frames a second>"])
    return fps


def map_knob(link: str, knob: str, value: str) -> RawConfig:
    """The plan of one knob on `link`'s camera map: the exposure in
    microseconds, the gain in millidecibels, the rate in milli-frames per
    second; any other knob is the row of that name, an integer in the
    row's own unit. A rate is judged by the port's laws before this plan
    is made, and the value against the row the map serves when it commits
    (`map_commit.within`)."""
    cfg = RawConfig(f"set-{knob}")
    if knob not in FORMS:
        cfg.add_map_commit(f"set_{knob}", link, knob,
                           _number(knob, value, lambda v: int(v, 0), "a whole number in the row's unit",
                                   [f"{knob} <whole number>"]))
        return cfg
    if knob == "fps":
        staged = _number(knob, value, lambda v: round(float(v) * 1000), "frames a second",
                         ["fps <frames a second>"])
    elif knob == "exposure":
        staged = _number(knob, value, lambda v: round(parse_us(v)), "a time: 4ms, 4000us or 4000",
                         ["exposure <us|ms>"])
    elif knob == "gain_db":
        staged = _number(knob, value, lambda v: round(float(v) * 1000), "decibels", ["gain_db <dB>"])
    else:
        staged = _number(knob, value, lambda v: int(v, 0), "whole millidecibels",
                         ["gain <mdB>", "gain_db <dB>"])
    cfg.add_map_commit(f"set_{knob}", link, FORMS[knob], staged)
    return cfg
