#!/usr/bin/env python3
# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""stampstats: a bench tool beside `nxs`, not one of its verbs.

Subscribe to NXS topics for a while and judge their header stamps: the
period's median, the p99 deviation from it, the largest gap in periods, the
gaps above two periods, and for two camera topics the stamp difference.

usage: stampstats.py SECONDS TOPIC:TYPE [TOPIC:TYPE ...]   (TYPE = imu | info)
"""

import statistics
import sys
import time


def sink(stamps, arrivals, topic):
    """The callback that keeps each message of `topic`: its header stamp and
    its arrival, both nanoseconds on the realtime clock the stamps run on."""
    def on_msg(msg):
        stamps[topic].append(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        arrivals[topic].append(time.time_ns())
    return on_msg


def judge(s):
    """The cadence of one topic's stamps `s` (ns, in order): the span, the
    rate, the period's median, the p99 deviation from it, the largest period
    in medians, the periods over two medians, and the samples those gaps
    swallowed (a period of k medians hides k - 1)."""
    periods = [b - a for a, b in zip(s, s[1:])]
    median = statistics.median(periods)
    devs = sorted(abs(p - median) for p in periods)
    span = (s[-1] - s[0]) / 1e9
    return {"span_s": span, "rate_hz": (len(s) - 1) / span, "median_ns": median,
            "p99_dev_ns": devs[int(0.99 * (len(devs) - 1))],
            "largest_periods": max(periods) / median,
            "gaps": sum(1 for p in periods if p > 2 * median),
            "missing": sum(round(p / median) - 1 for p in periods if p > 1.5 * median)}


def main() -> int:
    if len(sys.argv) < 3 or sys.argv[1] in ("-h", "--help"):
        print(__doc__.strip())
        return 0 if len(sys.argv) > 1 and sys.argv[1] in ("-h", "--help") else 2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Imu

    types = {"imu": Imu, "info": CameraInfo}
    seconds = float(sys.argv[1])
    specs = [arg.rsplit(":", 1) for arg in sys.argv[2:]]
    rclpy.init()
    node = Node("stampstats")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=500)
    stamps = {topic: [] for topic, _ in specs}
    arrivals = {topic: [] for topic, _ in specs}
    subs = [node.create_subscription(types[kind], topic, sink(stamps, arrivals, topic), qos)
            for topic, kind in specs]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    del subs
    node.destroy_node()
    rclpy.shutdown()
    for topic, _ in specs:
        s = stamps[topic]
        if len(s) < 3:
            print(f"{topic}: {len(s)} messages")
            continue
        j = judge(s)
        lag = statistics.median(a - t for a, t in zip(arrivals[topic], s)) / 1e6
        print(f"{topic}: n={len(s)} over {j['span_s']:.1f} s, rate {j['rate_hz']:.2f} Hz, "
              f"period median {j['median_ns'] / 1e6:.3f} ms, p99 dev {j['p99_dev_ns'] / 1e3:.0f} us, "
              f"largest {j['largest_periods']:.2f} periods, gaps>2 periods {j['gaps']}, "
              f"missing {j['missing']}, stamp-to-arrival median {lag:.1f} ms")
    infos = [topic for topic, kind in specs if kind == "info"]
    if len(infos) == 2 and all(len(stamps[t]) > 3 for t in infos):
        a, b = (stamps[t] for t in infos)
        j, diffs = 0, []
        for t in a:
            while j + 1 < len(b) and abs(b[j + 1] - t) < abs(b[j] - t):
                j += 1
            diffs.append(b[j] - t)
        diffs.sort()
        print(f"{infos[1]} minus {infos[0]} at the nearest frame: median {statistics.median(diffs) / 1e3:.0f} us, "
              f"min {diffs[0] / 1e3:.0f} us, max {diffs[-1] / 1e3:.0f} us, pairs {len(diffs)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
