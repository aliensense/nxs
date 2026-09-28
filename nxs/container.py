"""Container entry point: one wire in, decoded samples out. `--transport
{i2c, cyphal-serial}` picks the source; both yield the same `Sample`, so the
output loop is wire-blind. Sinks print decoded fields or one JSON object per line."""

import argparse
import os
import json
from typing import Callable, Optional

from nxs.client import Sample


def make_source(args):
    """Build the sample source for `--transport`: the library's client, whose
    `iter_samples()` yields `Sample`."""
    from nxs.transports import open_client
    if args.transport == "i2c":
        if args.bus is None:
            raise SystemExit("container: pass --bus or set $NXS_BUS")
        return open_client("i2c", bus=args.bus, address=args.addr)
    if args.transport == "cyphal-serial":
        if not args.port:
            from nxs.serial_util import autodetect_serial_port
            args.port = autodetect_serial_port()
            if not args.port:
                raise SystemExit("no serial port: pass --port explicitly for cyphal-serial")
        return open_client("cyphal-serial", port=args.port, baud=args.baud)
    if args.transport == "cyphal-can":
        return open_client("cyphal-can", can_iface=args.port or "can0")
    raise SystemExit(f"unknown transport: {args.transport}")


def print_sink(sample: Sample) -> None:
    fields = "  ".join(f"{k}={v:.4f}" if isinstance(v, float) else f"{k}={v}"
                       for k, v in sample.values.items())
    print(f"[{sample.count}] {fields}")


def json_sink(sample: Sample) -> None:
    """One JSON object per line for a downstream consumer (the customer's ROS
    node, a logger, a script) to parse."""
    obj = {"seq": sample.count}
    if sample.timestamp_us is not None:
        obj["t_us"] = sample.timestamp_us
    obj.update(sample.values)
    print(json.dumps(obj))


def run(source, sink: Callable[[Sample], None], count: Optional[int] = None) -> None:
    """Stream decoded samples to `sink`, stopping after `count` (or forever),
    and close the source on exit."""
    n = 0
    try:
        for sample in source.iter_samples():
            sink(sample)
            n += 1
            if count is not None and n >= count:
                break
    finally:
        close = getattr(source, "close", None)
        if callable(close):
            close()


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="nxs-container",
                                description="Decode NXS samples over any wire.")
    p.add_argument("--transport", required=True,
                   choices=["i2c", "cyphal-serial", "cyphal-can"])
    p.add_argument("--port", default=None, help="serial device (cyphal-serial)")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--bus", default=os.environ.get("NXS_BUS"),
                   help="I2C bus (i2c; $NXS_BUS)")
    p.add_argument("--addr", type=lambda x: int(x, 0), default=0x30,
                   help="I2C address (i2c)")
    p.add_argument("--count", type=int, default=None, help="stop after N samples")
    p.add_argument("--format", choices=["pretty", "json"], default="pretty",
                   help="pretty for a terminal, json (one object per line) for a consumer")
    args = p.parse_args(argv)

    sink = json_sink if args.format == "json" else print_sink
    run(make_source(args), sink, count=args.count)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
