"""Container entry point: one wire in, decoded samples out.

`--transport {i2c, cyphal-serial}` picks the source — the I2C reg-map bridge or
the pycyphal `CyphalSampleSource` — and both yield the same `Sample`, so the
output loop is wire-blind. The default sink prints decoded fields; the ROS 2
projection is a thin sink over the same `Sample.values`.

This is the host container the customer runs: `docker run … --transport
cyphal-serial --port /dev/ttyUSB0` decodes a self-describing Cyphal node;
`--transport i2c --bus /dev/i2c-2` decodes the GMSL reg-map. pycyphal owns the
Cyphal wires, nxs's I2C transport owns the reg-map.
"""

import argparse
import json
from typing import Callable, Optional

from nxs.client import Sample


def make_source(args):
    """Build the sample source for `--transport`. Both returned objects expose
    `iter_samples()` yielding `Sample`."""
    if args.transport == "i2c":
        from nxs.transports import open_client
        return open_client("i2c", bus=args.bus, address=args.addr)
    if args.transport == "cyphal-serial":
        from nxs.transports.cyphal_source import CyphalSampleSource
        if not args.port:
            from nxs.serial_util import autodetect_serial_port
            args.port = autodetect_serial_port()
            if not args.port:
                raise SystemExit("no serial port: pass --port explicitly for cyphal-serial")
        return CyphalSampleSource(port=args.port, baud=args.baud)
    if args.transport == "cyphal-can":
        # The CyphalSampleSource transport is serial-only; CAN needs the pycyphal
        # CANTransport over a socketcan media, which the container does not provide.
        raise SystemExit("cyphal-can is unsupported in the container — use cyphal-serial or i2c")
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
    p.add_argument("--bus", default="/dev/i2c-2", help="I2C bus (i2c)")
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
