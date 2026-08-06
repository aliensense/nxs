# nxs

Python SDK and command-line tool for NXS sensor co-processors. The
library carries two surfaces — driver authoring (write a sensor driver
as a Python class, compile it to VM bytecode) and integration
(`open_client()`, decoded sample streams, parameter access) — and the
`nxs` CLI is built on the same public imports: driver compile and
upload, VM control, sample streaming, driver store management, and
firmware update, over I2C, serial, or CAN.

```python
from nxs import open_client

client = open_client("i2c", bus="/dev/i2c-2", address=0x30)
for sample in client.iter_samples():
    handle(sample.values)
```

## Install

Requires Python ≥ 3.10. The base install covers I2C and serial; the
`cyphal` extra adds the pycyphal source for Cyphal/serial. The Cyphal DSDL
ships vendored in the wheel, so the install is self-contained — no repo
checkout and no `public_regulated_data_types` clone.

**uv** — native, with direct device access; the only option on macOS, where
Docker cannot reach USB:

```
uv tool install 'nxs[cyphal]'      # omit [cyphal] for an I2C-only install
```

**pip**, from a release wheel:

```
python3 -m pip install 'nxs-<version>-py3-none-any.whl[cyphal]'
```

**Container** — the second channel, for fleet / hermetic / locked-down hosts;
ships no ROS. See `Dockerfile`:

```
docker build --build-arg EXTRA='[cyphal]' -t nxs:cyphal .
docker run --rm --device /dev/ttyUSB0 nxs:cyphal --transport cyphal-serial \
    --port /dev/ttyUSB0 --format json
```

## Deploy to a Jetson

From a dev host to the target. The target keeps exactly one nxs — the
installed wheel. Never copy the source tree to the target and never
`pip install -e` there: a second copy shadows the wheel and driver
edits stop reaching the compiler.

**1 — Build the wheel** (dev host, in this directory):

```
python3 -m pip install build
python3 -m build --wheel
```

The wheel lands in `dist/`. A standalone checkout carries the vendored
DSDL and builds as-is; inside the firmware repository, run
`../scripts/vendor-dsdl.sh` first so the bundle is current.

**2 — Copy to the Jetson and install:**

```
scp dist/nxs-<version>-py3-none-any.whl <user>@<jetson>:/tmp/
ssh <user>@<jetson> "python3 -m pip install --user --force-reinstall 'nxs[cyphal] @ file:///tmp/nxs-<version>-py3-none-any.whl'"
```

`--force-reinstall` because dev wheels reuse the version number. The
PEP 508 `name[extra] @ file://` form is required — pip does not apply
extras to a bare wheel path. For an I2C-only target install the bare
path with no extras: `pip install --user --force-reinstall /tmp/nxs-<version>-py3-none-any.whl`.

**3 — Verify the single installed copy:**

```
ssh <user>@<jetson> "nxs --version && python3 -c 'import nxs, os; print(os.path.dirname(nxs.__file__))'"
```

The printed path must be under `~/.local/lib/python3.*/site-packages`.
Any other path — a home-directory source tree, an editable install — is
a shadowing copy: `python3 -m pip uninstall nxs` until none remains,
then reinstall the wheel.

**Iterating on a driver** needs no redeploy. Compile on the dev host,
ship the image:

```
nxs upload fxos8700 --config bus=spi -o fxos8700.nxs    # dev host: compile only
scp fxos8700.nxs <user>@<jetson>:
ssh <user>@<jetson> nxs upload fxos8700.nxs
```

An image whose format predates the installed tool is refused together
with its rebuild command.

Then bring the sensor up with the **Quick start** commands below
(`probe` → `upload iam20680 --param sample_rate=250` → `stream` →
`store save 0`).

## Quick start

```
nxs probe
nxs upload iam20680 --param sample_rate=250
nxs stream
nxs store save 0
nxs push-fw zephyr.signed.bin
```

The default transport is I2C on a Linux host (Jetson/SBC) and
cyphal-serial on macOS or Windows, which have no `/dev/i2c-*`. `$NXS_TRANSPORT`
overrides the choice (`i2c` / `cyphal-serial` / `cyphal-can`); `$NXS_BUS`
sets the I²C bus and `$NXS_PORT` the serial port or SocketCAN interface,
or pass `-t`, `-b`, `-p` per command. `nxs --help` lists every command.

## ROS 2

`nxs ros2` maps the device-served field descriptors onto standard ROS 2
messages by semantic — no per-sensor configuration. With a suite
manifest it bridges every unit under `nxs/<unit>/...`; with `--unit` or
explicit transport flags it bridges one device. ROS stays the
environment's: install the wheel into a sourced ROS 2 environment
(`rclpy` is imported only by this verb) and run:

```
nxs ros2 --plan     # print the topic plan, no ROS required
nxs ros2            # publish; ros2 topic echo nxs/<unit>/imu
```

The topic table, timestamp and covariance conventions are in the
integration manual's ROS 2 section.

## Reference

The full interface contract — register map, procedures, framing,
limits — is the NXS host interface specification, shipped with the
product documentation.

## Building from source

The Cyphal DSDL is vendored into the wheel but not committed — it is assembled
from two sources of truth: the firmware's `dsdl/aliensense` types and the full
`uavcan` namespace of OpenCyphal's `public_regulated_data_types` (pinned to the
firmware's west revision) — the stock pycyphal Node and file server need
`uavcan.node`, `uavcan.file`, and their dependencies.

A standalone checkout carries the bundle in `nxs/dsdl/` already. Inside
the firmware repository it is git-ignored codegen output:
`../scripts/vendor-dsdl.sh` assembles it, the release pipeline runs that
before building the wheel, and the published wheel is self-contained —
an end user's `pip` / `uv` install needs no repo checkout and no PRDT
clone. Run it by hand before building the cyphal container or running
the cyphal tests from a fresh firmware-repo checkout:

```
scripts/vendor-dsdl.sh        # needs `west update`, or PRDT=/path
```
