# nxs

Python SDK and command-line tool for NXS sensor co-processors. The library
covers personality authoring (a sensor personality is a Python class compiled to VM
bytecode) and integration (`open_client()`, decoded sample streams, parameter
access). The `nxs` CLI is built on the same public imports: personality compile and
upload, VM control, sample streaming, personality store management, and firmware
update over I2C, serial, or CAN.

```python
from nxs import open_client

client = open_client("i2c", bus="/dev/i2c-2", address=0x30)
for sample in client.iter_samples():
    handle(sample.values)
```

## Contracts and agents

Every YAML the tool reads (the suite manifest, unit personality descriptors, camera
descriptor packs) ships with a JSON Schema under `nxs/schemas/`, enforced by
editors through the files' `yaml-language-server` modelines and by CI on the
shipped personalities and packs. `nxs status` validates the suite manifest against
its schema before the laws run; the pack, descriptor, blob, and topology
loaders parse strictly (an unknown key or a malformed value is an error naming
the file). Every read verb also emits its document as JSON (`nxs status --json`,
`nxs cam1 status --json`, `nxs cam1 caps --json`), each conforming to the shipped
`surface` contract. Shape is the schema's job; the timing and feasibility laws
stay in code and answer through `nxs status`: a refusal is one fact line, then
the lawful alternatives under it.

`nxs mcp` serves the same verbs to an AI agent over the Model Context
Protocol (install the `mcp` extra). Hand-written YAML, the `nxs tune` personality
panel, and the MCP tools edit one declared config through the same laws; MCP
also operates the hardware, and a refused request carries its refusal. See
the MCP Tool Reference in the released documentation set.

## Install

Requires Python ≥ 3.10. The base install covers every wire, I2C, Cyphal/serial
and Cyphal/CAN, through the bundled library; the `cyphal` extra adds pyserial for
the serial port autodetect. The Cyphal DSDL ships vendored in the wheel for yakut
and yukon, so they need no repo checkout and no `public_regulated_data_types` clone.

**uv**, native with direct device access; the only option on macOS, where
Docker cannot reach USB:

```
uv tool install 'aliensense-nxs[cyphal]'      # omit [cyphal] for an I2C-only install
```

**pip**, from a release wheel:

```
python3 -m pip install 'aliensense_nxs-<version>-py3-none-manylinux_2_35_x86_64.whl[cyphal]'
```

**Container**, for fleet, hermetic, or locked-down hosts; ships no ROS. See
`Dockerfile`:

```
docker build --build-arg EXTRA='[cyphal]' -t nxs:cyphal .
docker run --rm --device /dev/ttyUSB0 nxs:cyphal --transport cyphal-serial \
    --port /dev/ttyUSB0 --format json
```

`nxs switch` installs the platform's udev bus aliases and the tab completion
at `/etc/bash_completion.d/nxs` (the writes under `/etc` go through sudo);
zsh sources the same file after `autoload -U bashcompinit && bashcompinit`.

## Deploy to a Jetson

From a dev host to the target. The target keeps exactly one nxs, the
installed wheel. Never copy the source tree to the target and never
`pip install -e` there: a second copy shadows the wheel and personality
edits stop reaching the compiler.

**1. Build the wheel** (dev host, in this directory):

```
python3 -m pip install build
python3 -m build --wheel
```

The wheel lands in `dist/`. A standalone checkout carries the vendored
DSDL and builds as-is; in a source checkout, run `../scripts/vendor-dsdl.sh`
first so the bundle is current.

**2. Copy to the Jetson and install:**

```
scp dist/aliensense_nxs-<version>-py3-none-manylinux_2_35_aarch64.whl <user>@<jetson>:/tmp/
ssh <user>@<jetson> "python3 -m pip install --user --force-reinstall 'aliensense-nxs[cyphal] @ file:///tmp/aliensense_nxs-<version>-py3-none-manylinux_2_35_aarch64.whl'"
```

`--force-reinstall` because dev wheels reuse the version number. The
PEP 508 `name[extra] @ file://` form is required; pip does not apply
extras to a bare wheel path. For an I2C-only target install the bare
path with no extras: `pip install --user --force-reinstall /tmp/aliensense_nxs-<version>-py3-none-manylinux_2_35_aarch64.whl`.

**3. Verify the single installed copy:**

```
ssh <user>@<jetson> "nxs --version && python3 -c 'import nxs, os; print(os.path.dirname(nxs.__file__))'"
```

The printed path must be under `~/.local/lib/python3.*/site-packages`.
Any other path (a home-directory source tree, an editable install) is
a shadowing copy: `python3 -m pip uninstall nxs` until none remains,
then reinstall the wheel.

**Iterating on a personality** needs no redeploy. Compile on the dev host,
ship the image:

```
nxs upload fxos8700 --config bus=spi -o fxos8700.nxs    # dev host: compile only
scp fxos8700.nxs <user>@<jetson>:
ssh <user>@<jetson> nxs upload fxos8700.nxs
```

An image whose format predates the installed tool is refused together
with its rebuild command.

Then bring the sensor up with the **Quick start** commands below
(`probe`, `upload iam20680 --param sample_rate=250`, `stream`,
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
messages by semantic, with no per-sensor configuration. With a suite
manifest it bridges every unit under `nxs/<unit>/...`; with `--unit` or
explicit transport flags it bridges one device. Install the wheel with the
`ros2` extra (`aliensense-nxs[cyphal,ros2]`) into a sourced ROS 2 environment (`rclpy`
comes from the distribution; only this verb imports it) and run:

```
nxs ros2 --plan     # print the topic plan, no ROS required
nxs ros2            # publish; ros2 topic echo nxs/<unit>/imu
```

The topic table, timestamp and covariance conventions are in the
integration manual's ROS 2 section.

## Cameras (nxs cam)

`nxs cam` brings up and drives camera chains (an image sensor behind a
serializer behind a deserializer link) from **descriptor packs**:
runtime-discovered bundles that describe the hardware (register maps,
modes, limits, flow assembly). The tool is hardware-agnostic; the pack is
the camera's self-description.

Commands address the hardware node first: the port, then optionally
one link:

```
nxs cam0 status              # presence of every device on every link, then the readbacks
nxs cam0 caps                # modes and knobs: feature discovery
nxs cam0 on                  # bring the port's links up (constructs as needed)
nxs cam0 A set exposure 4ms  # a knob on one link, judged by the laws; a refused
                             # value names the lawful alternatives
nxs cam0 capture --frames 60  # delivery at the declared rate, headless
nxs cam0 off
```

`cam0` is the connector's stable name (`nxs switch` installs the platform's
udev aliases); `A`/`B` are its links. The ports come from the suite
manifest's `ports:` section (which hub on which bus, which chains behind
it, the declared mode and sync); a host without a manifest follows the
booted device tree and the pack's default topology, and `nxs tune --freeze
--ports` writes what it found. The pack ships inside the wheel (a development host names its own with
`nxs --experimental` and `$NXS_CAM_DESCRIPTORS`); without a pack for the
described deserializer, the tool says so and where it searched. Authoring a pack for
your own camera is the Camera Personality Reference (`docs/nxs-camera-personalities.md`
in a standalone checkout), shipped with the product documentation. The
guide set on the documentation site
(https://aliensense.github.io/nxs-docs/guides/) walks from a fresh Jetson
to a dual camera.

## Reference

The full interface contract (register map, procedures, framing, limits)
is the NXS host interface specification, shipped with the product
documentation.

## Building from source

The Cyphal DSDL is vendored into the wheel but not committed. It is assembled
from the firmware's `dsdl/aliensense` types and the full `uavcan` namespace of
OpenCyphal's `public_regulated_data_types`, which yakut and yukon need beside
the vendor types (`uavcan.node`, `uavcan.file`, and their dependencies).

A standalone checkout carries the bundle in `nxs/dsdl/` already, assembled
by the release pipeline before building the wheel. Run that step by hand before building the cyphal
container or running the cyphal tests from a fresh source checkout:

```
../scripts/vendor-dsdl.sh        # needs `west update`, or PRDT=/path
```
