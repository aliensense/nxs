# Changelog

All notable changes to NXS: the `nxs` tool and daemon, the `libnxs` runtime library, the NXS unit firmware, the personalities and images, and the descriptor pack. The format follows Keep a Changelog; the versions follow the release tags.

## [Unreleased]

### Added

- The guides on the documentation site read as getting-started pages: the NVIDIA Jetson deployment guide, Getting started with the NXS unit, Multi-sensor dashboard on ROS 2, Custom sensor personality and Custom camera personality, each with a version driven from an AI agent, and a landing page that lists them. The hotfix refusal on JetPack 7.2.1 names the deployment guide.
- Camera personalities. The unit on a camera's pod stores the image sensor's program as a sealed image and runs it on the pod's bus. It serves the sensor's modes, controls and capture facts back to the host as records, so the host carries no sensor source.
- The camera personalities of the two shipped Sony heads, a global-shutter and a rolling-shutter one, at 1920x1080 RAW10 on 2 CSI lanes. The global-shutter head runs at 20 to 74 fps alone and as a frame-synced pair; the rolling-shutter head runs alone at 20 to 59 fps.
- `nxs <port> <link> upload <name>` puts a camera personality in the pod's store. A unit holds one camera personality: an upload replaces it, and `--slot` puts it in another slot.
- A camera link without a pod. The host runs the sensor's sealed image through the hub by the same steps a pod takes. A pod adds the unit's samples and time sync.
- A hub link that declares a pod and no camera comes up with the pod alone. `nxs <port> status` reads `cam0/A: pod unit-cam0-a (fxos8700), up` for it.
- The hub's and the serializer's programs ship as sealed images in the assets tarball. The host runs them through `libnxs` when it brings a port up.
- `nxs <port> set sync fsync --exposure <us>` sets the integration time under frame sync as the trigger pulse's low time. The capture stack's exposure loop then drives gain alone.
- `nxs <port> status` prints each pod's camera personality with its last run and the line time and frame length that run achieved. The sensor's status rows come from probes the unit serves.
- `ros2 launch "$(nxs ros2 --launch-file)" cameras:=true` publishes the frames of each camera link that is up on `/nxs/<port>/<link>/image_raw` and `camera_info`, through the GStreamer camera node.
- `camera_encoding:=` picks `yuv422` (the default), `mono8`, `rgb8` or `jpeg` from the host's hardware encoder. `camera_source:=argus` uses the Isaac ROS Argus node where it is installed.
- `nxs.cam.frames(port, link)` yields a link's frames in Python as HxWx3 uint8 arrays with the frame's presentation stamp, with no ROS. It needs numpy and the GStreamer Python bindings on the host.
- `nxs::cam::Frames(port, link)` in `libnxs` hands a C++ program each frame as the platform's device buffer (an NV12 dmabuf on Jetson) with the sensor's timestamp, exposure and gain. On JetPack 7.2.1 a session reports `EIO` on the first frame, so the ROS 2 topics and `nxs.cam.frames` are the frame paths there.
- `nxs personality check` and `nxs personality install` take a camera personality. An installed one extends the descriptor pack, so `nxs <port> caps` lists its modes and `suite.yaml` may name its sensor.
- The `generic` law family, for a camera whose modes run at the rate their register tables set. It runs one camera behind the hub. Another rate, frame sync or a second camera is refused.
- `CameraSensor` in the personality DSL, with `write_table()`, `load_table()`, `poll()`, `select()`, `check()` and `retry()`. The compiler that builds sensor personalities builds camera personalities too.
- A camera personality computes register values from the parameters staged for its run. `param()` reads one, `write_wide()` writes a result and `store_param()` records what the run achieved. The compiler refuses an expression that can overflow, underflow or divide by zero.
- The `nxs-generate-camera-personality` skill writes a camera personality from a datasheet and the vendor's register setting file, then compiles it.
- `nxs --experimental` runs a camera mode that has no shipped point yet, such as the mode of a new camera personality before its point is written. Nothing under the flag is part of the product.
- JetPack 7.2.1 (Jetson Linux 39.2.1) on the Jetson Orin Nano developer kit, beside JetPack 6.2.1. The boot table install finds the module's base DTB on either release.
- `nxs assets install` fetches the assets tarball of the tool's own version from the release page, asks once to accept its licence and installs it under `/opt/aliensense`. `nxs assets install <file>` takes a downloaded tarball, and `--accept-license` answers the prompt for a script.
- The tool refuses installed assets of another release and names the assets tarball of its own version.
- `nxs switch` realizes camera ports. It brings each declared pod to its declared personality and installs the port's boot table when the booted table lacks a declared mode or lane count. It then prints `REBOOT NEEDED` and exits 3.
- `nxs switch` and `nxsd` build the capture stack's configuration for the booted table, one short capture per mode, and keep it in the state store under the table's digest. A table seen before is installed without a build. `nxs <port> status` reads `preparing the capture stack` while a build runs.
- The capture stack's configuration build folds in the sensor vendor's ISP override, fetched once by URL and checked against the digest the descriptor names. An offline host copies the file to the place the refusal line names.
- `nxs switch` updates every declared unit that has no `firmware:` pin to the firmware in the installed assets.
- `nxs switch` restarts an `nxsd` of another build and prints `host: nxsd restarted (<old> -> <new>)`.
- On JetPack 7.2.1, `nxs switch` and `nxs <port> status` name NVIDIA's camera hotfix when its files are missing, before any build.
- `nxs switch` and `nxs <port> status` name a missing kernel package first, `no camera kernel package for <kernel>`, with the `apt install` line under it.
- `nxs generate --json` prints the walk as data. The MCP server gains a `generate` tool.
- `nxs tune --schema` prints the rig's rules as JSON Schema: the manifest schema narrowed to the nodes on this rig. The MCP server gains a `suite_schema` tool.
- The MCP server gains an `upload` tool that compiles a personality and uploads it to a unit.
- `suite.yaml` may be written as JSON. It loads the same as YAML.
- Sample FIFO. The unit queues every sample and the host reads them in batches over I²C, so a stream loses nothing while the host is late by less than the queue. The default depth holds 85 IMU samples, 425 ms at 200 Hz.
- `nxs stream` over I²C drains the sample FIFO and reports lost samples.
- `nxs get fifo-depth` and `nxs set fifo-depth <N>` read and set the queue's depth, where 0 means all the storage holds. The Cyphal register is `aliensense.nxs.sample_fifo.depth`, and `nxs commission --save` persists the depth.
- `libnxs`, the runtime library, ships in each platform wheel with its C headers `nxs.h` and `Frames.h`. It holds the unit client over I²C, Cyphal/CAN and Cyphal/serial, the sample stream, time sync and the port bring-up. The `nxs` tool runs on it.
- Firmware: `CAM_RUN` runs a camera personality from a store slot on the pod's bus, and `CAM_ABORT` stops the run within 50 ms. `CAM_STATE` and `CAM_ERROR` report the run's state and errno.
- Firmware: a cursor written after the `SAMPLE_DATA` pointer reads a burst of queued samples. The device-parameter view (`DRIVER_SELECT = 4`) sets the FIFO depth over I²C.
- Firmware: `XFER_TYPE` 6 and 7 serve a camera personality's descriptor trailer, up to 2048 bytes.
- Firmware: the personality instructions `POLL_REG`, `PARAM_LOAD`, `PARAM_STORE`, `MUL_REG` and `DIVU_REG`.
- An offline bundle per Jetson Linux release, `nxs-<version>-bundle-<l4t>.tar.gz`, with the aarch64 wheel, the assets tarball and the kernel package.
- Each platform wheel carries the NXS Runtime Library License for `libnxs` as `nxs/_lib/LICENSE`. The Python code and the headers stay Apache-2.0.
- `NOTICE` lists the third-party parts of `libnxs`, the unit firmware and the bootloader with their licence texts. The wheel carries it, and the assets tarball carries it with `LICENSE-Apache-2.0` beside its own `LICENSE`.


Earlier releases were internal; 1.1.0 is the first public release.
