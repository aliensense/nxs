# Changelog

Product releases of NXS. Version semantics (see the Datasheet, Versioning section):

- **MAJOR** — the host-facing contract changed incompatibly (register map, served Cyphal types): requalify the integration.
- **MINOR** — the driver image format moved: rebuild compiled `.nxs` artifacts with the matching `nxs` tool; deployed devices are unaffected.
- **PATCH** — no compatibility impact: drop-in.

## 1.0.0 — Unreleased

First release. Image format major 1, minor 0.

- Sensor co-processor with an on-device driver VM: runtime driver upload, self-described SI outputs, persistent driver store (8 slots).
- Three host transports: I²C register map (target `0x30`), Cyphal/serial (460800), Cyphal/CAN-FD (1M/4M).
- Canonical SI units enforced at driver compile time; temperature in kelvin at the source.
- Standard `uavcan.si.sample.*` subject projection with per-subject decimation; vendor `RawSample` + descriptor services.
- Multi-node commissioning: persistent node-ID and subject-ID configuration.
- Dual-slot firmware update with automatic rollback, over all transports.
- Driver DSL: register, command-response, stream, and custom-FRAME driver classes; `read(reg, width, signed=…, endian=…)`, `read_burst(reg, n, into=off)`, `read_analog(ch)` (mikroBUS AN pad), `drive_pwm(freq, duty)` (mikroBUS PWM pad), and `if`/`elif`/`else` in `measure()`. Enum and range (`[min, max]`) parameters, reload or live, runtime-settable via `nxs set`; the image parser rejects malformed descriptors at load with named errors.
- Shipped drivers: `iam20680`, `iim20670` (6-axis IMUs), `fxos8700` (accelerometer + magnetometer), `mc6470` (two-die eCompass on one I²C bus, via the companion-device DSL), `as5047d` (magnetic rotary encoder), `ms5611` (barometer), `neo_m9n` and `zed_f9p` (u-blox GNSS, UBX geodetic output).
- Factory test: 3-board bench rig (host + DUT + reference emulator) with 8-signal PASS/FAIL acceptance computed on the DUT, including a Cyphal/CAN-FD round-trip against the DUT's shipping node — `tools/factory/`, the `nxs-v1.0-test-ref` firmware, and the bring-up runbook.
- Released documentation set: Datasheet, Interface Description, Integration & Operation Manual, Driver Development Guide.
