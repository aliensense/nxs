# Agent instructions

To generate a driver for a sensor attached to an NXS unit, follow
[skills/nxs-generate-sensor-driver/SKILL.md](skills/nxs-generate-sensor-driver/SKILL.md).
It is a complete instruction set: datasheet in, validated driver out —
probe, configure, measure loop, offline compile, upload, hardware checks.

Ground rules for any agent working in this tree:

- Deterministic steps live in the `nxs` CLI (`nxs upload`, `nxs run`,
  `nxs suite`, ...). Drive the CLI; do not reimplement its steps.
- Driver files under `nxs/drivers/` are generated artifacts. Regenerate them
  through the skill instead of hand-editing.
- The test suite runs hardware-free: `python -m pytest nxs/tests/ -q`.
