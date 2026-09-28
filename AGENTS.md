# Agent instructions

To generate a driver for a sensor attached to an NXS unit, follow
[skills/nxs-generate-sensor-personality/SKILL.md](skills/nxs-generate-sensor-personality/SKILL.md).
It is a complete instruction set, datasheet in, validated driver out:
probe, configure, measure loop, offline compile, upload, hardware checks.
For an image sensor on a camera pod, follow
[skills/nxs-generate-camera-personality/SKILL.md](skills/nxs-generate-camera-personality/SKILL.md):
datasheet and vendor setting file in, descriptor, behaviour class and
register tables out.

Ground rules for any agent working in this tree:

- Deterministic steps live in the `nxs` CLI (`nxs upload`, `nxs run`,
  `nxs switch`, ...). Drive the CLI; do not reimplement its steps.
- Camera bring-up flows live in descriptor packs. Drive `nxs cam`
  (`on`, `status`, `set`, ...); do not poke sensor or SerDes registers
  by hand, and put chip knowledge in a pack, never in `nxs/cam/` code.
- Driver files under `nxs/drivers/` are generated artifacts. Regenerate them
  through the skill instead of hand-editing.
- The tests live in the private repository. A standalone checkout is checked
  with `pip install -e '.[cyphal]'` and `nxs --version`.
