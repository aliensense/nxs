# Agent instructions

To generate a click personality for a sensor attached to an NXS unit, follow [skills/nxs-generate-click-personality/SKILL.md](skills/nxs-generate-click-personality/SKILL.md). It is a complete instruction set, datasheet in, validated click personality out: probe, configure, measure loop, offline compile, upload, hardware checks. For an image sensor on a camera pod, follow [skills/nxs-generate-cam-personality/SKILL.md](skills/nxs-generate-cam-personality/SKILL.md): datasheet and vendor setting file in, facts file, behaviour class and register tables out.

Ground rules for any agent working in this tree:

- Deterministic steps live in the `nxs` CLI (`nxs upload`, `nxs run`, `nxs switch`, ...). Drive the CLI and never reimplement its steps.
- Camera bring-up flows live in the hubs, and a sensor's program in its cam personality. Drive `nxs cam` (`on`, `status`, `set`, ...) and never poke sensor or SerDes registers by hand. Put chip knowledge in a hub or a cam personality, never in `nxs/cam/` code.
- Click personality files under `nxs/click_personalities/` are generated artifacts. Regenerate them through the skill instead of hand-editing.
- The tests live in the private repository. A standalone checkout is checked with `pip install -e '.[cyphal]'` and `nxs --version`.
