# Inertial class (accelerometer / gyroscope)

Conventions for parts streaming acceleration and/or angular rate —
3-axis accelerometers, gyros, 6-axis IMUs. A combo package (IMU +
magnetometer, eCompass) reads this file for its inertial dies and
`magnetometer.md` for the mag die.

## Canonical parameters

| Concept | Name | Values |
|---|---|---|
| Output data rate | `sample_rate` | the part's ODR rows, Hz |
| Accel full-scale | `accel_fs` | the FS rows, in g |
| Gyro full-scale | `gyro_fs` | the FS rows, in dps |
| Accel bandwidth | `accel_bw` | cutoff rows, Hz |
| Gyro bandwidth | `gyro_bw` | cutoff rows, Hz |
| Joint filter knob | `filter_hz` | when ONE register filters all quantities |

Values are physical magnitudes from the datasheet tables (g, dps,
Hz), never register codes. Prefer runtime params: a full-scale or
filter field on a read-modify-write register is still a runtime param
— tag its `write_modify` with `param=` and clear the whole field (see
the `write_modify` rule). A field split across two registers (a filter
cutoff at two axis registers) owns two patch sites. A setting stays a
compile-time config key only when its write is genuinely
value-independent. When the datasheet's range labels are rounded (a
"61 dps" range whose true full-scale is 61.44), the label is the clean
scale_param value; the resulting scale is within the part's
sensitivity tolerance — note the approximation in the driver.
Consume every table row or state which rows were dropped and why.

## Output vector

- `accel_x/y/z` in m/s² — sensitivity from the datasheet's
  LSB-per-g table; base scale `9.80665 / counts_per_g`, with
  `scale_param` tracking a runtime full-scale.
- `gyro_x/y/z` in rad/s — fold `pi/180` into the dps sensitivity.
- Die temperature is a measurand: publish `temp` in kelvin
  (sensitivity and offset from the temperature section, +273.15
  folded) unless the wire-time budget excludes it — stated either
  way.
- Field order and byte order follow the part's data-register /
  packet order exactly; never reorder for readability.

## Identity and probe

- Anchor on WHO_AM_I. When identity bits are factory-variable (a
  product-code register spanning several legal values), enumerate
  EVERY documented value in `WHO_AM_I_VALUES`.
- Provenance gates how hard the set may be trusted. A set of
  *listed* datasheet values is normal evidence. A set *derived*
  from a fixed-bits claim ("bits n:m vary, the rest read 0") is
  weak — silicon ships with "fixed" bits set. Corroborate a
  derived set against a vendor or kernel reference driver when
  one is retrievable and cite the source in the identity comment;
  with no second source, a derived set must not be the probe's
  only hard assert when the part offers a stronger anchor (a
  fixed companion WIA, a second ID register).
- `0x00` and `0xFF` never belong in a WHO_AM_I set without a
  stated justification: they also match empty or mispointed reads
  (an unimplemented register commonly reads `0x00`), so a set
  containing them false-accepts a wrong die.
- Some parts return a valid WHO_AM_I only after their mandated
  soft reset — order reset and probe per the datasheet and seed
  the tracer accordingly.
- `I2C_ADDRS` carries the full strap set.

## Configuration order

- Standby-gated parts (config registers writable only in
  standby/sleep): enter standby first, write config, drive the
  part active LAST.
- Bank-switched or unlock-gated parts: follow the datasheet's
  unlock/bank walk verbatim (see the FRAME rules), returning to
  the data bank before `measure()`.
- Reset handling per "Reset handling" under Critical API Rules —
  the decision is per-Click wiring plus the datasheet, not a class
  default.
- Trigger: an inertial part almost always has a DRDY/ODR output —
  DRDY is the default per "Trigger handling"; run the FIFO
  wire-time budget for the datapath.
- A fixed-rate part (internal rate with no ODR divider): declare
  `drdy_base_hz=<sync Hz>` on the decorator and `sample_rate`
  values that are exact divisors of the sync, capped at what the
  read burst sustains end-to-end (per-transaction platform
  overhead plus staging sleeps — the wire-time budget's overhead
  term, decisive for per-word framed protocols) — the drdy loop
  is paced by dividing the sync at the source, so rates deliver
  exactly.

## Self-checks

- Param names match the canon table; no `range` / `bandwidth` /
  `fs` synonyms.
- A framed part declares every integrity field the protocol defines
  (`crc=` and `status_ok=` on the FRAME) — the compiled measure
  section then carries one OP_CRC8 per harvested word.
- Full-scale / filter fields on read-modify-write registers are
  runtime params (`write_modify` with `param=`, whole-field clear)
  — compile-time keys only for genuinely value-independent writes.
- `WHO_AM_I_VALUES` provenance stated: listed values, or a
  derived set with its corroborating source named; no `0x00` /
  `0xFF` entries without justification.
- Every present quantity's full-scale table consumed whole (count
  the rows).
- Scales recomputed from the sensitivity table, never the feature
  summary; `temp` lands in kelvin. A runtime full-scale links
  `scale_param` (base × range label; fold any constant
  label-to-true-range factor into the base).
- Pipelined FRAME reads carry the response-staging settle: one
  internal update period with 2× margin (`inter_frame_sleep_us`),
  never a defaulted whole millisecond.
- Fixed-sync parts: `drdy_base_hz` declared; every `sample_rate`
  value divides the sync exactly.
- SPI clock at the board-validated rate, the die ceiling only in
  the comment.
- Axis fields match the data-register order and the datasheet's
  byte order (big vs little endian).
- Die temperature published or its exclusion stated.
