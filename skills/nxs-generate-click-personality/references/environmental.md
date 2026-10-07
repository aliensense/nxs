# Environmental class

Conventions for humidity / ambient-condition parts — RH+temperature
combos, gas/IAQ, IR-thermometry. Typically command-response I²C
with per-word CRC; that template carries the wire mechanics.

## Output vector

- `humidity` with authored unit `%RH` — a non-SI semantic: it ships
  in RawSample with its output-field descriptor but feeds no typed SI subject.
- `temp` in kelvin — fold the conversion formula's Celsius zero
  into the offset.
- Gas/IAQ channels are generic fields with authored units; a
  channel whose transfer function the datasheet does not give
  numerically is a gap-card stop, not a guess.
- Scales come from the datasheet's conversion formulas — linear
  `a + b * raw / 2^N` forms fold into scale+offset exactly.

## Canonical parameters

- `sample_rate` — poll-paced; the bus paces each conversion.
- Measurement precision/repeatability modes change the command
  opcode and its conversion time: expose them as the `precision`
  compile-time config key, one measure variant per mode, and size
  each variant's conversion sleep to its mode's max conversion time.
- An on-part heater is an operating mode under capability parity —
  expose it or state the exclusion.

## Self-checks

- Formulas transcribed from the datasheet's conversion section
  (never a vendor demo's approximation); offsets folded to kelvin.
- Each measure variant's conversion sleep ≥ its mode's max conversion
  time.
- A response framed with a per-word CRC is a gap card (see the
  command-response rules).
- Humidity carries `%RH`; no invented SI projection.
