# Barometer class

Conventions for absolute-pressure parts. Most are command-protocol
parts with factory calibration — the wire mechanics live in the
command-response and on-device-compensation templates; this file
fixes the class contract.

## Output vector

- `pressure` in pascal — fold mbar/hPa (×100) or the part's native
  step into the scale/offset.
- `temperature` in kelvin — the compensation temperature is a real
  measurand; publish it (centi-°C records: scale 0.01, offset
  273.15).
- **No altitude on-device.** A baro publishes pressure SI; the
  sea-level reference P0 that yields metres is the consumer's
  mission state.

## Canonical parameters

- `sample_rate` — poll-paced; the conversion delays bound the
  ceiling, so declare only rates the full convert+read cycle fits.
- Oversampling is `osr`, values = the datasheet's ratio rows. When
  the convert opcodes differ per ratio the setting has no patch
  site — expose it as a compile-time config key — and the
  conversion sleeps are sized to the CHOSEN ratio's max conversion
  time, not the fastest ratio's.

## Calibration and compensation

- Factory coefficients (PROM/NVM) are read ONCE in `configure()`,
  bound to `self.<attr>` so they persist into `measure()`.
- The compensation polynomial is plain integer math on-device (the
  int64 path); transcribe the datasheet's formulas with their exact
  `2^k` shifts.
- **Cross-check against the worked example.** The datasheet's
  compensation section carries example values — coefficients, raw
  D-values, expected intermediates and outputs. Recompute every
  intermediate (the dT/OFF/SENS-style chain) with the transcribed
  formulas and verify the final pressure and temperature match
  before finishing: a shifted exponent produces plausible-but-wrong
  pressure on every unit.
- A documented PROM integrity check (CRC-4/CRC-16) decomposes into
  plain shifts/XORs in `configure()` — see "CRC is RISC".

## Self-checks

- Pressure lands in pascal, temperature in kelvin (offsets folded).
- The worked-example recomputation was done and matches.
- `osr` rows consumed whole; conversion sleeps cite the chosen
  ratio's max time.
- No altitude field.
