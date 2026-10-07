# Magnetometer class

Conventions for parts streaming magnetic field — standalone
magnetometers, the mag die of an eCompass/combo package (read
together with `inertial.md`), and single-die hybrid accel+mag
parts.

## Canonical parameters

- `sample_rate` — the shared output rate.
- `mag_odr` — the mag die's own rate rows (Hz) when it rates
  independently of the part's pacing die.
- `mag_res` — resolution rows (bits) for parts with a switchable
  measurement width.
- `mag_fs` — full-scale rows when the range is selectable.

## Output vector and frame

- `mag_x/y/z` in tesla. Datasheet sensitivities come in µT or
  gauss per LSB — fold to tesla (`1 µT = 1e-6 T`,
  `1 gauss = 1e-4 T`).
- **One body frame per package.** When the mag axes are rotated or
  inverted relative to the package's accel axes, fold the correction
  into per-field scale signs so `mag_*` and `accel_*` share one
  frame — a combo publishing two frames ships a compass that cannot
  be fused. **The sign authority is the vendor driver's applied
  coefficient**, not the datasheet figure: a `-1.0` the vendor code
  multiplies onto a mag axis is wire-normative (it encodes the
  data-register sign relative to the accel frame). The orientation
  figure shows the geometry, but the register sign can be opposite
  what the arrows suggest, so when the two disagree the vendor code
  wins.

## Modes and freshness

- Prefer continuous mode when it covers the declared rates.
- Force/single-shot parts re-trigger each conversion from
  `measure()` and hold the previous bytes until the next
  completion (stale-hold): gate on the status flag, read when
  fresh, re-trigger, and state the freshness consequence (the mag
  lags when its conversion is slower than the pace) in a one-line
  comment at the read.
- Self-test and offset-calibration modes are operating modes under
  capability parity: expose them or state the exclusion.

## Identity and packaging

- Mags commonly carry a solid WIA/WHO_AM_I — anchor on it. In a
  two-die package the mag's fixed identity is often the strongest
  anchor of the whole part — as whichever check its position gives
  it: address topology picks the primary (the strapped die), never
  anchor strength (see "Companion I²C devices").
- When the primary die's code is factory-variable or mask-derived
  and a co-resident die exposes a fixed WIA, `probe()` hard-asserts
  the fixed companion value in addition to the primary read — the
  companion is the identity gate the sloppy product code cannot
  provide. The primary's `WHO_AM_I_VALUES` still resolve the strap.
- A second co-resident die at its own I²C address is a companion
  (`I2C_COMPANIONS` + `dev=`, one fused sample) — see "Companion
  I²C devices".
- A single-die hybrid (accel+mag behind one address) bursts both
  banks through its documented hybrid auto-increment mode when one
  exists — one read, one sample.

## Self-checks

- Tesla scales (never raw µT/gauss); axis sign matches the vendor
  driver's coefficient (the orientation figure is the geometry
  cross-check, not the sign authority).
- Two-die package: `probe()` hard-asserts the fixed companion WIA
  when the primary code is factory-variable or mask-derived.
- Mode choice stated; forced-mode freshness stated at the read.
- `mag_odr` / `mag_res` / `mag_fs` tables consumed whole.
