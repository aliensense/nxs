# GNSS receiver class

Conventions for positioning receivers — GNSS modules, including
RTK-grade parts, streaming position/velocity/time epochs over UART.
Wire mechanics (the StreamDriver method split, probe patterns,
framing helpers, runtime checksums) live in the skill's stream
sections; this file fixes the class contract: which protocol to
speak, which fields to publish, and which datum the altitude
carries.

## Protocol policy — binary by default

A receiver that documents a binary protocol (a framed sync +
class/ID + payload + checksum format — UBX-style, SBF-style) ships
it as the DEFAULT, publishing typed fields at their record offsets.
A part that also speaks NMEA ships BOTH protocols behind the
`protocol` config key, whose values are exactly `binary` (the
default) and `nmea` — generic, fleet-uniform names, never the
vendor protocol's own name: a host scripts one vocabulary across
every receiver. The NMEA side is the secondary variant (`when=`). The variant is not generator-discretionary — omitting it
on a part that speaks it is a capability reduction, however much
better the binary output is. Binary is never demoted: an NMEA
sentence is an opaque string the host must parse, it carries less
precision than the binary record, and it feeds no geodetic SI
subject.

NMEA-only is legal solely for parts that genuinely speak nothing
else; the docstring's Outputs line then states that limitation.

**The protocol specification is a gating document.** Binary framing
and record layouts commonly live in an interface description
separate from the hardware datasheet. That document is normative
input, exactly like the datasheet: when it cannot be retrieved, the
typed output cannot be written — stop with the gap card naming the
exact document (the "gating fact you cannot read" rule). Never fall
back to NMEA-only for a part whose binary protocol exists but whose
specification you could not read.

## Output vector

Map the full PVT epoch. Geodetic SI semantics (these feed the typed
geodetic subject):

| Field | Unit | Note |
|---|---|---|
| `latitude`, `longitude` | rad | fold the record scale — a 1e-7-degree integer scales by `1e-7 * pi/180` |
| `altitude` | m | height above MSL — the datum rule below |
| `vel_north`, `vel_east`, `vel_down` | m/s | NED frame; mm/s records scale by 0.001 |
| `pos_h_acc`, `pos_v_acc` | m | position accuracy estimates |
| `vel_s_acc` | m/s | speed accuracy estimate |

Generic fields (RawSample + descriptors, no typed subject) — ship
them all, under these exact names (the canon rule covers fields as
much as params: the same concept never gets a synonym across the
fleet); every epoch publishes, consumers gate on quality:

- `fix_type`, `num_sv` as uint8
- `pdop` — position DOP, scaled to its dimensionless value
- `speed` (ground speed, m/s — a canonical scalar semantic)
- `heading` (rad — fold the record's degree scale)
- `alt_ellipsoid` (m — the datum rule below)

Publish at the record's native precision: position integers carry
centimetre-order resolution, and RTK-grade records may add
high-resolution extension fields — map the highest-precision
representation the record carries.

## The altitude datum — MSL, never ellipsoid

The geodetic subject that the `altitude` semantic feeds is
`reg.udral.physics.kinematics.geodetic.PointStateVarTs`, whose
position type defines the altitude field as:

> Distance between the local mean sea level (MSL) and the focal
> point of the antenna. Positive altitude above the MSL.

Receivers report TWO heights in the same epoch: height above the
WGS84 ellipsoid (the constellation-native solution) and height
above mean sea level (the receiver applies its geoid model). Both
are metres — the unit cannot disambiguate them — and they differ by
the local geoid separation (tens of metres). The field named
`altitude` carries **MSL**. The ellipsoidal height ships alongside
as the generic field `alt_ellipsoid` (abbreviated to stay inside
the 16-character field-name limit — a longer name truncates in the
I²C descriptor window); dropping it when the record carries it is a
capability reduction.

## Probe and configuration

- Probe is an ack-handshake — send a fixed config-class command,
  expect the structured acknowledgement (see "Stream-driver probe
  patterns"). Receivers have no WHO_AM_I.
- The epoch rate is the `rate` param (Hz), patched into the
  rate-command frame with the checksum recomputed at runtime (see
  "Runtime-tunable parameters with `compute_checksum`"). On a UART
  part `sample_rate` is the driver-pinned VM poll cadence, so the
  receiver's own rate keeps the `rate` name.
- Receivers with layered configuration (volatile / battery-backed /
  flash) target the volatile layer: the driver reconfigures on
  every load, and burning non-volatile layers wears the part and
  leaks state into later sessions.
- Enable exactly the record the measure loop frames and disable
  default chatter (broadcast NMEA) in the binary variant, so the
  sync search never wades through foreign sentences.

## Self-checks

- **Datum quotes**: quote the interface-description line defining
  the record field mapped to `altitude` — it must reference mean
  sea level (or the geoid). Quote the line for
  `alt_ellipsoid` — it must reference the ellipsoid. A
  mapping that cannot produce both quotes is wrong or unverified.
- Every field of the chosen record is mapped or its exclusion
  stated; scale folds verified per field (deg→rad, mm→m,
  1e-5-degree headings).
- The `protocol` key offers exactly the values `binary` (default)
  and `nmea`, and the NMEA variant EXISTS whenever the part speaks
  NMEA, framing single sentences with the delimiter pattern.
- Generic field names match the pinned set (`fix_type`, `num_sv`,
  `pdop`, `speed`, `heading`, `alt_ellipsoid`) — no synonyms.
- The rate command's checksum is computed at runtime, never baked.
