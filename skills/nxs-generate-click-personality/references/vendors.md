# Vendor-family priors

Manufacturer-*family* conventions: recognition priors that say where a
family's traps usually hide and which datasheet sections settle them.
Every entry is a **prior to verify, never an authoritative value** —
facts enter a personality only from the datasheet (or vendor driver source)
in front of you at generation time. Part numbers are banned here; a
part-number fact is a cached datasheet that goes stale. When the part
in hand contradicts its family prior, the part wins; say so in a
one-line comment.

Use: after classing the part (Step 2), match its manufacturer and
interface style below and read the matching section before writing.

## TDK / InvenSense — consumer register-bus IMUs

- Conventional SPI register bus: read = `reg | 0x80`, no profile
  switches needed beyond the clock; I²C strap via an AD0-style pin.
- WHO_AM_I at a fixed register with a listed value — normal-strength
  identity. Verify against the register map, not the feature summary.
- PWR_MGMT-style reset/wake ordering: device reset bit, then a settle,
  then clock-select/wake before config writes — take the sequence and
  the settle from the reset section (2× the stated power-up max).
- ODR = internal rate / (1 + divider register); bandwidth via DLPF
  config fields. Both are runtime params on tagged writes.

## TDK / InvenSense — industrial / automotive CRC-framed SPI

- 32-bit worded frames (rw/addr/status/data/CRC) instead of a register
  bus: declare a `FRAME`, and verify the CRC feedback style against the
  datasheet's *worked CRC examples* — this family's CRC is often a
  chip-specific feedback variant a textbook CRC-8 will not reproduce.
- Declare the frame's return-status field too (`status_ok=` with the
  status truth table's success code — verify in the protocol chapter).
  Declared `crc`/`status_ok` make the compiler verify every harvested
  measure response on-device; the family ships these fields for
  integrity, and a personality that strips them publishes corrupted or
  unprepared ("data not ready") responses as plausible samples.
- Reads are pipelined (response in frame K+1). Sampled registers stage
  the response on the part's internal update tick — set the inter-frame
  settle to one update period with 2× margin (`inter_frame_sleep_us`),
  and expect *self-test tables* to show back-to-back frames that do NOT
  govern the data-read settle.
- Fixed internal rate, no ODR divider; the sync pin is the DRDY. Pace
  with `drdy_base_hz` + exact-divisor `sample_rate` values.
- Config registers mix settings with undocumented reserved factory
  bits: writes are `write_modify` (RMW), and configurable fields stay
  runtime params via `param=` with a whole-field clear.
- Bank-switched register space behind a fixed unlock word walk — copy
  the words verbatim from the unlock section and return to the data
  bank before `measure()`.

## NXP — combo accel/mag parts

- SPI framing deviates from the conventional bus: expect a two-byte
  address phase and an inverted R/W level — declare the profile
  switches from the bus-interface section, not by convention.
- Hybrid modes burst both quantities behind one address through a
  documented hybrid auto-increment — one read, one sample.

## ST — register-bus inertial/mag parts

- Multi-byte access needs the sub-address MSB set: profile
  `auto_inc='msb'`. Single-byte reads work without it, so the trap
  surfaces only on bursts.

## mCube-style two-die eCompass packages

- Two dies at two I²C addresses in one package: the strapped die is
  the primary, the fixed-address die the companion.
- Primary product codes are often factory-variable or mask-derived —
  weak identity. Hard-assert the companion's fixed WIA in the probe;
  corroborate any mask-derived value set against silicon or a vendor
  driver before trusting it.

## Sensirion-style command-protocol parts

- No register map: 16-bit commands, conversion delay, then a read
  with CRC-8 per 16-bit word. Per-precision conversion delays come
  from the timing table, and the command set replaces WHO_AM_I
  (serial-number command as the identity anchor when one exists).

## u-blox-style binary-protocol GNSS receivers

- See `gnss.md` — binary protocol mandatory when documented, checksum
  and framing from the interface description, MSL datum rule. This
  file adds nothing beyond the cross-reference.
