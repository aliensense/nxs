"""
AS5047D driver for NXS VM.

ams (ams OSRAM) AS5047D 14-bit on-axis magnetic rotary position encoder.
Absolute angle over a full 360 deg turn, read over a 4-wire SPI slave
interface. The chip exposes ABI (incremental), UVW (commutation) and PWM
angle outputs on dedicated pins as well, but this driver reads the absolute
angle register over SPI — the ABI/UVW/PWM output pins are not part of the
NXS datapath.

Wire protocol (parity-framed SPI, no register-addr/data split):
    Every access is ONE 16-bit word, MSB first, SPI mode 1 (CPOL=0, CPHA=1):
        bit 15    : PARC/PARD — EVEN parity over bits 14:0
        bit 14    : R/W for a command (1 = read, 0 = write);
                    EF (error flag) in a data response
        bits 13:0 : register address (command) or 14-bit data (response)
    Reads are PIPELINED: the command word is clocked in frame K and the
    addressed register's contents come back on MISO in frame K+1. So the
    angle read primes with a command word, then clocks a second word to
    shift the response out (see measure()).
    The command words are compile-time constants (address + fixed R/W +
    computed parity), so they are built in plain Python at class-definition
    time and clocked verbatim with self.xfer() — this is the guide's
    "Parity-Framed SPI (Literal Words)" pattern, not a FRAME/CRC part.

Volatile register map (from the ams AS5047D datasheet, confirmed against the
mikroE Magnetic Rotary 4 Click C driver register defines):
    NOP       0x0000    no operation
    ERRFL     0x0001    error flags (framing / invalid cmd / parity)
    PROG      0x0003    OTP programming control
    DIAAGC    0x3FFC    diagnostics + automatic gain control value
    MAG       0x3FFD    CORDIC magnitude (relative field strength)
    ANGLEUNC  0x3FFE    measured angle, NO dynamic angle error compensation
    ANGLECOM  0x3FFF    measured angle WITH dynamic angle error compensation
    ZPOSM/L   0x0016/0x0017   zero-position offset (OTP)
    SETTINGS1 0x0018    direction / DAEC disable / ABI-UVW-PWM output config
    SETTINGS2 0x0019    UVW pole pairs / hysteresis / ABI resolution

This driver reads ANGLECOM (0x3FFF): the DAEC-compensated angle is the
recommended output for a rotating target (near-zero-latency compensation of
the propagation delay), and it is available at power-on with no configuration.

Config keys:
    sample_rate   poll cadence in Hz — one of {10, 50, 100, 250, 500, 1000},
                  default 500. No rate register exists (the die refreshes the
                  angle continuously); sample_rate patches the poll loop's
                  sleep interval, so `nxs set sample_rate N` retunes it with
                  no re-upload.

Capability-parity notes (settings/measurands deliberately not exposed):
  * Rotation direction (SETTINGS1.Dir), zero position (ZPOSM/L), and the
    ABI/UVW/PWM output configuration (SETTINGS1/SETTINGS2) are OTP-backed and
    govern the incremental/commutation/PWM OUTPUT pins, which the SPI read
    path does not use. They are left at their power-on default; a non-default
    rotation sense is applied host-side. A literal-xfer part also has no
    param= patch site, so these could not be runtime params in any case.
  * ANGLEUNC (0x3FFE) is the same measurand as ANGLECOM without DAEC — not a
    second quantity; ANGLECOM is read.
  * The MAG magnitude and DIAAGC AGC/flag values are non-SI diagnostic fields;
    they are not published (the integrity-gated read covers the primary angle
    measurand). Corrupt reads are dropped via the response error flag + parity.

Datasheet: ams AS5047D, DS000394 (SPI Interface Timing table: SCLK f_max =
           10 MHz; family value shared with AS5047P/AS5147/AS5048).
mikroE Click: https://www.mikroe.com/magnetic-rotary-4-click (AS5047D, 3.3V, SPI)
"""

from nxs import RegisterDriver, Sample, SpiProfile

_TWO_PI = 6.283185307179586


def _read_command(addr):
    """AS5047D read command word: R/W=1 at bit 14, 14-bit address in bits
    13:0, and bit 15 = EVEN parity over bits 14:0. Plain Python, evaluated at
    class definition and cross-checked below against the datasheet's worked
    ANGLECOM read command (0xFFFF)."""
    word = (1 << 14) | (addr & 0x3FFF)
    return word | ((bin(word).count("1") & 1) << 15)


class As5047d(RegisterDriver):
    """ams AS5047D — 14-bit magnetic rotary encoder, parity-framed SPI."""

    # ── SPI only ────────────────────────────────────────
    # Literal wire words have no defined I²C behaviour; the AS5047D SPI
    # interface is the only register transport. (ABI/UVW/PWM are output-only.)
    BUSES = ('spi',)

    # No GPIO signals: the Magnetic Rotary 4 Click reads angle over SPI
    # (CS/SCK/MISO/MOSI handled by the bus); no data-ready line is wired.
    PINS = {}

    # ── Communication profile (clock + mode) ────────────
    # SCLK max 10 MHz and SPI mode 1 (CPOL=0, CPHA=1) from the AS5047D
    # datasheet SPI Interface Timing table. Above the 1 MHz firmware baseline,
    # so declared to expose the part's real speed.
    SPI_PROFILE = SpiProfile(max_hz=10_000_000, mode=1)

    # ── Identity ────────────────────────────────────────
    # No WHO_AM_I / chip-ID register on this part. Every SPI response is
    # error-flag- and parity-gated, so the first gated angle read is the
    # integrity anchor.
    WHO_AM_I_VALUES = []
    WHO_AM_I_SKIP_REASON = "no identity register; SPI reads are error-flag + parity gated"

    # ── Command words (compile-time constants) ──────────
    # ANGLECOM (0x3FFF): DAEC-compensated absolute angle read command.
    CMD_ANGLECOM = _read_command(0x3FFF)

    def configure(self, config):
        # Hardcoded-poll driver: no rate register exists, so declare
        # sample_rate unconditionally — the poll loop always has a SLEEP_MS
        # for the compiler to patch. No param= tag (no register write).
        self.declare_param("sample_rate", values=[10, 50, 100, 250, 500, 1000],
                           default=500, unit="Hz")

        # 14-bit angle: a full turn spans the count SPACE 2^14 = 16384, so the
        # scale divisor is 16384 (never the max code 16383). Published in
        # radians (canonical SI for `angle`; unit inherited from the semantic).
        self.set_output([
            {'name': 'angle', 'type': 'uint16',
             'scale': _TWO_PI / 16384.0},
        ])
        self.set_sample_size(2)   # one 14-bit angle in a uint16

    @RegisterDriver.measure_loop(trigger="poll", sample_rate=500)
    def measure(self):
        self.xfer(self.CMD_ANGLECOM)      # prime: request ANGLECOM
        a = self.xfer(self.CMD_ANGLECOM)  # response = ANGLECOM from prior frame
        if a & 0x4000:                     # EF error flag set → drop the sample
            return None
        t = a >> 8                         # even-parity fold: xor the halves
        p = a ^ t                          # down until bit 0 holds the parity
        t = p >> 4
        p = p ^ t
        t = p >> 2
        p = p ^ t
        t = p >> 1
        p = p ^ t
        if p & 1:                          # parity violation → drop the sample
            return None
        angle = a & 0x3FFF                 # 14-bit absolute angle
        return Sample(angle=angle)


# Cross-check the computed command word against the datasheet's worked example:
# reading ANGLECOM (0x3FFF) with R/W=1 and even parity is the word 0xFFFF.
assert As5047d.CMD_ANGLECOM == 0xFFFF
