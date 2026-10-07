"""
AS5047D personality for NXS VM.

ams AS5047D 14-bit magnetic rotary encoder; datasheet DS000394.
Bus: SPI (mikroBUS CS), mode 1, 10 MHz max; 16-bit parity-framed words,
    response pipelined one frame behind the command.
Config: sample_rate 10|50|100|250|500|1000 Hz (500), poll cadence only.
Outputs: angle (rad), the DAEC-compensated ANGLECOM register (0x3FFF).
Pins: none (no data-ready output; ABI/UVW/PWM outputs unused).
mikroE Click: https://www.mikroe.com/magnetic-rotary-4-click
"""

from nxs import RegisterClickPersonality, Sample, SpiProfile

_TWO_PI = 6.283185307179586


def _read_command(addr):
    """Read command word (datasheet command frame): even parity at bit 15
    over bits 14:0, R/W=1 at bit 14, 14-bit address in bits 13:0."""
    word = (1 << 14) | (addr & 0x3FFF)
    return word | ((bin(word).count("1") & 1) << 15)


class As5047d(RegisterClickPersonality):
    """ams AS5047D 14-bit magnetic rotary encoder, parity-framed SPI."""

    # ── SPI only ────────────────────────────────────────
    # SPI is the only register interface; ABI/UVW/PWM are output-only pins.
    BUSES = ('spi',)

    # No data-ready output; the angle is polled over SPI.
    PINS = {}

    # ── Communication profile (clock + mode) ────────────
    # SCLK max 10 MHz, mode 1 (CPOL=0, CPHA=1): DS SPI Interface Timing table.
    SPI_PROFILE = SpiProfile(max_hz=10_000_000, mode=1)

    # ── Identity ────────────────────────────────────────
    # No identity register; every response is error-flag and parity gated.
    WHO_AM_I_VALUES = []
    WHO_AM_I_SKIP_REASON = "no identity register; SPI reads are error-flag + parity gated"

    # ── Command words (compile-time constants) ──────────
    # ANGLECOM (0x3FFF): DAEC-compensated absolute angle read command.
    CMD_ANGLECOM = _read_command(0x3FFF)

    def configure(self, config):
        # No rate register: sample_rate patches the poll interval only.
        self.declare_params_from_descriptor()

        # Full turn = 2^14 counts: the divisor is the count space 16384,
        # not the maximum code 16383.
        self.set_output([
            {'name': 'angle', 'type': 'uint16',
             'scale': _TWO_PI / 16384.0},
        ])
        self.set_sample_size(2)   # one 14-bit angle in a uint16

    @RegisterClickPersonality.measure_loop(trigger="poll", sample_rate=500)
    def measure(self):
        self.xfer(self.CMD_ANGLECOM)      # prime: request ANGLECOM
        a = self.xfer(self.CMD_ANGLECOM)  # response = ANGLECOM from prior frame
        if a & 0x4000:                     # EF error flag set: drop the sample
            return None
        t = a >> 8                         # even-parity fold: xor the halves
        p = a ^ t                          # down until bit 0 holds the parity
        t = p >> 4
        p = p ^ t
        t = p >> 2
        p = p ^ t
        t = p >> 1
        p = p ^ t
        if p & 1:                          # parity violation: drop the sample
            return None
        angle = a & 0x3FFF                 # 14-bit absolute angle
        return Sample(angle=angle)


# Datasheet worked example: the ANGLECOM (0x3FFF) read command is 0xFFFF.
assert As5047d.CMD_ANGLECOM == 0xFFFF
