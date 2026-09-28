"""What a compile yields: the image kind, the compiled driver, its report, and the measure loop's `Sample`."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from nxs._generated_constants import NxsDriverImage
from nxs.dsl.fields import MAX_PARAMS, MAX_PATCH_SITES, MAX_TRAILER_SIZE, ParamDescriptor, PatchEntry, VM_MAX_PROGRAM_SIZE


ImageKind = NxsDriverImage.ImageKind

# ── Compiled output ─────────────────────────────────────────

@dataclass
class CompiledDriver:
    bytecode: bytes
    sample_size: int
    name: str
    config: dict = field(default_factory=dict)
    output_fields: List[dict] = field(default_factory=list)
    params: List[ParamDescriptor] = field(default_factory=list)
    patch_map: List[PatchEntry] = field(default_factory=list)
    # Probe metadata from the driver's class attributes (`WHO_AM_I_REG`,
    # `WHO_AM_I_VALUES`, `I2C_ADDRS`); empty for stream drivers.
    who_am_i_reg: int = 0
    who_am_i_values: List[int] = field(default_factory=list)
    i2c_addrs: List[int] = field(default_factory=list)
    # Justification for `WHO_AM_I_VALUES = []`, from `WHO_AM_I_SKIP_REASON`;
    # required whenever the values are empty.
    who_am_i_skip_reason: Optional[str] = None
    # Bus-config trailer: one register-access profile per supported bus.
    # None means no trailer (a stream driver with no UART profile).
    bus_config: Optional[list] = None
    # Image kind (`ImageKind.DRIVER` measures; `CAMERA` runs once under the
    # host's token) and the header flag byte.
    kind: int = ImageKind.DRIVER
    flags: int = 0
    # A sealed section reads back opaque: the CTR nonce and, in `bytecode`,
    # the ciphertext; the required minor is then carried, not recomputed.
    seal_nonce: bytes = b""
    required_minor: Optional[int] = None
    # Descriptor trailer records as `(type, bytes)`, supplied by the caller
    # (the datasheet YAML's modes, controls, laws, capture facts).
    trailer: List[Tuple[int, bytes]] = field(default_factory=list)
    # Bytecode budget lines as `(label, bytes)`: probe, the shared configure
    # sections, each select() block and its dispatch, the halt.
    budget: List[Tuple[str, int]] = field(default_factory=list)
    # Bytes of the probe program: the offset where the traced `probe()` ends
    # and `configure()` begins. Firmware compares the program counter
    # against it to name the phase a run is in.
    probe_len: int = 0

    @property
    def sealed(self) -> bool:
        return bool(self.flags & NxsDriverImage.IMAGE_FLAG_SEALED)

    @sealed.setter
    def sealed(self, value: bool) -> None:
        if value:
            self.flags |= NxsDriverImage.IMAGE_FLAG_SEALED
        else:
            self.flags &= ~NxsDriverImage.IMAGE_FLAG_SEALED & 0xFF


def compile_report(compiled: CompiledDriver) -> str:
    """The bytecode budget of a compiled personality as text: one line per
    budget entry, the totals, and the caps. A camera personality's report
    names each select() block and its dispatch overhead."""
    lines = []
    for label, size in compiled.budget:
        lines.append(f"  {label:<40} {size:>5} B")
    used = len(compiled.bytecode)
    lines.append(f"  {'bytecode':<40} {used:>5} B of {VM_MAX_PROGRAM_SIZE} "
                 f"({100 * used // VM_MAX_PROGRAM_SIZE}%)")
    sites = {}
    for entry in compiled.patch_map:
        sites[entry.param_name] = sites.get(entry.param_name, 0) + 1
    lines.append(f"  {'params':<40} {len(compiled.params):>5} of {MAX_PARAMS}"
                 + (", patch sites " + ", ".join(
                     f"{name} {n}/{MAX_PATCH_SITES}" for name, n in sites.items())
                    if sites else ""))
    trailer = 1 + sum(3 + len(payload) for _, payload in compiled.trailer)
    lines.append(f"  {'descriptor trailer':<40} {trailer:>5} B of {MAX_TRAILER_SIZE}")
    return "\n".join(lines)


class Sample:
    """Marker returned by measure() to signal 'commit sample buffer'."""

    def __init__(self, raw=None):
        self.raw = raw


