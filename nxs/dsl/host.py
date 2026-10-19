"""Host devices: a host's description as a block, compiled from its plugin's data, run by nothing."""

from __future__ import annotations

from typing import Tuple

from nxs.dsl.camera import CamPersonality
from nxs.dsl.compiled import ImageKind
from nxs.dsl.errors import CompileError
from nxs.dsl.fields import Port


class HostDevice(CamPersonality):
    """Base class for a host's description: the ports its connectors and
    buses offer and the rows a declaration sets on it, compiled to a
    HOST-kind image that nothing runs. The class carries no `probe()` or
    `configure()`. `compile()` takes the rows `declare_rows()` declares and
    emits the halt alone, as it does for a native head, so the image is its
    IDENTITY, PARAMS, PRESENTATION and PORTS records and stages no VM
    parameter. Its rows are `declare_row` rows: a run parameter is refused.
    `COMPATIBLE` is the type a declaration names the host by, `PRODUCT` the
    name the tool prints."""

    IMAGE_KIND = ImageKind.HOST
    #: A host masters its buses itself: its image binds none.
    BUSES: Tuple[str, ...] = ()
    #: The host's connectors and buses, the image's PORTS record.
    PORTS: Tuple[Port, ...] = ()
    #: The compatible the IDENTITY head carries, the type a declaration names.
    COMPATIBLE = ""
    #: The product the IDENTITY head carries, 8 characters at most.
    PRODUCT = ""

    def declare_rows(self) -> None:
        """Declare the host's rows with `declare_row()`; a host without a
        row declares nothing."""

    def _validate_kind(self) -> None:
        """Refuse a program: a host runs none."""
        cls = type(self)
        for name in ("probe", "configure"):
            if hasattr(cls, name):
                raise CompileError(
                    f"{cls.__name__}: a host runs no program, so {name}() has nothing to run "
                    f"on it; its rows come from declare_rows()")

    def _declare_host_rows(self) -> None:
        """The rows `declare_rows()` declares: the PARAMS record carries
        them and no bytecode reads them."""
        self.declare_rows()
        if self._params:
            raise CompileError(f"{type(self).__name__}: a host's parameters are its rows, and "
                               f"{', '.join(self._params)} is a run parameter; declare it with "
                               f"declare_row()")

    def _identity_names(self) -> Tuple[str, str]:
        """`(compatible, product)`: the class's `COMPATIBLE`, else its name,
        and its `PRODUCT`."""
        return self.COMPATIBLE or type(self).__name__, self.PRODUCT
