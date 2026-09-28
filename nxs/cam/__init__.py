# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""Camera and GMSL SerDes control (`nxs cam`): the sequence engine, the socket
contracts, the plan container, pack discovery, diagnostics, and the CLI verbs.
Hardware knowledge lives in descriptor packs discovered at runtime."""

from .frame_source import Frame, frames  # noqa: E402

__all__ = ["Frame", "frames"]
