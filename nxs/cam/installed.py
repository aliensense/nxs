# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The installed personalities as a descriptor source. Every camera image
under the personality store carries its sensor's descriptor trailer; the
pack adopts those descriptors so a host that holds no sensor source, the
robot with the descriptor pack and the personalities asset, still knows
every head its overlays and verbs may meet."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List

from nxs.image import ImageKind, deserialize, peek_format, trailer_bytes
from nxs.suite import personality_dirs

from .descriptors import Descriptor

IMAGE_SUFFIX = ".nxs"


def installed_images() -> List[Path]:
    """Every `<store>/<name>.nxs`, the first store to carry a name winning.
    The release store is read only when its manifest names this release
    (`personality_cli.store_refusal`): a store of another release raises
    `StoreError` naming the asset to install."""
    from nxs.personality_cli import StoreError, store_dir, store_refusal

    seen = set()
    found = []
    # The store (`$NXS_PERSONALITIES` when set) first, then every fixed
    # location once.
    stores = list(dict.fromkeys([store_dir(), *personality_dirs()]))
    store = Path(store_dir()).resolve()
    for directory in stores:
        root = Path(directory)
        if not root.is_dir():
            continue
        if root.resolve() == store:
            refusal = store_refusal(str(root))
            if refusal:
                raise StoreError(refusal)
        for path in sorted(root.glob(f"*{IMAGE_SUFFIX}")):
            if path.stem in seen:
                continue
            seen.add(path.stem)
            found.append(path)
    return found


def installed_descriptors() -> List[Descriptor]:
    """The descriptor of every installed camera image. A driver image is
    passed over; an image this tool cannot read is named on stderr and
    passed over, so one stray file never stops the camera verbs."""
    from nxs.personality import records

    out = []
    for path in installed_images():
        try:
            img = path.read_bytes()
            if peek_format(img)[2] != ImageKind.CAMERA:
                continue
            compiled = deserialize(img)
            out.append(records.descriptor_from_trailer(trailer_bytes(compiled.trailer)))
        except (OSError, ValueError, records.RecordError) as exc:
            print(f"nxs: {path}: not a readable camera personality ({exc})",
                  file=sys.stderr)
    return out
