"""The bundled vendor DSDL must cover every type in the repo source.

`nxs/dsdl/` is a git-ignored mirror of the repo's `dsdl/aliensense/`,
refreshed by `scripts/vendor-dsdl.sh`. Because it is git-ignored it never shows
as a diff, and no CI job regenerates it — so it silently lags the source the
moment a vendor service is added without re-vendoring. A host compiled against a
stale bundle then speaks a dialect missing the firmware's newest service
(observed on the bench: `aliensense.nxs.GetDriverInfo` absent). This fails in
CI the instant that drift appears.
"""
import os

import pytest


def _rel_dsdl(root):
    return {
        os.path.relpath(os.path.join(d, f), root)
        for d, _, files in os.walk(root)
        for f in files
        if f.endswith(".dsdl")
    }


def test_bundled_aliensense_covers_repo_source():
    here = os.path.dirname(os.path.abspath(__file__))
    # tests → nxs → SDK root; the firmware repo is its parent only when
    # the tree is embedded as sdk/. A standalone checkout must not read
    # a dsdl/ that happens to sit beside it.
    sdk_root = os.path.abspath(os.path.join(here, os.pardir, os.pardir))
    if os.path.basename(sdk_root) != "sdk":
        pytest.skip("standalone SDK checkout (no firmware repo to compare against)")
    source = os.path.join(os.path.dirname(sdk_root), "dsdl", "aliensense")
    if not os.path.isdir(source):
        pytest.skip("repo source dsdl/aliensense absent (installed, not a checkout)")

    bundled = os.path.join(os.path.dirname(here), "dsdl", "aliensense")
    # Internal channel payload types (nxs/internal/) never appear on a
    # wire; vendor-dsdl.sh strips them from the customer bundle.
    internal_prefix = os.path.join("nxs", "internal") + os.sep
    missing = {f for f in _rel_dsdl(source) - _rel_dsdl(bundled)
               if not f.startswith(internal_prefix)}
    assert not missing, (
        "bundled DSDL is stale — run scripts/vendor-dsdl.sh. Missing: "
        + ", ".join(sorted(missing)))
