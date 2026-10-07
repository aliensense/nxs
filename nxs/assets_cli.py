# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""`nxs assets install`: the release's assets (the sealed personality
images, the pod firmware, the capture-stack tuning) under /opt/aliensense,
from the release page for this tool's version or from a downloaded file."""

from __future__ import annotations

import argparse
import os
import re
import sys
import tarfile
import tempfile
import urllib.request
from typing import Optional

from nxs.host import root
from nxs.suite import PERSONALITY_DIR

ASSETS_ROOT = os.path.dirname(PERSONALITY_DIR)
RELEASE_URL = "https://github.com/aliensense/nxs/releases/download/{tag}/nxs-assets-{version}.tar.gz"
_VERSION = re.compile(r"^(\d+\.\d+\.\d+)(?:rc(\d+))?$")
LICENSE_MEMBER = "LICENSE"
MANIFEST_MEMBER = "personalities/manifest.yaml"


def _tool_version() -> str:
    from nxs import __version__

    return str(__version__).split("+", 1)[0].lstrip("v")


def tool_build() -> Optional[str]:
    """The build this wheel was packed from (`git describe` at the release
    build, `nxs/_build_info.py`); None in a source checkout, which carries
    no such record."""
    try:
        from nxs._build_info import BUILD_GIT_VERSION
    except ImportError:
        return None
    return BUILD_GIT_VERSION or None


def other_build(found: Optional[str]) -> Optional[str]:
    """The refusal's fact when an assets manifest names a build other than
    this wheel's: the wheel runs the hub images and the hub its own build
    compiled, so assets of another build of the same version disagree with
    it. None when they agree or either side names none."""
    own = tool_build()
    if not found or not own or found == own:
        return None
    return f"the assets are build {found}, this nxs is build {own}"


def _manifest_build(tar: tarfile.TarFile) -> Optional[str]:
    """The build an assets tarball's manifest names, if any."""
    import yaml

    try:
        member = tar.extractfile(MANIFEST_MEMBER)
        doc = yaml.safe_load(member.read()) if member is not None else None
    except (KeyError, yaml.YAMLError):
        return None
    return str(doc["build"]) if isinstance(doc, dict) and doc.get("build") else None


def _manifest_version(tar: tarfile.TarFile) -> Optional[str]:
    try:
        member = tar.extractfile(MANIFEST_MEMBER)
    except KeyError:
        return None
    if member is None:
        return None
    import yaml

    doc = yaml.safe_load(member.read()) or {}
    release = doc.get("release") or doc.get("version")
    return str(release).lstrip("v") if release else None


def _license_text(tar: tarfile.TarFile) -> Optional[str]:
    try:
        member = tar.extractfile(LICENSE_MEMBER)
    except KeyError:
        return None
    return member.read().decode("utf-8", "replace") if member is not None else None


def _accepted(text: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    print(text)
    if not sys.stdin.isatty():
        print("nxs assets: the license needs your yes; run with --accept-license",
              file=sys.stderr)
        return False
    answer = input("Accept the license? [y/N] ").strip().lower()
    return answer in ("y", "yes")


def install(path: str, *, accept_license: bool = False, log=print) -> int:
    """Install the assets tarball at `path`: the release must be this tool's,
    the license accepted once, then the tree lands under /opt/aliensense."""
    tool = _tool_version()
    with tarfile.open(path) as tar:
        for member in tar.getmembers():
            if member.name.startswith("/") or ".." in member.name.split("/"):
                print(f"nxs assets: {path} carries an unsafe path {member.name}",
                      file=sys.stderr)
                return 1
        release = _manifest_version(tar)
        if release is None:
            # The same-version gate needs a version to compare: an archive
            # without its manifest is not a release's and is not extracted.
            print(f"nxs assets: {os.path.basename(path)} carries no release version "
                  f"({MANIFEST_MEMBER})", file=sys.stderr)
            print(f"  - install nxs-assets-{tool}.tar.gz", file=sys.stderr)
            return 1
        if release != tool:
            print(f"assets {release} in {os.path.basename(path)}, tool {tool}",
                  file=sys.stderr)
            print(f"  - install nxs-assets-{tool}.tar.gz", file=sys.stderr)
            return 1
        if (fact := other_build(_manifest_build(tar))) is not None:
            print(f"nxs assets: {fact} ({os.path.basename(path)})", file=sys.stderr)
            print(f"  - install the nxs-assets-{tool}.tar.gz built with this nxs",
                  file=sys.stderr)
            return 1
        text = _license_text(tar)
        marker = os.path.join(ASSETS_ROOT, ".license-accepted")
        if text and not os.path.exists(marker) and not _accepted(text, accept_license):
            return 1
    root.run(["install", "-d", "-m", "755", ASSETS_ROOT])
    # Root's own, whatever owner the archive names: tar keeps the archive's
    # owners when it runs as root, and a directory an earlier install left
    # under another owner keeps it through an extraction.
    root.run(["tar", "--no-same-owner", "-xzf", path, "-C", ASSETS_ROOT])
    root.run(["chown", "-R", "root:root", ASSETS_ROOT])
    if text:
        root.write_text(marker, f"accepted for {tool}\n")
    log(f"installed the assets {release or tool} under {ASSETS_ROOT}")
    return 0


def release_tag(version: str) -> str:
    """The tag a tool version was cut from: 1.1.0 is v1.1.0, 1.1.0rc1 is
    v1.1.0-rc1; ValueError for anything else."""
    m = _VERSION.match(version)
    if m is None:
        raise ValueError(f"not a release version: {version}")
    return f"v{m.group(1)}" + (f"-rc{m.group(2)}" if m.group(2) else "")


def fetch(version: str, into: str, log=print) -> str:
    """Download the release's assets tarball into `into`; returns its path.
    ValueError when the version is not a release's."""
    url = RELEASE_URL.format(tag=release_tag(version), version=version)
    target = os.path.join(into, f"nxs-assets-{version}.tar.gz")
    log(f"fetching {url}")
    urllib.request.urlretrieve(url, target)
    return target


def cmd_assets(args: argparse.Namespace) -> int:
    if args.assets_cmd != "install":
        return 1
    path = args.file
    if path is None:
        with tempfile.TemporaryDirectory() as tmp:
            try:
                path = fetch(_tool_version(), tmp)
            except (OSError, ValueError) as exc:
                print(f"nxs assets: cannot fetch the assets for {_tool_version()}: {exc}",
                      file=sys.stderr)
                print("  - download nxs-assets-<version>.tar.gz from the release page and "
                      "run nxs assets install <file>", file=sys.stderr)
                return 1
            return _install_or_say(path, args)
    if not os.path.isfile(path):
        print(f"nxs assets: no such file {path}", file=sys.stderr)
        return 1
    return _install_or_say(path, args)


def _install_or_say(path: str, args) -> int:
    try:
        return install(path, accept_license=bool(getattr(args, "accept_license", False)))
    except root.RootRefused as exc:
        print(f"nxs assets: {exc}", file=sys.stderr)
        return 1
    except (tarfile.TarError, OSError) as exc:
        print(f"nxs assets: {path}: {exc}", file=sys.stderr)
        return 1



def add_assets_parser(sub) -> None:
    p = sub.add_parser("assets", help="the release's assets under /opt/aliensense")
    ps = p.add_subparsers(dest="assets_cmd", required=True)
    q = ps.add_parser("install", help="install the assets for this tool's version "
                                      "(fetched from the release page, or a file)")
    q.add_argument("file", nargs="?", default=None,
                   help="a downloaded nxs-assets-<version>.tar.gz")
    q.add_argument("--accept-license", action="store_true",
                   help="accept the assets' license without the prompt")
