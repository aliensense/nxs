# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0

"""The few writes and commands under root the tool needs on a host: each
goes through `sudo` when the tool runs as a person (a terminal is asked
for the password once), and directly when it already runs as root."""

from __future__ import annotations

import os
import subprocess
import sys
from typing import List, Optional


class RootRefused(RuntimeError):
    """A root step failed: the reason names the target."""


def is_root() -> bool:
    return os.geteuid() == 0


def as_root(argv: List[str]) -> List[str]:
    """`argv` under root: itself when running as root, else through sudo,
    non-interactive when no terminal can be asked."""
    if is_root():
        return list(argv)
    return ["sudo"] + ([] if sys.stdin.isatty() else ["-n"]) + list(argv)


def read_text(target: str) -> Optional[str]:
    """The file's text, or None when it does not exist or cannot be read."""
    try:
        with open(target, encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return None


def write_text(target: str, text: str) -> None:
    """Write `text` to a path root owns; RootRefused when it cannot be
    written. Unchanged content is not rewritten."""
    if read_text(target) == text:
        return
    if is_root():
        try:
            os.makedirs(os.path.dirname(target) or ".", exist_ok=True)
            with open(target, "w", encoding="utf-8") as fh:
                fh.write(text)
        except OSError as exc:
            raise RootRefused(f"cannot write {target}: {exc}") from exc
        return
    try:
        proc = subprocess.run(as_root(["tee", target]), input=text, text=True,
                              capture_output=True)
    except OSError as exc:
        raise RootRefused(f"cannot write {target}: {exc}") from exc
    if proc.returncode != 0:
        why = (proc.stderr or "").strip() or "sudo refused"
        raise RootRefused(f"cannot write {target}: {why}")


def run(argv: List[str], check: bool = True) -> subprocess.CompletedProcess:
    """Run `argv` under root; RootRefused on a failure when checked."""
    cmd = as_root(argv)
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True)
    except OSError as exc:
        raise RootRefused(f"{' '.join(argv[:2])}: {exc}") from exc
    if check and proc.returncode != 0:
        why = (proc.stderr or "").strip() or f"exit {proc.returncode}"
        raise RootRefused(f"{' '.join(argv[:2])}: {why}")
    return proc
