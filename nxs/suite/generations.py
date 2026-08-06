"""Manifest generations: snapshot and rollback for the suite.

Every fully-converged switch records the manifest as a generation in a
tool-managed git repository under the state directory (the manifest is
copied in and committed — the operator's config directory is never
turned into a repo). `snapshot` tags a generation as known-good,
`list-generations` shows the history, and `rollback` restores a
recorded manifest over the live file so the next apply converges the
suite back to it. The state file is deliberately not versioned: TOFU
serials are trust history, not intent.

Git is the substrate because a generation here is pure text intent —
there is no built artifact to store. A missing git binary degrades the
generation verbs with a clear error; apply itself never needs git.
"""
import os
import shutil
import subprocess

TRACKED_NAME = "manifest.yaml"


class GenerationsError(Exception):
    """A generations operation that could not run; message says why."""


def repo_dir(state_path: str) -> str:
    """The generations repository, beside the state file."""
    return os.path.join(os.path.dirname(state_path), "generations")


def record_generation(config_path: str, state_path: str, summary: str):
    """Commit the current manifest as a generation; returns the short
    sha, or None when the manifest is identical to the latest
    generation (converging twice is one intent, not two)."""
    repo = repo_dir(state_path)
    if not os.path.isdir(os.path.join(repo, ".git")):
        os.makedirs(repo, exist_ok=True)
        _git(repo, "init", "-q")
        _git(repo, "config", "user.name", "nxs suite")
        _git(repo, "config", "user.email", "nxs-suite@localhost")
    shutil.copyfile(config_path, os.path.join(repo, TRACKED_NAME))
    _git(repo, "add", TRACKED_NAME)
    rc, _ = _git(repo, "diff", "--cached", "--quiet", check=False)
    if rc == 0:
        return None
    _git(repo, "commit", "-q", "-m", summary)
    _, sha = _git(repo, "rev-parse", "--short", "HEAD")
    return sha.strip()


def snapshot(state_path: str, label: str) -> str:
    """Tag the latest generation as known-good; returns its short sha."""
    repo = _existing_repo(state_path)
    rc, out = _git(repo, "tag", label, check=False)
    if rc != 0:
        raise GenerationsError(f"cannot tag {label!r}: {out.strip()}")
    _, sha = _git(repo, "rev-parse", "--short", "HEAD")
    return sha.strip()


def list_generations(state_path: str):
    """`(sha, date, labels, summary)` per generation, newest first."""
    repo = _existing_repo(state_path)
    _, out = _git(repo, "log", "--format=%h%x1f%ad%x1f%D%x1f%s",
                  "--date=format:%Y-%m-%d %H:%M")
    rows = []
    for line in out.splitlines():
        sha, date, decorations, summary = line.split("\x1f")
        labels = ", ".join(d.strip().removeprefix("tag: ")
                           for d in decorations.split(",")
                           if "tag: " in d)
        rows.append((sha, date, labels, summary))
    return rows


def rollback(state_path: str, ref: str, config_path: str):
    """Restore the manifest recorded at `ref` (a label or sha) over the
    live file. The caller re-applies to converge the suite to it."""
    repo = _existing_repo(state_path)
    rc, out = _git(repo, "show", f"{ref}:{TRACKED_NAME}", check=False)
    if rc != 0:
        raise GenerationsError(
            f"no generation {ref!r} — see `nxs suite list-generations`")
    tmp = config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(out)
    os.replace(tmp, config_path)


def _existing_repo(state_path: str) -> str:
    repo = repo_dir(state_path)
    if not os.path.isdir(os.path.join(repo, ".git")):
        raise GenerationsError(
            "no generations recorded yet — they are written by the first "
            "fully-converged switch")
    return repo


def _git(repo: str, *args, check: bool = True):
    """Run git in the generations repo; returns `(rc, stdout)`."""
    try:
        proc = subprocess.run(["git", "-C", repo, *args],
                              capture_output=True, text=True)
    except FileNotFoundError:
        raise GenerationsError(
            "git is not installed — generations need it (apply itself "
            "does not)") from None
    if check and proc.returncode != 0:
        raise GenerationsError(
            f"git {' '.join(args)} failed: {proc.stderr.strip()}")
    if proc.returncode != 0:
        # Failures speak on stderr; check=False callers build their own
        # error message from the output, so hand them the reason.
        return proc.returncode, proc.stdout + proc.stderr
    return proc.returncode, proc.stdout
