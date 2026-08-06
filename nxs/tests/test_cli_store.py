"""Regression: `store save <slot>` takes a positional slot, matching
`store rm <slot>`.

The slot was once a `--slot` option, so `store save 0` — the natural form,
and the one the bring-up runbook uses — was rejected with "unrecognized
arguments: 0". `nargs='?'` keeps the omit → next-free-slot default that
cmd_save relies on (`slot is None → read_store_count()`).

argparse rejects unknown arguments with exit code 2 before any command
runs; a non-existent numeric bus parses cleanly and fails later at the
transport, so a non-2 exit proves the arguments were accepted.
"""
import subprocess
import sys


def _run(*argv):
    return subprocess.run(
        [sys.executable, '-m', 'nxs.cli', '-t', 'i2c', '-b', '99', *argv],
        capture_output=True, text=True)


def test_store_save_accepts_positional_slot():
    r = _run('store', 'save', '0')
    assert r.returncode != 2                       # 2 = argparse reject
    assert 'unrecognized arguments' not in r.stderr


def test_store_save_slot_is_optional():
    r = _run('store', 'save')
    assert r.returncode != 2
    assert 'unrecognized arguments' not in r.stderr


def test_store_rm_positional_slot_unchanged():
    r = _run('store', 'rm', '0')
    assert r.returncode != 2
    assert 'unrecognized arguments' not in r.stderr
