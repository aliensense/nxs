#!/usr/bin/env bash
# Copyright (c) 2026 Aliensense.
# SPDX-License-Identifier: Apache-2.0
#
# wedge-repro: a bench tool beside `nxs`, not one of its verbs. It runs the
# map check on a parked pod again and again, and stops when the pod's
# unit no longer answers at its alias.
set -euo pipefail

UNIT="unit-cam1-a"
ALIAS="0x32"
RUNS=20
PORT_BUS="/dev/i2c-cam1"
NXS_BIN_DIR="$HOME/.local/bin"

usage() {
    cat <<USAGE
usage: wedge-repro.sh [UNIT [ALIAS [RUNS [BUS]]]]
  UNIT   the pod's unit as the declaration names it (default $UNIT)
  ALIAS  the unit map's host alias (default $ALIAS)
  RUNS   how many checks to run (default $RUNS)
  BUS    the port's I2C device (default $PORT_BUS)
Exit 0 when the unit still answers after every run, 1 when it falls silent.
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi
UNIT="${1:-$UNIT}"
ALIAS="${2:-$ALIAS}"
RUNS="${3:-$RUNS}"
PORT_BUS="${4:-$PORT_BUS}"
BUS="$(readlink -f "$PORT_BUS" | sed 's#.*/i2c-##')"
export PATH="$NXS_BIN_DIR:$PATH"

answers() {
    sudo i2ctransfer -f -y "$BUS" w1@"$ALIAS" 0x00 r1 >/dev/null 2>&1
}

for i in $(seq 1 "$RUNS"); do
    printf "%s run %d: " "$(date +%T.%N | cut -c1-12)" "$i"
    nxs "$UNIT" map check 2>&1 | tail -1 || true
    if ! answers; then
        sleep 5
    fi
    if ! answers; then
        echo "$(date +%T.%N | cut -c1-12) $UNIT silent at $ALIAS after run $i"
        sudo dmesg -T | grep -iE "tegra-i2c|nxs-csi2" | tail -3
        exit 1
    fi
done
echo "$(date +%T) $RUNS runs, $UNIT still answering"
