#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/raspberry/arUco_termit"
CURRENT_DIR="${ROOT_DIR}/current"

if [[ ! -x "${CURRENT_DIR}/start_termit.sh" ]]; then
    echo "Active TERMiT release is missing: ${CURRENT_DIR}/start_termit.sh" >&2
    exit 1
fi

exec "${CURRENT_DIR}/start_termit.sh" "$@"
