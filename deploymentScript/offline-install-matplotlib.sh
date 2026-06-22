#!/usr/bin/env bash
# Backward-compatible wrapper — use offline-install-packages.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "${1:-}" in
  -h|--help|help)
    exec "${SCRIPT_DIR}/offline-install-packages.sh" --help
    ;;
esac

exec "${SCRIPT_DIR}/offline-install-packages.sh" matplotlib "$@"
