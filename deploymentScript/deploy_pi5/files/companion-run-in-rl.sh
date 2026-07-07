#!/usr/bin/env bash
# Run a command inside the CatSwarm companion conda env (default: RL).
set -euo pipefail

CONDA_ROOT="${COMPANION_CONDA_ROOT:-/home/pi/miniconda}"
CONDA_ENV="${COMPANION_CONDA_ENV:-RL}"
CONDA_SH="${COMPANION_CONDA_SH:-${CONDA_ROOT}/etc/profile.d/conda.sh}"

usage() {
    cat <<EOF
Usage:
  $0 '<shell-command>'
  $0 <executable> [args ...]
  $0 -h|--help

Run a command with conda env ${CONDA_ENV} activated (used by companion tmux panes).

Examples:
  $0 'python -c "import zmq; print(zmq.zmq_version())"'
  $0 /home/pi/miniconda/envs/RL/bin/python script.py --config=foo.json

Environment:
  COMPANION_CONDA_ROOT   Miniconda root (default: /home/pi/miniconda)
  COMPANION_CONDA_ENV    Env name (default: RL)
EOF
}

case "${1:-}" in
    -h|--help|help) usage; exit 0 ;;
esac

if [ ! -f "${CONDA_SH}" ]; then
    echo "companion-run-in-rl: conda.sh not found: ${CONDA_SH}" >&2
    exit 1
fi

if [ "$#" -eq 0 ]; then
    echo "companion-run-in-rl: command required" >&2
    echo "Run: $0 --help" >&2
    exit 1
fi

if [ "$#" -eq 1 ]; then
    # Shell command string — always run under bash with conda activated inside.
    exec bash -lc "source ${CONDA_SH} && conda activate ${CONDA_ENV} && $1"
fi

# Direct argv invocation: activate here, then exec (no nested shell).
# shellcheck disable=SC1090
source "${CONDA_SH}"
conda activate "${CONDA_ENV}"
exec "$@"
