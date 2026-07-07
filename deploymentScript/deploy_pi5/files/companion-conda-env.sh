#!/usr/bin/env bash
# Activate CatSwarm companion conda env for boot scripts and tmux panes.
# Source this file; do not execute directly unless testing.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    case "${1:-}" in
        -h|--help|help)
            cat <<EOF
Usage: source $0
       $0 --help

Source this file in bash to activate conda env RL for companion scripts.

Sets: PATH, PYTHON, COMPANION_PYTHON, COMPANION_RUN_IN_RL, companion_wrap_rl_cmd()

Not for direct execution in normal workflow — use:
  /usr/local/bin/companion-run-in-rl.sh
  ~/deploy_pi5/install-companion-boot.sh <drone_id>

Environment:
  COMPANION_CONDA_ROOT   Miniconda root (default: /home/pi/miniconda)
  COMPANION_CONDA_ENV    Env name (default: RL)
EOF
            exit 0
            ;;
        "")
            echo "Source this file: source $0" >&2
            exit 1
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Run: $0 --help" >&2
            exit 1
            ;;
    esac
fi

COMPANION_CONDA_ROOT="${COMPANION_CONDA_ROOT:-/home/pi/miniconda}"
COMPANION_CONDA_ENV="${COMPANION_CONDA_ENV:-RL}"
COMPANION_CONDA_SH="${COMPANION_CONDA_SH:-${COMPANION_CONDA_ROOT}/etc/profile.d/conda.sh}"

_resolve_companion_run_in_rl() {
    if [ -n "${COMPANION_RUN_IN_RL:-}" ] && [ -x "${COMPANION_RUN_IN_RL}" ]; then
        return 0
    fi
    for candidate in \
        /usr/local/bin/companion-run-in-rl.sh \
        "${HOME}/deploy_pi5/files/companion-run-in-rl.sh"; do
        if [ -x "${candidate}" ]; then
            export COMPANION_RUN_IN_RL="${candidate}"
            return 0
        fi
    done
    echo "companion-conda: companion-run-in-rl.sh not found" >&2
    return 1 2>/dev/null || exit 1
}

if [ ! -f "${COMPANION_CONDA_SH}" ]; then
    echo "companion-conda: conda.sh not found: ${COMPANION_CONDA_SH}" >&2
    return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1090
source "${COMPANION_CONDA_SH}"
conda activate "${COMPANION_CONDA_ENV}"

export PATH
export PYTHON="${COMPANION_CONDA_ROOT}/envs/${COMPANION_CONDA_ENV}/bin/python"
export COMPANION_PYTHON="${PYTHON}"
export CONDA_DEFAULT_ENV="${COMPANION_CONDA_ENV}"
_resolve_companion_run_in_rl
unset -f _resolve_companion_run_in_rl 2>/dev/null || true

# Wrap a shell command string for tmux send-keys.
companion_wrap_rl_cmd() {
    printf '%s %q' "${COMPANION_RUN_IN_RL}" "$1"
}
