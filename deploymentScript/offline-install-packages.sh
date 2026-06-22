#!/usr/bin/env bash
# Install Python packages into conda env RL from local wheels (no internet on the Pi).
#
# Wheels must be in ~/deploy_pi5/offline-wheels (see push-offline-packages.sh on dev PC).
set -euo pipefail

CONDA_ENV="${CONDA_ENV:-RL}"
CONDA_ROOT="${CONDA_ROOT:-/home/pi/miniconda}"
PIP="${CONDA_ROOT}/envs/${CONDA_ENV}/bin/pip"
PYTHON="${CONDA_ROOT}/envs/${CONDA_ENV}/bin/python"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WHEELS_DIR="${OFFLINE_WHEELS_DIR:-${SCRIPT_DIR}/offline-wheels}"

usage() {
  cat <<EOF
Usage: $0 [package ...]
       $0 --help

Install packages into conda env ${CONDA_ENV} from local wheels (no network).

Default package if none given: matplotlib

Wheels directory: ${WHEELS_DIR}

Fetch from dev PC (Pi has no internet, dev PC has internet):
  ~/deploy_pi5/pull-offline-packages.sh --from=user@192.168.0.10 matplotlib --install

Or push from dev PC:
  ~/deploy_pi5/push-offline-packages.sh pi@rlcat3 matplotlib
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help|help) usage; exit 0 ;;
    --wheels-dir=*) WHEELS_DIR="${1#--wheels-dir=}"; shift ;;
    -*) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    *) break ;;
  esac
done

PACKAGES=("$@")
if [ "${#PACKAGES[@]}" -eq 0 ]; then
  PACKAGES=(matplotlib)
fi

if [ ! -x "${PIP}" ]; then
  echo "Error: pip not found: ${PIP}" >&2
  exit 1
fi

if [ ! -d "${WHEELS_DIR}" ]; then
  echo "Error: wheels directory not found: ${WHEELS_DIR}" >&2
  echo "Run on dev PC: ~/deploy_pi5/push-offline-packages.sh pi@rlcat3 ${PACKAGES[*]}" >&2
  exit 1
fi

shopt -s nullglob
wheels=( "${WHEELS_DIR}"/*.whl "${WHEELS_DIR}"/*.tar.gz )
if [ "${#wheels[@]}" -eq 0 ]; then
  echo "Error: no .whl or .tar.gz files in ${WHEELS_DIR}" >&2
  exit 1
fi

already_ok=()
need_install=()
for pkg in "${PACKAGES[@]}"; do
  import_name="${pkg//-/_}"
  if "${PYTHON}" -c "import ${import_name}" 2>/dev/null; then
    already_ok+=("${pkg}")
  else
    need_install+=("${pkg}")
  fi
done

if [ "${#already_ok[@]}" -gt 0 ]; then
  echo "Already installed: ${already_ok[*]}"
fi
if [ "${#need_install[@]}" -eq 0 ]; then
  exit 0
fi

echo "Installing from ${WHEELS_DIR} into ${CONDA_ENV}: ${need_install[*]}…"
"${PIP}" install --no-index --find-links="${WHEELS_DIR}" "${need_install[@]}"

for pkg in "${need_install[@]}"; do
  import_name="${pkg//-/_}"
  "${PYTHON}" -c "import ${import_name}; print('OK:', '${pkg}', getattr(${import_name}, '__version__', '(import ok)'))"
done
