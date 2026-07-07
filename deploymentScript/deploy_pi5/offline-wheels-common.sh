#!/usr/bin/env bash
# Shared wheel download settings and functions for Pi companion offline installs.
# Sourced by push-offline-packages.sh and pull-offline-packages.sh, or run directly:
#   OFFLINE_WHEELS_DIR=/path/to/wheels ./offline-wheels-common.sh download matplotlib

PI_WHEEL_PLATFORM="${PI_WHEEL_PLATFORM:-manylinux2014_aarch64}"
PI_WHEEL_PYTHON="${PI_WHEEL_PYTHON:-3.11}"
PI_WHEEL_IMPLEMENTATION="${PI_WHEEL_IMPLEMENTATION:-cp}"

offline_download_wheels() {
  local wheels_dir="$1"
  shift
  local packages=("$@")

  if [ "${#packages[@]}" -eq 0 ]; then
    echo "offline_download_wheels: no packages specified" >&2
    return 1
  fi

  local pip_download=(python3 -m pip download "${packages[@]}" -d "${wheels_dir}")
  local with_platform=(
    --platform "${PI_WHEEL_PLATFORM}"
    --python-version "${PI_WHEEL_PYTHON}"
    --implementation "${PI_WHEEL_IMPLEMENTATION}"
    --only-binary=:all:
  )

  mkdir -p "${wheels_dir}"
  echo "Downloading wheels for ${PI_WHEEL_PLATFORM} Python ${PI_WHEEL_PYTHON}: ${packages[*]}…"

  if "${pip_download[@]}" "${with_platform[@]}"; then
    return 0
  fi

  echo "Retrying download (allowing source packages for pure-Python deps)…"
  "${pip_download[@]}" \
    --platform "${PI_WHEEL_PLATFORM}" \
    --python-version "${PI_WHEEL_PYTHON}" \
    --implementation "${PI_WHEEL_IMPLEMENTATION}"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  set -euo pipefail

  usage() {
    cat <<EOF
Usage:
  OFFLINE_WHEELS_DIR=path $0 download <package> [package ...]
  $0 -h|--help

Download Python wheels for Pi companion (aarch64, Python ${PI_WHEEL_PYTHON}).

Used by:
  ~/deploy_pi5/push-offline-packages.sh   (dev PC → Pi)
  ~/deploy_pi5/pull-offline-packages.sh   (Pi ← dev PC)

Environment:
  OFFLINE_WHEELS_DIR         Output directory for wheels
  PI_WHEEL_PLATFORM          Default: ${PI_WHEEL_PLATFORM}
  PI_WHEEL_PYTHON            Default: ${PI_WHEEL_PYTHON}
  PI_WHEEL_IMPLEMENTATION    Default: ${PI_WHEEL_IMPLEMENTATION}
EOF
  }

  case "${1:-}" in
    -h|--help|help) usage; exit 0 ;;
    download)
      shift
      wheels_dir="${OFFLINE_WHEELS_DIR:-$(cd "$(dirname "$0")" && pwd)/offline-wheels}"
      offline_download_wheels "${wheels_dir}" "$@"
      ;;
    "")
      usage >&2
      exit 1
      ;;
    *)
      echo "Unknown command: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
fi
