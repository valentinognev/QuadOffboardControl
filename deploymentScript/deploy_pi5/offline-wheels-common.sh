#!/usr/bin/env bash
# Shared wheel download settings and functions for Pi companion offline installs.
# Sourced by push-offline-packages.sh and pull-offline-packages.sh, or run directly:
#   OFFLINE_WHEELS_DIR=/path/to/wheels ./offline-wheels-common.sh download matplotlib
#
# Cross-platform note: pip evaluates environment markers on the *host* (often x86_64),
# so torch's nvidia-*-cu12 deps get pulled even when --platform is aarch64. We always
# keep --only-binary=:all: with platform flags, and fall back to --no-deps (+ explicit
# CPU runtime deps for torch).

PI_WHEEL_PLATFORM="${PI_WHEEL_PLATFORM:-manylinux2014_aarch64}"
PI_WHEEL_PYTHON="${PI_WHEEL_PYTHON:-3.11}"
PI_WHEEL_IMPLEMENTATION="${PI_WHEEL_IMPLEMENTATION:-cp}"

# Pure-Python / CPU deps for torch when downloaded with --no-deps (no nvidia CUDA pkgs).
TORCH_CPU_DEPS=(
  filelock
  typing-extensions
  sympy
  networkx
  jinja2
  fsspec
)

_pi_wheel_abi() {
  # 3.11 -> cp311
  local ver="${PI_WHEEL_PYTHON}"
  echo "cp${ver//./}"
}

_offline_pip_platform_args() {
  echo --platform "${PI_WHEEL_PLATFORM}" \
    --python-version "${PI_WHEEL_PYTHON}" \
    --implementation "${PI_WHEEL_IMPLEMENTATION}" \
    --abi "$(_pi_wheel_abi)" \
    --only-binary=:all:
}

_pkg_is_torch() {
  local raw="$1" name
  name="${raw%%[=<>!~]*}"
  name="${name%%[*}"
  [[ "${name}" == "torch" ]]
}

offline_download_wheels() {
  local wheels_dir="$1"
  shift
  local packages=("$@")

  if [ "${#packages[@]}" -eq 0 ]; then
    echo "offline_download_wheels: no packages specified" >&2
    return 1
  fi

  # If torch is requested, also fetch CPU runtime deps (needed after --no-deps).
  local expanded=() pkg dep
  local want_torch_deps=0
  for pkg in "${packages[@]}"; do
    expanded+=("${pkg}")
    if _pkg_is_torch "${pkg}"; then
      want_torch_deps=1
    fi
  done
  if [ "${want_torch_deps}" -eq 1 ]; then
    for dep in "${TORCH_CPU_DEPS[@]}"; do
      expanded+=("${dep}")
    done
  fi

  mkdir -p "${wheels_dir}"
  echo "Downloading wheels for ${PI_WHEEL_PLATFORM} Python ${PI_WHEEL_PYTHON} ($(_pi_wheel_abi)): ${expanded[*]}…"

  # Read platform args into array (word-splitting intentional for flags).
  # shellcheck disable=SC2207
  local platform_args=( $(_offline_pip_platform_args) )
  local failed=()

  for pkg in "${expanded[@]}"; do
    echo "  → ${pkg}"
    # torch: always --no-deps when cross-downloading from x86_64 (host markers pull nvidia-*).
    if _pkg_is_torch "${pkg}"; then
      if python3 -m pip download "${pkg}" -d "${wheels_dir}" "${platform_args[@]}" --no-deps; then
        continue
      fi
      echo "  FAILED: ${pkg}" >&2
      failed+=("${pkg}")
      continue
    fi
    if python3 -m pip download "${pkg}" -d "${wheels_dir}" "${platform_args[@]}"; then
      continue
    fi
    # Host markers can still break other packages; retry without deps.
    echo "  retry ${pkg} with --no-deps (cross-arch marker workaround)…"
    if python3 -m pip download "${pkg}" -d "${wheels_dir}" "${platform_args[@]}" --no-deps; then
      continue
    fi
    echo "  FAILED: ${pkg}" >&2
    failed+=("${pkg}")
  done

  if [ "${#failed[@]}" -gt 0 ]; then
    echo "offline_download_wheels: failed packages: ${failed[*]}" >&2
    return 1
  fi
  return 0
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
