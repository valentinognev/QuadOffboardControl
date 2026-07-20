#!/usr/bin/env bash
# Download companion build .debs into offline-debs/ for Pis without internet.
#
# Run on a networked host (dev PC). Architecture:all packages (Eigen) work from
# any host arch; arm64 packages are fetched from Debian pool by URL.
#
# Usage:
#   ./fetch-offline-debs.sh
#   ./fetch-offline-debs.sh /path/to/offline-debs
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${1:-${SCRIPT_DIR}/offline-debs}"
mkdir -p "${OUT_DIR}"

# Keep in sync with COMPANION_APT_BUILD in deploy_pi5_companion.sh.
# Prefer Debian trixie arm64 / all pool (Pi 5 Bookworm/Trixie).
DEBS=(
  # Eigen is Architecture: all — required for SystemManagerCPP.
  "http://deb.debian.org/debian/pool/main/e/eigen3/libeigen3-dev_3.4.0-5_all.deb"
)

download_one() {
  local url="$1"
  local base
  base="$(basename "${url}")"
  local dest="${OUT_DIR}/${base}"
  if [ -f "${dest}" ]; then
    echo "present: ${base}"
    return 0
  fi
  echo "fetch: ${base}"
  curl -fL --connect-timeout 30 -o "${dest}.partial" "${url}"
  mv "${dest}.partial" "${dest}"
}

# Also try host apt for Architecture:all packages (Ubuntu/Debian).
if command -v apt-get >/dev/null 2>&1; then
  echo "apt-get download libeigen3-dev (Architecture: all)…"
  (
    cd "${OUT_DIR}"
    apt-get download libeigen3-dev 2>/dev/null || true
  )
fi

for url in "${DEBS[@]}"; do
  download_one "${url}" || echo "WARNING: failed ${url}" >&2
done

# Optional: arm64 libzmq3-dev if curl can resolve a current pool file.
# (Exact version changes; skip if apt on an arm64 host can download.)
if [ "$(dpkg --print-architecture 2>/dev/null || true)" = "arm64" ]; then
  (
    cd "${OUT_DIR}"
    apt-get download libzmq3-dev build-essential cmake pkg-config 2>/dev/null || true
  )
fi

echo "offline-debs in ${OUT_DIR}:"
ls -la "${OUT_DIR}"/*.deb 2>/dev/null || echo "(none yet)"
