#!/usr/bin/env bash
#
# CatSwarm Raspberry Pi 5 companion computer — full deployment
#
# Installs OS packages, Pi 5 UART boot config, mavlink-server (systemd), Miniconda
# env "RL" (torch + companion deps by default; use --with-ml for opencv/plotly), clones Git repos under
# ~/RL, builds hardware_adapter, expects startup_scripts under ${RL_ROOT}, and runs basic verification.
#
# Usage (on the Pi, as root):
#   sudo ./deploy_pi5_companion.sh
#   sudo ./deploy_pi5_companion.sh --with-ml
#   sudo ./deploy_pi5_companion.sh --phase=repos
#
# From dev machine (push local ~/RL trees to the Pi, prompt for Pi SSH password):
#   ./deploy_pi5_companion.sh --push-to=192.168.0.153
#   ./deploy_pi5_companion.sh --push-to=pi@192.168.0.153 --run-remote
#
# On the Pi (pull from dev PC over SSH, prompt for dev PC password):
#   sudo ./deploy_pi5_companion.sh --from-dev=192.168.0.10
#   sudo ./deploy_pi5_companion.sh --from-dev=valentin@192.168.0.10
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILES_DIR="${SCRIPT_DIR}/files"

RL_ROOT="${RL_ROOT:-/home/pi/RL}"
DEPLOY_USER="${DEPLOY_USER:-pi}"
DEPLOY_HOME=""

# mavlink-server: default = latest from https://github.com/bluerobotics/mavlink-server/releases
MAVLINK_SERVER_VERSION="${MAVLINK_SERVER_VERSION:-}"
MAVLINK_SERVER_URL="${MAVLINK_SERVER_URL:-}"
MAVLINK_SERVER_ASSET="${MAVLINK_SERVER_ASSET:-mavlink-server-aarch64-unknown-linux-musl}"
MAVLINK_SERVER_FALLBACK_VERSION="${MAVLINK_SERVER_FALLBACK_VERSION:-0.9.0}"
CONDA_ENV="${CONDA_ENV:-RL}"
PYTHON_VERSION="${PYTHON_VERSION:-3.11.5}"
TORCH_VERSION="${TORCH_VERSION:-2.5}"

WITH_ML=0
WITH_SIM=0
SKIP_CLONE=0
BUILD_CPP_SYSMGR=0
SKIP_REBOOT_HINT=0
PHASE="all"
SHOW_PROGRESS=1
PROGRESS_BAR_WIDTH=40
_PROGRESS_TOTAL=0
_PROGRESS_DONE=0
_SUB_TOTAL=0
_SUB_DONE=0

# Sync from dev PC (SSH pull on Pi, or SSH push from dev via --push-to)
FROM_DEV_MODE=0
FROM_DEV_SPEC=""
FROM_DEV_USER=""
FROM_DEV_HOST=""
FROM_DEV_RL_ROOT=""
FROM_DEV_PASSWORD=""
PIP_VIA_DEV_MODE=0
PIP_VIA_DEV_SPEC=""
PIP_VIA_DEV_USER=""
PIP_VIA_DEV_HOST=""
PIP_VIA_DEV_PASSWORD="${DEV_PC_PASSWORD:-}"
PUSH_TO_MODE=0
PUSH_TO_SPEC=""
PUSH_TO_USER="pi"
PUSH_TO_HOST=""
PUSH_TO_PASSWORD=""
RUN_REMOTE_AFTER_PUSH=0
LOCAL_RL_ROOT="${LOCAL_RL_ROOT:-}"

# dest_name:relative_path_under_DEV_RL_ROOT (on the dev machine)
SYNC_COMPONENTS=(
  "hardware_adapter:CatSwarm/hardware_adapter"
  "system_manager:CatSwarm/system_manager"
  "GPS_RTK:GPS_RTK"
  "startup_scripts:CatSwarm/general_infrastructure/deploymentScript/startup_scripts"
)
SYNC_SIM_COMPONENT="general_infrastructure:CatSwarm/general_infrastructure"

# Apt packages for companion (HA C bridges + SystemManagerCPP).
# Required: must be present before --build-cpp-sysmgr / make.
# Optional: best-effort (cppzmq is also vendored under system_managerCPP/third_party).
COMPANION_APT_SYSTEM=(
  git tmux curl wget ca-certificates rsync
  ninja-build meson
  python3 python3-venv python3-dev
  libtool autoconf automake
)
COMPANION_APT_BUILD=(
  build-essential cmake pkg-config
  libzmq3-dev libeigen3-dev
)
COMPANION_APT_OPTIONAL=(
  libcppzmq-dev
)
# Local .deb cache for offline Pis (Architecture:all + arm64). See fetch-offline-debs.sh.
OFFLINE_DEBS_DIR="${OFFLINE_DEBS_DIR:-${SCRIPT_DIR}/offline-debs}"

RSYNC_EXCLUDES=(
  --exclude='.git'
  --exclude='node_modules'
  --exclude='lc29h/src/node_modules'
  --exclude='__pycache__'
  --exclude='.pytest_cache'
  --exclude='*.pyc'
  --exclude='.cursor'
  --exclude='logs/'
  --exclude='**/logs/'
  --exclude='log/'
  --exclude='**/log/'
  --exclude='*_control_logs_*.csv'
  --exclude='*_system_manager.csv'
  --exclude='*.tar'
  --exclude='*.tar.gz'
  --exclude='*.tgz'
  --exclude='*.gz'
  --exclude='obj/'
  --exclude='**/obj/'
  --exclude='*.o'
  --exclude='bin/'
  --exclude='**/bin/'
  --exclude='build/'
  --exclude='**/build/'
  # Host-built ELF at repo root (not under bin/) — x86_64 copy → Exec format error on Pi.
  --exclude='SystemManagerMain'
  --exclude='**/SystemManagerMain'
  --exclude='CommSimVerify'
  --exclude='**/CommSimVerify'
  --exclude='mavlink_to_ZMQ'
  --exclude='**/mavlink_to_ZMQ'
  --exclude='mavlink_to_ZMQ_release'
  --exclude='**/mavlink_to_ZMQ_release'
  --exclude='zmq_commands_mavlink'
  --exclude='**/zmq_commands_mavlink'
  --exclude='zmq_commands_mavlink_release'
  --exclude='**/zmq_commands_mavlink_release'
  --exclude='simplegs'
  --exclude='**/simplegs'
)

REPO_HARDWARE_ADAPTER_SSH="git@github.com:valentinognev/hardware_adapter.git"
REPO_SYSTEM_MANAGER_SSH="git@github.com:valentinognev/system_manager.git"
REPO_GPS_RTK_SSH="git@github.com:valentinognev/GPS_RTK.git"
REPO_GENERAL_INFRA_SSH="git@github.com:valentinognev/QuadOffboardControl.git"
REPO_HARDWARE_ADAPTER_HTTPS="https://github.com/valentinognev/hardware_adapter.git"
REPO_SYSTEM_MANAGER_HTTPS="https://github.com/valentinognev/system_manager.git"
REPO_GPS_RTK_HTTPS="https://github.com/valentinognev/GPS_RTK.git"
REPO_GENERAL_INFRA_HTTPS="https://github.com/valentinognev/QuadOffboardControl.git"
GIT_TRANSPORT="${GIT_TRANSPORT:-auto}"

log() { echo "[deploy] $*"; }
die() { echo "[deploy] ERROR: $*" >&2; exit 1; }

parse_ssh_spec() {
  # Sets global _ssh_user and _ssh_host from $1; default user in $2.
  # Must be defined before resolve_pip_via_dev runs at parse time.
  local spec="$1" default_user="$2"
  if [[ "${spec}" == *@* ]]; then
    _ssh_user="${spec%%@*}"
    _ssh_host="${spec#*@}"
  else
    _ssh_host="${spec}"
    _ssh_user="${default_user}"
  fi
  [ -n "${_ssh_host}" ] || die "missing host in SSH target"
}

progress_enabled() {
  [ "${SHOW_PROGRESS}" -eq 1 ]
}

# Overall deployment progress: [##########----------] 42% (3/7) label
progress_init() {
  _PROGRESS_TOTAL="$1"
  _PROGRESS_DONE=0
  progress_enabled || return 0
  echo ""
  progress_tick "starting"
}

progress_tick() {
  local label="${1:-}"
  if ! progress_enabled; then
    [ -n "${label}" ] && log "${label}"
    return 0
  fi
  if [ "${label}" != "starting" ]; then
    _PROGRESS_DONE=$((_PROGRESS_DONE + 1))
  fi
  local cur="${_PROGRESS_DONE}" tot="${_PROGRESS_TOTAL}"
  [ "${tot}" -lt 1 ] && tot=1
  [ "${cur}" -gt "${tot}" ] && cur="${tot}"
  local filled=$(( cur * PROGRESS_BAR_WIDTH / tot ))
  local empty=$(( PROGRESS_BAR_WIDTH - filled ))
  local bar
  bar="$(printf '%*s' "${filled}" '' | tr ' ' '#')$(printf '%*s' "${empty}" '' | tr ' ' '-')"
  local pct=$(( cur * 100 / tot ))
  if [ "${label}" = "starting" ]; then
    printf '[deploy] [%s] %3d%% (0/%d) preparing…\n' "${bar}" 0 "${tot}"
  else
    printf '[deploy] [%s] %3d%% (%d/%d) %s\n' "${bar}" "${pct}" "${cur}" "${tot}" "${label}"
  fi
}

progress_sub_init() {
  _SUB_TOTAL="$1"
  _SUB_DONE=0
}

progress_sub_tick() {
  local label="${1:-}"
  _SUB_DONE=$((_SUB_DONE + 1))
  if ! progress_enabled; then
    [ -n "${label}" ] && log "  ${label}"
    return 0
  fi
  local cur="${_SUB_DONE}" tot="${_SUB_TOTAL}"
  [ "${tot}" -lt 1 ] && tot=1
  local w=24
  local filled=$(( cur * w / tot ))
  local empty=$(( w - filled ))
  local bar
  bar="$(printf '%*s' "${filled}" '' | tr ' ' '#')$(printf '%*s' "${empty}" '' | tr ' ' '-')"
  printf '[deploy]   [%s] %d/%d %s\n' "${bar}" "${cur}" "${tot}" "${label}"
}

run_rsync() {
  if progress_enabled && [ -t 1 ]; then
    rsync -avz --info=progress2 "$@"
  else
    rsync -avz "$@"
  fi
}

curl_download() {
  local url="$1" out="$2" label="${3:-download}"
  if progress_enabled && [ -t 1 ]; then
    log "${label}…"
    curl -fL --progress-bar "${url}" -o "${out}"
    echo ""
  elif command -v pv >/dev/null 2>&1; then
    curl -fsSL "${url}" | pv -N "${label}" > "${out}"
  else
    log "${label}…"
    curl -fsSL "${url}" -o "${out}"
  fi
}

# True when we can SSH to a networked PC (from --from-dev / --pip-via-dev).
have_dev_pc_fetch() {
  [ -n "${PIP_VIA_DEV_HOST:-}" ] || [ -n "${FROM_DEV_HOST:-}" ]
}

dev_pc_user() { echo "${PIP_VIA_DEV_USER:-${FROM_DEV_USER}}"; }
dev_pc_host() { echo "${PIP_VIA_DEV_HOST:-${FROM_DEV_HOST}}"; }
dev_pc_password() { echo "${PIP_VIA_DEV_PASSWORD:-${FROM_DEV_PASSWORD}}"; }

# Download URL on the networked PC over SSH, then copy to local path (offline Pi).
# Returns 0 on success, 1 on failure.
fetch_url_via_dev_pc() {
  local url="$1" out="$2" label="${3:-download}"
  local user host password remote_tmp remote
  user="$(dev_pc_user)"
  host="$(dev_pc_host)"
  password="$(dev_pc_password)"
  if [ -z "${user}" ] || [ -z "${host}" ]; then
    log "ERROR: no --from-dev / --pip-via-dev host for offline ${label}"
    return 1
  fi
  remote="${user}@${host}"
  remote_tmp="deploy_pi5-offline-staging/${label//[^A-Za-z0-9._-]/_}.bin"

  log "${label}: fetching via ${remote} (Pi has no outbound internet)…"
  ensure_sshpass
  if [ -n "${password}" ]; then
    export SSHPASS="${password}"
    sshpass -e ssh -o StrictHostKeyChecking=accept-new "${remote}" \
      "mkdir -p deploy_pi5-offline-staging && curl -fL --connect-timeout 30 -o '${remote_tmp}' '${url}'" \
      || { log "ERROR: ${label}: download on ${remote} failed"; return 1; }
    sshpass -e scp -o StrictHostKeyChecking=accept-new \
      "${remote}:${remote_tmp}" "${out}" \
      || { log "ERROR: ${label}: scp from ${remote} failed"; return 1; }
    sshpass -e ssh -o StrictHostKeyChecking=accept-new "${remote}" "rm -f '${remote_tmp}'" 2>/dev/null || true
  else
    ssh -o StrictHostKeyChecking=accept-new "${remote}" \
      "mkdir -p deploy_pi5-offline-staging && curl -fL --connect-timeout 30 -o '${remote_tmp}' '${url}'" \
      || { log "ERROR: ${label}: download on ${remote} failed (set password or SSH keys)"; return 1; }
    scp -o StrictHostKeyChecking=accept-new \
      "${remote}:${remote_tmp}" "${out}" \
      || { log "ERROR: ${label}: scp from ${remote} failed"; return 1; }
    ssh -o StrictHostKeyChecking=accept-new "${remote}" "rm -f '${remote_tmp}'" 2>/dev/null || true
  fi
  return 0
}

# Prefer direct curl; on DNS/network failure use dev PC when available.
# With --from-dev/--pip-via-dev, skip the doomed GitHub attempt on offline Pis.
# Returns 0 on success, 1 on failure (caller may fall back to a local binary).
curl_or_dev_download() {
  local url="$1" out="$2" label="${3:-download}"
  if [ "${PIP_VIA_DEV_MODE:-0}" -eq 1 ] && have_dev_pc_fetch; then
    fetch_url_via_dev_pc "${url}" "${out}" "${label}"
    return $?
  fi
  if curl_download "${url}" "${out}" "${label}"; then
    return 0
  fi
  if have_dev_pc_fetch; then
    log "WARNING: direct download failed; retrying ${label} via dev PC…"
    fetch_url_via_dev_pc "${url}" "${out}" "${label}"
    return $?
  fi
  return 1
}

progress_count_pi_steps() {
  local n=1
  case "${PHASE}" in
    all)
      n=5   # mavlink, python header, build, verify (+ system tick in phase_system)
      n=$((n + 1))   # system
      if [ "${FROM_DEV_MODE}" -eq 1 ]; then
        n=$((n + 1 + ${#SYNC_COMPONENTS[@]}))   # ssh + components
        [ "${WITH_SIM}" -eq 1 ] && n=$((n + 1))
      elif [ "${SKIP_CLONE}" -eq 0 ]; then
        n=$((n + 3))
        [ "${WITH_SIM}" -eq 1 ] && n=$((n + 1))
      elif [ "${FROM_DEV_MODE}" -eq 0 ]; then
        n=$((n + 1))   # verify startup_scripts
      fi
      n=$((n + 9))   # python: miniconda?, env, pip base, torch, pip gps, conda bashrc, pyzmq, pyserial, pymavlink
      [ "${WITH_ML}" -eq 1 ] && n=$((n + 1))
      [ "${BUILD_CPP_SYSMGR}" -eq 1 ] && n=$((n + 1))
      ;;
    python)
      n=10
      [ "${WITH_ML}" -eq 1 ] && n=$((n + 1))
      ;;
    repos)
      if [ "${FROM_DEV_MODE}" -eq 1 ]; then
        n=$((1 + ${#SYNC_COMPONENTS[@]}))
        [ "${WITH_SIM}" -eq 1 ] && n=$((n + 1))
      elif [ "${SKIP_CLONE}" -eq 0 ]; then
        n=3
        [ "${WITH_SIM}" -eq 1 ] && n=$((n + 1))
      else
        n=1
      fi
      ;;
    build)
      n=1
      [ "${BUILD_CPP_SYSMGR}" -eq 1 ] && n=2
      ;;
    *) n=1 ;;
  esac
  echo "${n}"
}

progress_count_push_steps() {
  local n=2   # ssh test + bundle upload
  n=$((n + ${#SYNC_COMPONENTS[@]}))
  [ "${WITH_SIM}" -eq 1 ] && n=$((n + 1))
  [ "${RUN_REMOTE_AFTER_PUSH}" -eq 1 ] && n=$((n + $(progress_count_pi_steps)))
  echo "${n}"
}

usage() {
  cat <<EOF
Usage: sudo $0 [options]

Options:
  --with-ml              Also install opencv, plotly (heavy extras; torch is always installed)
  --with-sim             Also clone QuadOffboardControl → ${RL_ROOT}/general_infrastructure
  --build-cpp-sysmgr     Build system_manager SystemManagerMain (C++)
  --skip-clone           Skip git clone/pull and startup_scripts sync
  --from-dev[=USER@IP]   On Pi: rsync project trees FROM dev PC (prompts SSH password)
  --from-dev-path=PATH   Dev PC RL root (default: /home/USER/RL)
  --from-dev-password=P  Dev PC SSH password (non-interactive; avoid on shared systems)
  --pip-via-dev[=USER@IP]  Fetch pip wheels via dev PC (auto-enabled with --from-dev)
  --pip-via-dev-password=P SSH password for --pip-via-dev (default: --from-dev-password)
  --push-to[=USER@IP]    On dev PC: rsync local trees TO the Pi (prompts SSH password)
  --local-rl-root=PATH   Dev PC ~/RL path for --push-to (default: \$HOME/RL)
  --run-remote           With --push-to: SSH to Pi and run this script (--skip-clone)
  --push-password=P      Pi SSH password for --run-remote (optional)
  --phase=PHASE          Run one phase: all|system|peripherals|repos|mavlink|python|build|verify
  --rl-root=PATH         Target tree (default: ${RL_ROOT})
  --user=NAME            Repo owner (default: ${DEPLOY_USER})
  --skip-reboot-hint     Do not print reboot reminder after UART changes
  --git-https            Force HTTPS clone (no GitHub SSH key required)
  --git-ssh              Force SSH clone
  --no-progress          Disable progress bars (plain log lines only)
  -h, --help, help       Show this help

Environment:
  MAVLINK_SERVER_VERSION   Pin release (e.g. 0.9.0); default: latest from GitHub API
  MAVLINK_SERVER_URL       Full download URL (overrides version lookup)
  MAVLINK_SERVER_FALLBACK_VERSION  If GitHub unreachable (default: ${MAVLINK_SERVER_FALLBACK_VERSION})
  TORCH_VERSION            Pin torch for system_manager RL controller (default: ${TORCH_VERSION})
  DEV_PC_PASSWORD          Same as --pip-via-dev-password

Python (${CONDA_ENV} env): pyzmq, pyserial, pymavlink, matplotlib, torch, ...
  Offline Pi: sudo $0 --from-dev=user@192.168.0.39
  Or:         sudo $0 --pip-via-dev=user@192.168.0.39

Phases:
  system      apt, dialout, fleet fleet UART boot overlays (idempotent)
  peripherals dialout + UART boot rewrite + companion-gps AMA4→AMA0 (Update always)
  repos       git clone/pull, --from-dev/--push-to rsync (incl. startup_scripts), or verify existing ${RL_ROOT}/startup_scripts
  mavlink     mavlink-server binary, config, systemd
  companion   companion-drone systemd service (requires COMPANION_DRONE_ID=<id>)
  python      Miniconda + RL env (torch, matplotlib, …; --with-ml adds opencv/plotly)
  build       make in hardware_adapter (+ optional C++ system manager)
  verify      import checks, companion-gps ensure, service status
EOF
}

_DEPLOY_ARGC=$#

while [ $# -gt 0 ]; do
  case "$1" in
    --with-ml) WITH_ML=1; shift ;;
    --with-sim) WITH_SIM=1; shift ;;
    --build-cpp-sysmgr) BUILD_CPP_SYSMGR=1; shift ;;
    --skip-clone) SKIP_CLONE=1; shift ;;
    --from-dev) FROM_DEV_MODE=1; shift ;;
    --from-dev=*) FROM_DEV_MODE=1; FROM_DEV_SPEC="${1#--from-dev=}"; shift ;;
    --from-dev-path=*) FROM_DEV_RL_ROOT="${1#--from-dev-path=}"; shift ;;
    --from-dev-password=*) FROM_DEV_PASSWORD="${1#--from-dev-password=}"; shift ;;
    --pip-via-dev) PIP_VIA_DEV_MODE=1; shift ;;
    --pip-via-dev=*) PIP_VIA_DEV_MODE=1; PIP_VIA_DEV_SPEC="${1#--pip-via-dev=}"; shift ;;
    --pip-via-dev-password=*) PIP_VIA_DEV_PASSWORD="${1#--pip-via-dev-password=}"; shift ;;
    --push-to) PUSH_TO_MODE=1; shift ;;
    --push-to=*) PUSH_TO_MODE=1; PUSH_TO_SPEC="${1#--push-to=}"; shift ;;
    --local-rl-root=*) LOCAL_RL_ROOT="${1#--local-rl-root=}"; shift ;;
    --run-remote) RUN_REMOTE_AFTER_PUSH=1; shift ;;
    --push-password=*) PUSH_TO_PASSWORD="${1#--push-password=}"; shift ;;
    --skip-reboot-hint) SKIP_REBOOT_HINT=1; shift ;;
    --git-https) GIT_TRANSPORT=https; shift ;;
    --git-ssh) GIT_TRANSPORT=ssh; shift ;;
    --no-progress) SHOW_PROGRESS=0; shift ;;
    --phase=*) PHASE="${1#--phase=}"; shift ;;
    --rl-root=*) RL_ROOT="${1#--rl-root=}"; shift ;;
    --user=*) DEPLOY_USER="${1#--user=}"; DEPLOY_HOME="$(getent passwd "${DEPLOY_USER}" | cut -d: -f6)"; shift ;;
    -h|--help|help) usage; exit 0 ;;
    *) die "unknown option: $1 (try --help)" ;;
  esac
done

if [ "${PUSH_TO_MODE}" -eq 1 ] && [ "${FROM_DEV_MODE}" -eq 1 ]; then
  die "use either --from-dev (on Pi) or --push-to (on dev PC), not both"
fi

resolve_pip_via_dev() {
  if [ "${PIP_VIA_DEV_MODE}" -eq 0 ] && [ "${FROM_DEV_MODE}" -eq 1 ] && [ -n "${FROM_DEV_SPEC}" ]; then
    PIP_VIA_DEV_MODE=1
    PIP_VIA_DEV_SPEC="${FROM_DEV_SPEC}"
    log "pip: auto-enabled --pip-via-dev via --from-dev (${PIP_VIA_DEV_SPEC})"
  fi
  if [ "${PIP_VIA_DEV_MODE}" -eq 0 ]; then
    return 0
  fi
  [ -n "${PIP_VIA_DEV_SPEC}" ] || die "--pip-via-dev requires USER@HOST or IP"
  parse_ssh_spec "${PIP_VIA_DEV_SPEC}" "${DEV_PC_USER:-${DEPLOY_USER}}"
  PIP_VIA_DEV_USER="${_ssh_user}"
  PIP_VIA_DEV_HOST="${_ssh_host}"
  if [ -z "${PIP_VIA_DEV_PASSWORD}" ] && [ -n "${FROM_DEV_PASSWORD}" ]; then
    PIP_VIA_DEV_PASSWORD="${FROM_DEV_PASSWORD}"
  fi
}

resolve_pip_via_dev

resolve_deploy_home() {
  if [ -n "${DEPLOY_HOME}" ]; then
    return 0
  fi
  DEPLOY_HOME="$(getent passwd "${DEPLOY_USER}" 2>/dev/null | cut -d: -f6 || true)"
  DEPLOY_HOME="${DEPLOY_HOME:-/home/${DEPLOY_USER}}"
}

# --- push mode: run on dev PC, copy local trees to Pi ---------------------------------

is_raspberry_pi() {
  [ -r /proc/device-tree/model ] && grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null
}

ensure_sshpass() {
  if command -v sshpass >/dev/null 2>&1; then
    return 0
  fi
  if [ "$(id -u)" -eq 0 ]; then
    apt-get update -qq
    apt-get install -y sshpass
  elif command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update -qq
    sudo apt-get install -y sshpass
  else
    die "install sshpass (apt install sshpass) for password-based SSH sync"
  fi
}

prompt_password() {
  local varname="$1" prompt="$2" _val=""
  if [ -n "${!varname:-}" ]; then
    return 0
  fi
  read -rsp "${prompt}" _val
  echo ""
  printf -v "$varname" '%s' "$_val"
}

setup_rsync_rsh() {
  local password="$1"
  ensure_sshpass
  export SSHPASS="${password}"
  export RSYNC_RSH="sshpass -e ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=password,keyboard-interactive -o PubkeyAuthentication=yes"
}

clear_rsync_rsh() {
  unset SSHPASS RSYNC_RSH
}

resolve_local_rl_root() {
  if [ -n "${LOCAL_RL_ROOT}" ]; then
    return 0
  fi
  if [ -d "${HOME}/RL" ]; then
    LOCAL_RL_ROOT="${HOME}/RL"
  elif [ -d "$(cd "${SCRIPT_DIR}/../.." && pwd)/hardware_adapter" ]; then
    LOCAL_RL_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
  else
    die "set --local-rl-root=PATH to your dev machine RL directory (e.g. /home/you/RL)"
  fi
}

controller_push_deploy() {
  progress_init "$(progress_count_push_steps)"
  log "=== push mode: copy local RL trees to Raspberry Pi ==="
  resolve_local_rl_root
  log "local RL root: ${LOCAL_RL_ROOT}"

  if [ -z "${PUSH_TO_SPEC}" ]; then
    read -rp "Raspberry Pi IP address: " PUSH_TO_HOST
    read -rp "Pi SSH username [${PUSH_TO_USER}]: " _u
    PUSH_TO_USER="${_u:-${PUSH_TO_USER}}"
  else
    parse_ssh_spec "${PUSH_TO_SPEC}" "${PUSH_TO_USER}"
    PUSH_TO_USER="${_ssh_user}"
    PUSH_TO_HOST="${_ssh_host}"
  fi

  prompt_password PUSH_TO_PASSWORD "SSH password for ${PUSH_TO_USER}@${PUSH_TO_HOST}: "
  setup_rsync_rsh "${PUSH_TO_PASSWORD}"

  progress_tick "SSH test ${PUSH_TO_USER}@${PUSH_TO_HOST}"
  sshpass -e ssh -o StrictHostKeyChecking=accept-new \
    "${PUSH_TO_USER}@${PUSH_TO_HOST}" 'echo push-ssh-ok' >/dev/null \
    || die "SSH login failed for ${PUSH_TO_USER}@${PUSH_TO_HOST}"

  sshpass -e ssh "${PUSH_TO_USER}@${PUSH_TO_HOST}" "mkdir -p '${RL_ROOT}'"

  local entry dest_name rel src
  for entry in "${SYNC_COMPONENTS[@]}"; do
    dest_name="${entry%%:*}"
    rel="${entry#*:}"
    src="${LOCAL_RL_ROOT}/${rel}"
    [ -d "${src}" ] || die "missing local path: ${src}"
    progress_tick "push ${dest_name}"
    run_rsync "${RSYNC_EXCLUDES[@]}" "${src}/" \
      "${PUSH_TO_USER}@${PUSH_TO_HOST}:${RL_ROOT}/${dest_name}/"
  done

  if [ "${WITH_SIM}" -eq 1 ]; then
    dest_name="${SYNC_SIM_COMPONENT%%:*}"
    rel="${SYNC_SIM_COMPONENT#*:}"
    src="${LOCAL_RL_ROOT}/${rel}"
    if [ -d "${src}" ]; then
      progress_tick "push ${dest_name} (sim)"
      run_rsync "${RSYNC_EXCLUDES[@]}" "${src}/" \
        "${PUSH_TO_USER}@${PUSH_TO_HOST}:${RL_ROOT}/${dest_name}/"
    else
      log "WARNING: --with-sim but missing ${src}"
    fi
  fi

  progress_tick "upload deployment bundle"
  sshpass -e ssh "${PUSH_TO_USER}@${PUSH_TO_HOST}" "mkdir -p ~/deploy_pi5/files"
  run_rsync "${SCRIPT_DIR}/" "${PUSH_TO_USER}@${PUSH_TO_HOST}:~/deploy_pi5/"
  run_rsync "${FILES_DIR}/" "${PUSH_TO_USER}@${PUSH_TO_HOST}:~/deploy_pi5/files/"

  clear_rsync_rsh

  if [ "${RUN_REMOTE_AFTER_PUSH}" -eq 0 ]; then
    cat <<EOF

Push finished. On the Pi run:
  sudo bash ~/deploy_pi5/deploy_pi5_companion.sh --skip-clone

Or re-run from this PC with --run-remote (Pi user needs passwordless sudo, or enter Pi sudo password when prompted).
EOF
    return 0
  fi

  progress_tick "remote install on Pi"
  local remote_args="--skip-clone"
  [ "${WITH_ML}" -eq 1 ] && remote_args+=" --with-ml"
  [ "${BUILD_CPP_SYSMGR}" -eq 1 ] && remote_args+=" --build-cpp-sysmgr"
  [ "${PHASE}" != "all" ] && remote_args+=" --phase=${PHASE}"

  setup_rsync_rsh "${PUSH_TO_PASSWORD}"
  log "starting remote install (sudo password may be requested on the Pi)…"
  sshpass -e ssh -tt "${PUSH_TO_USER}@${PUSH_TO_HOST}" \
    "sudo bash ~/deploy_pi5/deploy_pi5_companion.sh ${remote_args}" \
    || die "remote deploy failed on ${PUSH_TO_HOST}"
  clear_rsync_rsh
}

if [ "${PUSH_TO_MODE}" -eq 1 ]; then
  if is_raspberry_pi; then
    die "--push-to is for running on your dev PC, not on the Pi (use --from-dev on the Pi)"
  fi
  controller_push_deploy
  exit 0
fi

if [ "${_DEPLOY_ARGC}" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
  usage
  exit 0
fi

resolve_deploy_home
CONDA_PREFIX="${DEPLOY_HOME}/miniconda"

# --- Pi install path (requires root) -------------------------------------------------

if [ "$(id -u)" -ne 0 ]; then
  die "run as root: sudo $0  (from dev PC use --push-to=pi@IP instead)"
fi

if [ ! -d "${FILES_DIR}" ]; then
  die "missing bundled files directory: ${FILES_DIR}"
fi

run_as_user() {
  sudo -u "${DEPLOY_USER}" -H env HOME="${DEPLOY_HOME}" "$@"
}

boot_config_path() {
  if [ -f /boot/firmware/config.txt ]; then
    echo /boot/firmware/config.txt
  elif [ -f /boot/config.txt ]; then
    echo /boot/config.txt
  else
    die "cannot find config.txt under /boot or /boot/firmware"
  fi
}

phase_enabled() {
  local p="$1"
  [ "${PHASE}" = "all" ] || [ "${PHASE}" = "${p}" ]
}

# --- apt helpers (online + offline-debs fallback) ---

package_installed() {
  local pkg="$1"
  dpkg -s "${pkg}" >/dev/null 2>&1
}

# Install one package from OFFLINE_DEBS_DIR if a matching .deb exists.
install_pkg_from_offline_deb() {
  local pkg="$1"
  local deb=""
  local f
  [ -d "${OFFLINE_DEBS_DIR}" ] || return 1
  # Prefer exact name prefix: libeigen3-dev_*.deb
  for f in "${OFFLINE_DEBS_DIR}/${pkg}_"*.deb "${OFFLINE_DEBS_DIR}/${pkg}"*.deb; do
    [ -f "${f}" ] || continue
    deb="${f}"
    break
  done
  [ -n "${deb}" ] || return 1
  log "offline-debs: dpkg -i $(basename "${deb}")"
  dpkg -i "${deb}" || {
    apt-get install -y -f --no-install-recommends || true
    dpkg -i "${deb}"
  }
}

# Ensure listed packages: apt if online, else offline-debs/. Returns 0 if all present.
ensure_apt_packages() {
  local -a pkgs=("$@")
  local -a missing=()
  local pkg
  local apt_extra=()
  progress_enabled && apt_extra=(-o Dpkg::Progress-Fancy=1)

  for pkg in "${pkgs[@]}"; do
    package_installed "${pkg}" || missing+=("${pkg}")
  done
  if [ "${#missing[@]}" -eq 0 ]; then
    log "apt packages already present: ${pkgs[*]}"
    return 0
  fi

  log "installing apt packages: ${missing[*]}"
  if apt-get "${apt_extra[@]}" update -qq \
    && apt-get "${apt_extra[@]}" install -y --no-install-recommends "${missing[@]}"; then
    return 0
  fi

  log "apt install failed or offline — trying ${OFFLINE_DEBS_DIR}"
  local still=()
  for pkg in "${missing[@]}"; do
    if package_installed "${pkg}"; then
      continue
    fi
    if install_pkg_from_offline_deb "${pkg}"; then
      continue
    fi
    still+=("${pkg}")
  done
  if [ "${#still[@]}" -gt 0 ]; then
    log "ERROR: missing packages: ${still[*]}"
    log "  online: sudo apt-get install -y ${still[*]}"
    log "  offline: place .debs in ${OFFLINE_DEBS_DIR} (run fetch-offline-debs.sh on host)"
    return 1
  fi
  return 0
}

ensure_companion_build_packages() {
  ensure_apt_packages "${COMPANION_APT_BUILD[@]}" \
    || die "companion build packages missing (need: ${COMPANION_APT_BUILD[*]})"
  # Optional cppzmq — ignore failure (vendored zmq.hpp).
  if ! package_installed libcppzmq-dev; then
    apt-get install -y --no-install-recommends libcppzmq-dev 2>/dev/null \
      || install_pkg_from_offline_deb libcppzmq-dev \
      || log "NOTE: libcppzmq-dev not available (OK — vendored zmq.hpp)"
  fi
}

# --- system ---

phase_system() {
  progress_tick "system packages & UART"
  log "=== phase: system ==="
  export DEBIAN_FRONTEND=noninteractive

  ensure_apt_packages "${COMPANION_APT_SYSTEM[@]}" "${COMPANION_APT_BUILD[@]}" \
    || die "phase_system: required apt packages failed"
  # Optional packages (do not fail the deploy).
  local opt
  for opt in "${COMPANION_APT_OPTIONAL[@]}"; do
    if package_installed "${opt}"; then
      continue
    fi
    apt-get install -y --no-install-recommends "${opt}" 2>/dev/null \
      || install_pkg_from_offline_deb "${opt}" \
      || log "NOTE: optional package ${opt} not installed"
  done

  if [ "${FROM_DEV_MODE}" -eq 1 ]; then
    # Prefer already-installed /usr/local wrapper when apt/mirrors are unreachable.
    if ! command -v sshpass >/dev/null 2>&1; then
      apt-get install -y sshpass \
        || install_pkg_from_offline_deb sshpass \
        || die "sshpass required for --from-dev"
    else
      log "sshpass already present ($(command -v sshpass)); skipping apt install"
    fi
  fi

  if ! id -nG "${DEPLOY_USER}" | tr ' ' '\n' | grep -qx dialout; then
    usermod -aG dialout "${DEPLOY_USER}"
    log "added ${DEPLOY_USER} to dialout (log out/in or reboot for new sessions)"
  fi

  ensure_fleet_uart_boot_config

  # Legacy mavlink-router on ttyS0 conflicts with fleet layout; disable if installed.
  if systemctl list-unit-files mavlink-router.service &>/dev/null; then
    systemctl disable --now mavlink-router.service 2>/dev/null || true
    log "disabled mavlink-router.service (use mavlink-server on ttyAMA3)"
  fi
}

# Canonical Pi 5 UART layout version. Bump when overlays/roles change so Update rewrites.
FLEET_UART_LAYOUT_VERSION=3

fleet_uart_boot_block() {
  cat <<EOF
# --- CatSwarm Pi 5 fleet UARTs (deploy_pi5_companion.sh) ---
# fleet-uart-layout: ${FLEET_UART_LAYOUT_VERSION}
# uart0-pi5 → ttyAMA0 (GPIO 14/15)   NMEA → PX4 (emulate_gps_to_px4)
#            Header pin 8 = GPIO14 TXD0 (probe here), pin 10 = GPIO15 RXD0
# uart2-pi5 → ttyAMA2 (GPIO 4/5)     GS comm radio
# uart3-pi5 → ttyAMA3 (GPIO 8/9)     PX4 MAVLink (mavlink-server)
# uart4-pi5 → ttyAMA4 (GPIO 12/13)   optional LC29H DA rover (NOT PX4 NMEA)
# USB       → ttyUSB0                LC29H EA rover (fleet default)
# Do NOT use dtparam=uart0=on (conflicts with uart0-pi5). Do NOT enable ctsrts.
[pi4]
dtoverlay=uart3

[pi5]
dtoverlay=uart0-pi5
dtoverlay=uart2-pi5
dtoverlay=uart3-pi5
dtoverlay=uart4-pi5

[all]
enable_uart=1
# --- end CatSwarm Pi 5 fleet UARTs ---
EOF
}

# Idempotent: rewrite boot UART block whenever layout version / overlays are stale.
# Soft Update must call this (via --phase=peripherals) — not only first New install.
ensure_fleet_uart_boot_config() {
  local cfg
  cfg="$(boot_config_path)"
  log "UART boot config: ${cfg}"

  local need=0
  if ! grep -q "fleet-uart-layout: ${FLEET_UART_LAYOUT_VERSION}" "${cfg}" 2>/dev/null; then
    need=1
    log "UART layout missing or outdated (want fleet-uart-layout: ${FLEET_UART_LAYOUT_VERSION})"
  fi
  if ! grep -q 'dtoverlay=uart0-pi5' "${cfg}" 2>/dev/null; then
    need=1
    log "UART overlay uart0-pi5 missing (required for NMEA→PX4 on GPIO14)"
  fi
  if ! grep -q 'dtoverlay=uart2-pi5' "${cfg}" 2>/dev/null \
    || ! grep -q 'dtoverlay=uart3-pi5' "${cfg}" 2>/dev/null \
    || ! grep -q 'dtoverlay=uart4-pi5' "${cfg}" 2>/dev/null; then
    need=1
    log "one or more fleet uart overlays missing"
  fi
  # Legacy / conflicting enable — forces wrong uart0 defaults on Pi 5.
  if grep -qE '^dtparam=uart0=on' "${cfg}" 2>/dev/null; then
    need=1
    log "removing conflicting dtparam=uart0=on"
  fi
  if grep -qE 'uart0-pi5,ctsrts|dtoverlay=uart0-pi5,ctsrts' "${cfg}" 2>/dev/null; then
    need=1
    log "uart0-pi5 ctsrts must be off for GPS NMEA TX"
  fi

  if [ "${need}" -eq 0 ]; then
    log "fleet UART boot config up to date (layout ${FLEET_UART_LAYOUT_VERSION})"
    return 0
  fi

  local tmp
  tmp="$(mktemp)"
  # Drop previous CatSwarm UART block (legacy end=enable_uart=1 or new end marker).
  awk '
    BEGIN { inblk=0 }
    /# --- CatSwarm Pi 5 fleet UARTs/ { inblk=1; next }
    inblk && /# --- end CatSwarm Pi 5 fleet UARTs ---/ { inblk=0; next }
    inblk && /^enable_uart=1$/ { inblk=0; next }
    inblk { next }
    { print }
  ' "${cfg}" > "${tmp}"
  # Strip conflicting global uart0 enable left over from older images.
  grep -vE '^dtparam=uart0=on[[:space:]]*$' "${tmp}" > "${tmp}.2"
  mv "${tmp}.2" "${tmp}"
  printf '\n' >> "${tmp}"
  fleet_uart_boot_block >> "${tmp}"
  cat "${tmp}" > "${cfg}"
  rm -f "${tmp}"
  log "wrote fleet UART boot config (layout ${FLEET_UART_LAYOUT_VERSION})"
  NEED_UART_REBOOT=1
}

# Soft Update / New: boot overlays + companion-gps roles (minimal new-drone peripherals).
phase_peripherals() {
  progress_tick "peripherals (UART + companion-gps)"
  log "=== phase: peripherals ==="

  if ! id -nG "${DEPLOY_USER}" | tr ' ' '\n' | grep -qx dialout; then
    usermod -aG dialout "${DEPLOY_USER}"
    log "added ${DEPLOY_USER} to dialout"
  fi

  ensure_fleet_uart_boot_config

  if systemctl list-unit-files mavlink-router.service &>/dev/null; then
    systemctl disable --now mavlink-router.service 2>/dev/null || true
    log "disabled mavlink-router.service"
  fi

  local ensure_ports="${RL_ROOT}/startup_scripts/util/ensure_companion_uart_ports.sh"
  if [ -f "${ensure_ports}" ]; then
    log "ensuring ~/.config/companion-gps (PX4 NMEA → /dev/ttyAMA0)…"
    if run_as_user bash "${ensure_ports}"; then
      log "companion-gps UART roles OK"
    else
      log "WARNING: ensure_companion_uart_ports.sh reported issues"
    fi
  else
    log "NOTE: missing ${ensure_ports} (sync startup_scripts first)"
  fi

  # Live checks (warn only — overlays may need reboot before nodes appear).
  local dev
  for dev in /dev/ttyAMA0 /dev/ttyAMA2 /dev/ttyAMA3; do
    if [ -e "${dev}" ]; then
      log "OK ${dev}"
    else
      log "NOTE: ${dev} not present yet (reboot after UART overlay change)"
    fi
  done
  if command -v pinctrl >/dev/null 2>&1; then
    local pin14
    pin14="$(pinctrl get 14 2>/dev/null || true)"
    if printf '%s' "${pin14}" | grep -q 'TXD0'; then
      log "OK GPIO14 = TXD0 (UART0 TX for NMEA→PX4) — probe header pin 8"
    else
      log "NOTE: GPIO14 not TXD0 yet (${pin14:-unavailable}) — reboot if overlays just changed"
    fi
  fi

  local verify_tx="${RL_ROOT}/startup_scripts/util/verify_px4_nmea_uart_tx.sh"
  if [ -f "${verify_tx}" ]; then
    if run_as_user bash "${verify_tx}"; then
      log "PX4 NMEA UART TX self-test OK"
    else
      log "WARNING: PX4 NMEA UART TX self-test failed (probe pin 8 / check wiring)"
    fi
  fi
}

# --- repos ---

git_clone_or_pull() {
  local url="$1" dest="$2" name="$3"
  progress_tick "git ${name}"
  if [ -d "${dest}/.git" ]; then
    run_as_user git -C "${dest}" pull --ff-only || die "git pull failed for ${dest}"
  else
    if progress_enabled; then
      run_as_user git -c advice.detachedHead=false clone --progress "${url}" "${dest}" \
        || die "git clone failed for ${dest}"
    else
      run_as_user git clone "${url}" "${dest}" \
        || die "git clone failed for ${dest}"
    fi
  fi
}

check_github_ssh() {
  if run_as_user ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -T git@github.com 2>&1 | grep -qi 'successfully authenticated'; then
    return 0
  fi
  return 1
}

resolve_git_transport() {
  case "${GIT_TRANSPORT}" in
    ssh)
      REPO_HARDWARE_ADAPTER="${REPO_HARDWARE_ADAPTER_SSH}"
      REPO_SYSTEM_MANAGER="${REPO_SYSTEM_MANAGER_SSH}"
      REPO_GPS_RTK="${REPO_GPS_RTK_SSH}"
      REPO_GENERAL_INFRA="${REPO_GENERAL_INFRA_SSH}"
      check_github_ssh || die "GitHub SSH not configured for ${DEPLOY_USER} (use --git-https)"
      ;;
    https)
      REPO_HARDWARE_ADAPTER="${REPO_HARDWARE_ADAPTER_HTTPS}"
      REPO_SYSTEM_MANAGER="${REPO_SYSTEM_MANAGER_HTTPS}"
      REPO_GPS_RTK="${REPO_GPS_RTK_HTTPS}"
      REPO_GENERAL_INFRA="${REPO_GENERAL_INFRA_HTTPS}"
      ;;
    auto|*)
      if check_github_ssh; then
        REPO_HARDWARE_ADAPTER="${REPO_HARDWARE_ADAPTER_SSH}"
        REPO_SYSTEM_MANAGER="${REPO_SYSTEM_MANAGER_SSH}"
        REPO_GPS_RTK="${REPO_GPS_RTK_SSH}"
        REPO_GENERAL_INFRA="${REPO_GENERAL_INFRA_SSH}"
        log "git transport: SSH"
      else
        REPO_HARDWARE_ADAPTER="${REPO_HARDWARE_ADAPTER_HTTPS}"
        REPO_SYSTEM_MANAGER="${REPO_SYSTEM_MANAGER_HTTPS}"
        REPO_GPS_RTK="${REPO_GPS_RTK_HTTPS}"
        REPO_GENERAL_INFRA="${REPO_GENERAL_INFRA_HTTPS}"
        log "git transport: HTTPS (no GitHub SSH key for ${DEPLOY_USER})"
      fi
      ;;
  esac
}

resolve_from_dev_target() {
  if [ -z "${FROM_DEV_SPEC}" ]; then
    read -rp "Dev PC IP address (this machine with the RL sources): " FROM_DEV_HOST
    read -rp "Dev PC SSH username [${FROM_DEV_USER:-valentin}]: " _u
    FROM_DEV_USER="${_u:-${FROM_DEV_USER:-valentin}}"
  else
    parse_ssh_spec "${FROM_DEV_SPEC}" "${FROM_DEV_USER:-valentin}"
    FROM_DEV_USER="${_ssh_user}"
    FROM_DEV_HOST="${_ssh_host}"
  fi
  if [ -z "${FROM_DEV_RL_ROOT}" ]; then
    FROM_DEV_RL_ROOT="/home/${FROM_DEV_USER}/RL"
  fi
}

sync_component_from_dev() {
  local dest_name="$1" rel="$2"
  local remote="${FROM_DEV_USER}@${FROM_DEV_HOST}:${FROM_DEV_RL_ROOT}/${rel}/"
  local dest="${RL_ROOT}/${dest_name}"
  run_as_user mkdir -p "${dest}"
  run_rsync "${RSYNC_EXCLUDES[@]}" "${remote}" "${dest}/"
}

phase_sync_from_dev() {
  log "=== phase: sync from dev PC (${FROM_DEV_USER}@${FROM_DEV_HOST}) ==="
  resolve_from_dev_target
  prompt_password FROM_DEV_PASSWORD "SSH password for ${FROM_DEV_USER}@${FROM_DEV_HOST}: "
  if [ -z "${PIP_VIA_DEV_PASSWORD}" ] && [ -n "${FROM_DEV_PASSWORD}" ]; then
    PIP_VIA_DEV_PASSWORD="${FROM_DEV_PASSWORD}"
  fi
  setup_rsync_rsh "${FROM_DEV_PASSWORD}"

  progress_tick "SSH test dev PC"
  sshpass -e ssh -o StrictHostKeyChecking=accept-new \
    "${FROM_DEV_USER}@${FROM_DEV_HOST}" 'echo from-dev-ok' >/dev/null \
    || die "SSH login failed for ${FROM_DEV_USER}@${FROM_DEV_HOST}"

  run_as_user mkdir -p "${RL_ROOT}"
  local entry dest_name rel
  for entry in "${SYNC_COMPONENTS[@]}"; do
    dest_name="${entry%%:*}"
    rel="${entry#*:}"
    sshpass -e ssh "${FROM_DEV_USER}@${FROM_DEV_HOST}" \
      "test -d '${FROM_DEV_RL_ROOT}/${rel}'" \
      || die "missing on dev PC: ${FROM_DEV_RL_ROOT}/${rel}"
    progress_tick "pull ${dest_name}"
    sync_component_from_dev "${dest_name}" "${rel}"
  done

  if [ "${WITH_SIM}" -eq 1 ]; then
    dest_name="${SYNC_SIM_COMPONENT%%:*}"
    rel="${SYNC_SIM_COMPONENT#*:}"
    if sshpass -e ssh "${FROM_DEV_USER}@${FROM_DEV_HOST}" "test -d '${FROM_DEV_RL_ROOT}/${rel}'"; then
      progress_tick "pull ${dest_name} (sim)"
      sync_component_from_dev "${dest_name}" "${rel}"
    else
      log "WARNING: --with-sim but missing ${FROM_DEV_RL_ROOT}/${rel} on dev PC"
    fi
  fi

  clear_rsync_rsh
  chown -R "${DEPLOY_USER}:${DEPLOY_USER}" "${RL_ROOT}"
  SKIP_CLONE=1
}

ensure_startup_scripts() {
  local dir="${RL_ROOT}/startup_scripts"
  local launcher="${dir}/start_companion_drone_tmux.sh"
  if [ ! -f "${launcher}" ]; then
    die "missing ${launcher} — populate ${RL_ROOT}/startup_scripts (use --from-dev on Pi or --push-to from dev PC)"
  fi
  run_as_user chmod +x "${dir}"/*.sh "${dir}"/util/*.sh 2>/dev/null || true
  chown -R "${DEPLOY_USER}:${DEPLOY_USER}" "${dir}" 2>/dev/null || true
  log "startup_scripts: ${dir}"
}

phase_repos() {
  log "=== phase: repos ==="
  if [ "${FROM_DEV_MODE}" -eq 0 ] && [ "${SKIP_CLONE}" -eq 1 ]; then
    progress_tick "verify startup_scripts"
  fi
  run_as_user mkdir -p "${RL_ROOT}"

  if [ "${FROM_DEV_MODE}" -eq 1 ]; then
    phase_sync_from_dev
  elif [ "${SKIP_CLONE}" -eq 1 ]; then
    log "skip-clone: not updating git repos"
  else
    resolve_git_transport
    git_clone_or_pull "${REPO_HARDWARE_ADAPTER}" "${RL_ROOT}/hardware_adapter" hardware_adapter
    git_clone_or_pull "${REPO_SYSTEM_MANAGER}" "${RL_ROOT}/system_manager" system_manager
    git_clone_or_pull "${REPO_GPS_RTK}" "${RL_ROOT}/GPS_RTK" GPS_RTK
    if [ "${WITH_SIM}" -eq 1 ]; then
      git_clone_or_pull "${REPO_GENERAL_INFRA}" "${RL_ROOT}/general_infrastructure" general_infrastructure
    fi
  fi

  ensure_startup_scripts
}

# --- mavlink-server ---

resolve_mavlink_server_release() {
  if [ -n "${MAVLINK_SERVER_URL}" ]; then
    log "mavlink-server URL (preset): ${MAVLINK_SERVER_URL}"
    return 0
  fi

  if [ -z "${MAVLINK_SERVER_VERSION}" ] \
      && command -v curl >/dev/null \
      && command -v python3 >/dev/null; then
    local resolved tag url
    resolved="$(curl -fsSL --connect-timeout 20 \
      "https://api.github.com/repos/bluerobotics/mavlink-server/releases/latest" 2>/dev/null \
      | python3 -c "
import json, sys
r = json.load(sys.stdin)
tag = r['tag_name']
url = next(
    a['browser_download_url'] for a in r.get('assets', [])
    if 'aarch64-unknown-linux-musl' in a.get('name', '')
)
print(tag)
print(url)
" 2>/dev/null)" || true
    if [ -n "${resolved}" ]; then
      tag="$(printf '%s\n' "${resolved}" | sed -n '1p')"
      url="$(printf '%s\n' "${resolved}" | sed -n '2p')"
      if [ -n "${tag}" ] && [ -n "${url}" ]; then
        MAVLINK_SERVER_VERSION="${tag}"
        MAVLINK_SERVER_URL="${url}"
        log "mavlink-server latest release: ${MAVLINK_SERVER_VERSION}"
        log "  download: ${MAVLINK_SERVER_URL}"
        return 0
      fi
    fi
    log "WARNING: could not resolve latest mavlink-server from GitHub API"
  elif [ -n "${MAVLINK_SERVER_VERSION}" ]; then
    log "mavlink-server pinned version: ${MAVLINK_SERVER_VERSION}"
  fi

  MAVLINK_SERVER_VERSION="${MAVLINK_SERVER_VERSION:-${MAVLINK_SERVER_FALLBACK_VERSION}}"
  MAVLINK_SERVER_URL="https://github.com/bluerobotics/mavlink-server/releases/download/${MAVLINK_SERVER_VERSION}/${MAVLINK_SERVER_ASSET}"
  log "mavlink-server: ${MAVLINK_SERVER_VERSION}"
  log "  download: ${MAVLINK_SERVER_URL}"
}

phase_mavlink() {
  progress_tick "mavlink-server"
  log "=== phase: mavlink ==="
  resolve_mavlink_server_release
  local tmp cache="${SCRIPT_DIR}/offline-binaries/${MAVLINK_SERVER_ASSET}"
  tmp="$(mktemp)"
  if [ -f "${cache}" ]; then
    log "mavlink-server: using cached ${cache}"
    cp -f "${cache}" "${tmp}"
  elif curl_or_dev_download "${MAVLINK_SERVER_URL}" "${tmp}" "mavlink-server ${MAVLINK_SERVER_VERSION}"; then
    mkdir -p "${SCRIPT_DIR}/offline-binaries"
    cp -f "${tmp}" "${cache}" 2>/dev/null || true
    chown -R "${DEPLOY_USER}:${DEPLOY_USER}" "${SCRIPT_DIR}/offline-binaries" 2>/dev/null || true
    chmod 0755 "${cache}" 2>/dev/null || true
  elif [ -x /usr/bin/mavlink-server ]; then
    log "WARNING: download failed; keeping existing /usr/bin/mavlink-server"
    rm -f "${tmp}"
    tmp=""
  else
    die "mavlink-server download failed (offline Pi: --from-dev=user@dev-pc)"
  fi
  if [ -n "${tmp}" ]; then
    install -m 0755 "${tmp}" /usr/bin/mavlink-server
    rm -f "${tmp}"
  fi
  /usr/bin/mavlink-server --version 2>/dev/null || log "mavlink-server installed (no --version)"

  install -d /etc/mavlink-server
  install -m 0644 "${FILES_DIR}/mavlink-server.conf" /etc/mavlink-server/mavlink-server.conf
  install -m 0755 "${FILES_DIR}/run-mavlink-server.sh" /usr/local/bin/run-mavlink-server.sh

  cat > /etc/systemd/system/mavlink-server.service <<'EOF'
[Unit]
Description=MAVLink Server Service
Documentation=https://github.com/bluerobotics/mavlink-server
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/run-mavlink-server.sh /etc/mavlink-server/mavlink-server.conf
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal
SyslogIdentifier=mavlink-server
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
EOF

  systemctl daemon-reload
  systemctl enable mavlink-server.service
  systemctl restart mavlink-server.service
  sleep 2
  if systemctl is-active --quiet mavlink-server.service; then
    log "mavlink-server.service is active"
  else
    log "WARNING: mavlink-server not active (UART3 may be unplugged); check: journalctl -u mavlink-server -n 30"
  fi

  local cfg_script="${SCRIPT_DIR}/mavlink-server-configuration.sh"
  if [ -f "${cfg_script}" ]; then
    chmod +x "${cfg_script}"
    echo ""
    log "mavlink-server installed with fleet default config."
    log "Reconfigure interactively:"
    log "  sudo ${cfg_script}"
    log "Or apply Pi 5 preset without prompts:"
    log "  sudo ${cfg_script} --fleet-preset"
  fi
}

phase_companion() {
  progress_tick "companion-drone service"
  log "=== phase: companion ==="
  local drone_id="${COMPANION_DRONE_ID:-}"
  if [ -z "${drone_id}" ]; then
    die "COMPANION_DRONE_ID is required (e.g. COMPANION_DRONE_ID=3 sudo $0 --phase=companion)"
  fi
  if ! [[ "${drone_id}" =~ ^[0-9]+$ ]] || [ "${drone_id}" -lt 1 ]; then
    die "COMPANION_DRONE_ID must be a positive integer (got: ${drone_id})"
  fi

  install -m 0755 "${FILES_DIR}/run-companion-drone.sh" /usr/local/bin/run-companion-drone.sh
  install -m 0644 "${FILES_DIR}/companion-conda-env.sh" /usr/local/lib/companion-conda-env.sh
  install -m 0755 "${FILES_DIR}/companion-run-in-rl.sh" /usr/local/bin/companion-run-in-rl.sh
  install -d /etc/default
  cat > /etc/default/companion-drone <<EOF
# CatSwarm companion drone boot configuration
COMPANION_DRONE_ID=${drone_id}
RL_ROOT=${RL_ROOT}
COMPANION_CONDA_ROOT=/home/pi/miniconda
COMPANION_CONDA_ENV=RL
COMPANION_UART_WAIT_S=30
EOF
  chmod 0644 /etc/default/companion-drone
  log "installed /etc/default/companion-drone (COMPANION_DRONE_ID=${drone_id})"

  cat > /etc/systemd/system/companion-drone.service <<'EOF'
[Unit]
Description=CatSwarm companion drone stack (hardware adapter + system manager + RTK)
Documentation=file:///home/pi/RL/startup_scripts/README.md
After=network-online.target mavlink-server.service
Wants=network-online.target mavlink-server.service

[Service]
Type=oneshot
RemainAfterExit=yes
User=pi
Group=pi
Environment=HOME=/home/pi
EnvironmentFile=-/etc/default/companion-drone
ExecStartPre=/bin/sleep 5
ExecStart=/usr/local/bin/run-companion-drone.sh
StandardOutput=journal
StandardError=journal
SyslogIdentifier=companion-drone

[Install]
WantedBy=multi-user.target
EOF

  systemctl daemon-reload
  systemctl enable companion-drone.service
  log "companion-drone.service enabled at boot"
  if systemctl start companion-drone.service; then
    if systemctl is-active --quiet companion-drone.service; then
      log "companion-drone.service is active"
    else
      log "WARNING: companion-drone did not become active; check: journalctl -u companion-drone -n 40"
    fi
  else
    log "WARNING: companion-drone start failed (UART/RL tree may be missing); check: journalctl -u companion-drone -n 40"
  fi
}

# --- python ---

setup_conda_bashrc() {
  local bashrc="${DEPLOY_HOME}/.bashrc"
  local conda_bin="${CONDA_PREFIX}/bin/conda"
  local marker="CatSwarm companion (deploy_pi5_companion.sh): default conda env"
  [ -x "${conda_bin}" ] || die "conda not found at ${CONDA_PREFIX}"
  run_as_user "${conda_bin}" env list | awk '{print $1}' | grep -qx "${CONDA_ENV}" \
    || die "conda env ${CONDA_ENV} missing; create it before .bashrc setup"

  progress_tick "conda ${CONDA_ENV} default in .bashrc"
  log "configure conda in ${bashrc} (default env: ${CONDA_ENV}, not base)"

  if [ ! -f "${bashrc}" ]; then
    run_as_user touch "${bashrc}"
  fi

  if ! run_as_user grep -q '>>> conda initialize >>>' "${bashrc}" 2>/dev/null; then
    run_as_user "${conda_bin}" init bash
  else
    log "conda initialize block already in .bashrc"
  fi

  run_as_user "${conda_bin}" config --set auto_activate_base false

  if ! run_as_user grep -qF "${marker}" "${bashrc}" 2>/dev/null; then
    run_as_user tee -a "${bashrc}" >/dev/null <<EOF

# ${marker}
conda activate ${CONDA_ENV}
EOF
    log "appended: conda activate ${CONDA_ENV}"
  else
    log "default env block already in .bashrc"
  fi

  chown "${DEPLOY_USER}:${DEPLOY_USER}" "${bashrc}" 2>/dev/null || true
  [ -f "${DEPLOY_HOME}/.bash_profile" ] && chown "${DEPLOY_USER}:${DEPLOY_USER}" "${DEPLOY_HOME}/.bash_profile" || true
}

collect_companion_pip_packages() {
  COMPANION_PIP_PKGS=(
    pyzmq pyserial pymavlink meson numpy scipy pymap3d matplotlib
    "torch==${TORCH_VERSION}"
  )
  if [ -f "${RL_ROOT}/GPS_RTK/combination/requirements.txt" ]; then
    local line=""
    while IFS= read -r line || [ -n "${line}" ]; do
      line="${line%%#*}"
      line="${line#"${line%%[![:space:]]*}"}"
      line="${line%"${line##*[![:space:]]}"}"
      [ -n "${line}" ] && COMPANION_PIP_PKGS+=("${line}")
    done < "${RL_ROOT}/GPS_RTK/combination/requirements.txt"
  fi
  if [ "${WITH_ML}" -eq 1 ]; then
    COMPANION_PIP_PKGS+=( plotly opencv-python-headless )
  fi
}

install_companion_pip_via_dev() {
  local pull_script="${DEPLOY_HOME}/deploy_pi5/pull-offline-packages.sh"
  [ -f "${pull_script}" ] || pull_script="${SCRIPT_DIR}/pull-offline-packages.sh"
  [ -x "${pull_script}" ] || die "missing pull-offline-packages.sh (expected ${DEPLOY_HOME}/deploy_pi5/)"

  collect_companion_pip_packages
  ensure_sshpass

  progress_tick "pip via dev PC ${PIP_VIA_DEV_USER}@${PIP_VIA_DEV_HOST}"
  log "installing pip packages via dev PC ${PIP_VIA_DEV_USER}@${PIP_VIA_DEV_HOST}…"
  log "packages: ${COMPANION_PIP_PKGS[*]}"

  if [ -n "${PIP_VIA_DEV_PASSWORD}" ]; then
    run_as_user env DEV_PC_PASSWORD="${PIP_VIA_DEV_PASSWORD}" "${pull_script}" \
      --from="${PIP_VIA_DEV_USER}@${PIP_VIA_DEV_HOST}" \
      "${COMPANION_PIP_PKGS[@]}" --install \
      || die "pip via dev PC failed"
  else
    run_as_user "${pull_script}" \
      --from="${PIP_VIA_DEV_USER}@${PIP_VIA_DEV_HOST}" \
      "${COMPANION_PIP_PKGS[@]}" --install \
      || die "pip via dev PC failed (set --pip-via-dev-password or use SSH keys)"
  fi
}

install_companion_pip_online() {
  local pip="$1"
  shift
  local pip_flags=("$@")

  progress_tick "pip companion packages"
  if [ "${#pip_flags[@]}" -gt 0 ]; then
    run_as_user "${pip}" install "${pip_flags[@]}" --upgrade pip
    run_as_user "${pip}" install "${pip_flags[@]}" \
      pyzmq pyserial pymavlink meson numpy scipy pymap3d matplotlib
  else
    run_as_user "${pip}" install --upgrade pip
    run_as_user "${pip}" install pyzmq pyserial pymavlink meson numpy scipy pymap3d matplotlib
  fi

  if [ -f "${RL_ROOT}/GPS_RTK/combination/requirements.txt" ]; then
    progress_tick "pip GPS_RTK requirements"
    if [ "${#pip_flags[@]}" -gt 0 ]; then
      run_as_user "${pip}" install "${pip_flags[@]}" -r "${RL_ROOT}/GPS_RTK/combination/requirements.txt"
    else
      run_as_user "${pip}" install -r "${RL_ROOT}/GPS_RTK/combination/requirements.txt"
    fi
  fi

  progress_tick "pip install torch==${TORCH_VERSION} (${CONDA_ENV})"
  if [ "${#pip_flags[@]}" -gt 0 ]; then
    run_as_user "${pip}" install "${pip_flags[@]}" "torch==${TORCH_VERSION}" \
      || die "torch install failed (offline Pi: sudo $0 --pip-via-dev=user@dev-pc)"
  else
    run_as_user "${pip}" install "torch==${TORCH_VERSION}" \
      || die "torch install failed (offline Pi: sudo $0 --pip-via-dev=user@dev-pc)"
  fi
  run_as_user "${CONDA_PREFIX}/envs/${CONDA_ENV}/bin/python" -c \
    "import torch; print('${CONDA_ENV}: torch', torch.__version__)" \
    || die "torch not importable in ${CONDA_ENV}"

  if [ "${WITH_ML}" -eq 1 ]; then
    progress_tick "pip ML stack (--with-ml)"
    if [ "${#pip_flags[@]}" -gt 0 ]; then
      run_as_user "${pip}" install "${pip_flags[@]}" plotly opencv-python-headless \
        || die "ML extras install failed"
    else
      run_as_user "${pip}" install plotly opencv-python-headless \
        || die "ML extras install failed"
    fi
    run_as_user "${pip}" install sample-factory 2>/dev/null || log "sample-factory optional install skipped"
    run_as_user "${pip}" uninstall -y sample-factory 2>/dev/null || true
  fi
}

verify_companion_pip_imports() {
  local py="${CONDA_PREFIX}/envs/${CONDA_ENV}/bin/python"
  [ -x "${py}" ] || die "missing ${py}"

  progress_tick "verify pyzmq (${CONDA_ENV})"
  run_as_user "${py}" -c "import zmq; print('${CONDA_ENV}: pyzmq', zmq.zmq_version())" \
    || die "pyzmq not importable in ${CONDA_ENV}"

  progress_tick "verify pyserial (${CONDA_ENV})"
  run_as_user "${py}" -c "import serial; print('${CONDA_ENV}: pyserial', serial.__version__)" \
    || die "pyserial not importable in ${CONDA_ENV} (used by startInitRoverPI.sh)"

  progress_tick "verify pymavlink (${CONDA_ENV})"
  run_as_user "${py}" -c "
import pymavlink
_v = getattr(pymavlink, '__version__', None)
print('${CONDA_ENV}: pymavlink', _v if _v is not None else '(import ok)')
" || die "pymavlink not importable in ${CONDA_ENV}"
}

phase_python() {
  progress_tick "Python environment"
  log "=== phase: python (slim=$([ "${WITH_ML}" -eq 0 ] && echo yes || echo no)) ==="
  local miniconda_sh="${DEPLOY_HOME}/miniconda_installer.sh"
  local pip="${CONDA_PREFIX}/envs/${CONDA_ENV}/bin/pip"
  local pip_flags=()
  progress_enabled && pip_flags=(--progress-bar=on)

  if [ ! -x "${CONDA_PREFIX}/bin/conda" ]; then
    progress_tick "Miniconda download"
    curl_or_dev_download "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh" \
      "${miniconda_sh}" "Miniconda installer" \
      || die "Miniconda download failed (offline Pi: --from-dev=user@dev-pc)"
    chown "${DEPLOY_USER}:${DEPLOY_USER}" "${miniconda_sh}"
    progress_tick "Miniconda install"
    run_as_user bash "${miniconda_sh}" -b -p "${CONDA_PREFIX}"
    rm -f "${miniconda_sh}"
  fi

  run_as_user "${CONDA_PREFIX}/bin/conda" tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main 2>/dev/null || true
  run_as_user "${CONDA_PREFIX}/bin/conda" tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r 2>/dev/null || true

  if ! run_as_user "${CONDA_PREFIX}/bin/conda" env list | awk '{print $1}' | grep -qx "${CONDA_ENV}"; then
    progress_tick "conda env ${CONDA_ENV}"
    run_as_user "${CONDA_PREFIX}/bin/conda" create -n "${CONDA_ENV}" "python=${PYTHON_VERSION}" -y
  fi

  # Before pip (pip failure must not skip default-env setup)
  setup_conda_bashrc

  if [ "${PIP_VIA_DEV_MODE}" -eq 1 ]; then
    install_companion_pip_via_dev
  else
    install_companion_pip_online "${pip}" "${pip_flags[@]}"
  fi
  verify_companion_pip_imports
}

# --- build ---

# Pi5: fixed parallelism (nproc can OOM / thrash under concurrent HA+SM builds).
COMPANION_BUILD_JOBS="${COMPANION_BUILD_JOBS:-4}"

phase_build() {
  progress_tick "build C bridges"
  log "=== phase: build ==="
  [ -d "${RL_ROOT}/hardware_adapter" ] || die "missing ${RL_ROOT}/hardware_adapter (run repos phase)"
  export DEBIAN_FRONTEND=noninteractive
  # Update often skips phase_system — always ensure HA/SM build deps (incl. Eigen3).
  ensure_companion_build_packages
  # Wipe host-arch root ELFs left by older deploys (rsync now excludes them).
  rm -f "${RL_ROOT}/hardware_adapter/mavlink_to_ZMQ" \
        "${RL_ROOT}/hardware_adapter/mavlink_to_ZMQ_release" \
        "${RL_ROOT}/hardware_adapter/zmq_commands_mavlink" \
        "${RL_ROOT}/hardware_adapter/zmq_commands_mavlink_release" \
        "${RL_ROOT}/hardware_adapter/simplegs"
  log "HA make -j${COMPANION_BUILD_JOBS}"
  run_as_user make -C "${RL_ROOT}/hardware_adapter" -j"${COMPANION_BUILD_JOBS}"
  for b in mavlink_to_ZMQ zmq_commands_mavlink; do
    local hb="${RL_ROOT}/hardware_adapter/${b}"
    [ -x "${hb}" ] || die "${b} missing after HA make"
    file -b "${hb}" | grep -qi aarch64 || die "${b} is not aarch64 after build: $(file -b "${hb}")"
  done

  if [ "${BUILD_CPP_SYSMGR}" -eq 1 ]; then
    progress_tick "build system_manager C++"
    local sm_cpp="${RL_ROOT}/system_manager/system_managerCPP"
    [ -x "${sm_cpp}/build.sh" ] || die "missing system_managerCPP/build.sh"
    # build.sh uses cwd-relative build/; must run from system_managerCPP.
    # Prefer COMPANION_BUILD_JOBS over build.sh's nproc default.
    run_as_user bash -lc "cd $(printf '%q' "${sm_cpp}") && COMPANION_BUILD_JOBS=${COMPANION_BUILD_JOBS} ./build.sh Release"
    local built=""
    if [ -f "${RL_ROOT}/system_manager/SystemManagerMain" ]; then
      built="${RL_ROOT}/system_manager/SystemManagerMain"
    elif [ -f "${sm_cpp}/build/SystemManagerMain" ]; then
      built="${sm_cpp}/build/SystemManagerMain"
    fi
    [ -n "${built}" ] || die "SystemManagerMain not produced by build.sh"
    local dest="${RL_ROOT}/system_manager/SystemManagerMain"
    # CMake RUNTIME_OUTPUT_DIRECTORY is already system_manager/; skip self-copy.
    if [ "$(readlink -f "${built}")" = "$(readlink -f "${dest}")" ]; then
      chown "${DEPLOY_USER}:${DEPLOY_USER}" "${dest}"
      chmod 0755 "${dest}"
    else
      install -o "${DEPLOY_USER}" -g "${DEPLOY_USER}" -m 0755 "${built}" "${dest}"
    fi
    if ! file -b "${dest}" | grep -qi aarch64; then
      die "SystemManagerMain is not aarch64 after build: $(file -b "${dest}")"
    fi
    log "SystemManagerMain OK (aarch64)"
  fi
}

# --- verify ---

phase_verify() {
  progress_tick "verification"
  log "=== phase: verify ==="
  local py="${CONDA_PREFIX}/envs/${CONDA_ENV}/bin/python"
  run_as_user "${py}" -c "import zmq, serial, numpy, pymap3d, matplotlib, torch; print('python imports OK')"

  if [ -f "${RL_ROOT}/GPS_RTK/combination/verify_rtk_comm_frame.py" ]; then
    if run_as_user "${py}" "${RL_ROOT}/GPS_RTK/combination/verify_rtk_comm_frame.py"; then
      log "verify_rtk_comm_frame.py PASS"
    else
      log "WARNING: verify_rtk_comm_frame.py did not PASS"
    fi
  fi

  for dev in /dev/ttyAMA0 /dev/ttyAMA2 /dev/ttyAMA3 /dev/ttyAMA4; do
    if [ -e "${dev}" ]; then
      log "UART device present: ${dev}"
    else
      log "NOTE: ${dev} not present (normal until overlays apply or hardware wired)"
    fi
  done

  # Persist fleet UART roles into ~/.config/companion-gps (AMA4→AMA0 migration).
  local ensure_ports="${RL_ROOT}/startup_scripts/util/ensure_companion_uart_ports.sh"
  if [ -x "${ensure_ports}" ] || [ -f "${ensure_ports}" ]; then
    progress_tick "ensure companion UART ports"
    log "ensuring companion-gps PX4 port (UART0 / ttyAMA0)…"
    if run_as_user bash "${ensure_ports}"; then
      log "companion UART ports OK (see companion-gps)"
    else
      log "WARNING: ensure_companion_uart_ports.sh reported issues (check wiring / reboot for overlays)"
    fi
  else
    log "NOTE: missing ${ensure_ports} (sync startup_scripts)"
  fi

  systemctl is-enabled mavlink-server.service >/dev/null && log "mavlink-server enabled at boot"
  systemctl is-enabled companion-drone.service >/dev/null && log "companion-drone enabled at boot"
  ls -la "${RL_ROOT}/hardware_adapter/bin/" 2>/dev/null | head -5 || true
  log "deploy tree:"
  run_as_user ls -la "${RL_ROOT}"
}

print_next_steps() {
  cat <<EOF

=================================================================
Deployment finished.
=================================================================
RL tree:     ${RL_ROOT}
Python:      ${CONDA_PREFIX}/envs/${CONDA_ENV}/bin/python

If UART config was added, reboot once:
  sudo reboot

Before/after reboot, companion GPS UART roles are migrated by:
  ${RL_ROOT}/startup_scripts/util/ensure_companion_uart_ports.sh
  (sets COMPANION_PX4_GPS_PORT=/dev/ttyAMA0 in ~/.config/companion-gps)

After reboot:
  1. LC29H DA one-time init (power cycles required):
     ${RL_ROOT}/startup_scripts/startInitRoverPI.sh --phase1
     # power-cycle DA
     ${RL_ROOT}/startup_scripts/startInitRoverPI.sh --phase2

  2. Enable companion stack at boot:
     sudo ~/deploy_pi5/install-companion-boot.sh <drone_id>
     Manual start: ${RL_ROOT}/startup_scripts/start_companion_drone_tmux.sh <drone_id>

  3. Attach tmux:
     tmux attach -t catswarm_sim

  4. QGroundControl → Pi IP, TCP port 5760

Logs: journalctl -u mavlink-server -f
       journalctl -u companion-drone -f

Configure mavlink-server (interactive):
  sudo ~/deploy_pi5/mavlink-server-configuration.sh
  sudo ~/deploy_pi5/mavlink-server-configuration.sh --fleet-preset
EOF
  if [ "${WITH_ML}" -eq 0 ]; then
    echo "  ML extras: re-run with --with-ml to add opencv/plotly"
  fi
  if [ "${PIP_VIA_DEV_MODE}" -eq 1 ]; then
    echo "  Pip packages installed via dev PC ${PIP_VIA_DEV_USER}@${PIP_VIA_DEV_HOST}"
  else
    echo "  Offline Pi: sudo $0 --from-dev=user@DEV_PC_IP  (repos + pip via dev PC)"
  fi
}

NEED_UART_REBOOT=0

main() {
  progress_init "$(progress_count_pi_steps)"
  case "${PHASE}" in
    all)
      phase_system
      phase_repos
      phase_mavlink
      phase_python
      phase_build
      phase_verify
      ;;
    system) phase_system ;;
    peripherals) phase_peripherals ;;
    repos) phase_repos ;;
    mavlink) phase_mavlink ;;
    companion) phase_companion ;;
    python) phase_python ;;
    build) phase_build ;;
    verify) phase_verify ;;
    *) die "unknown --phase=${PHASE}" ;;
  esac

  if progress_enabled; then
    _PROGRESS_DONE="${_PROGRESS_TOTAL}"
    progress_tick "deployment complete"
    echo ""
  fi

  if [ "${PHASE}" = "all" ] || [ "${PHASE}" = "verify" ]; then
    print_next_steps
  fi
  if [ "${NEED_UART_REBOOT:-0}" -eq 1 ] && [ "${SKIP_REBOOT_HINT}" -eq 0 ]; then
    log "UART overlays changed — reboot required before /dev/ttyAMA* fleet devices appear"
  fi
}

main "$@"
