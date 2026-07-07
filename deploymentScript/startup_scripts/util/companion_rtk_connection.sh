#!/usr/bin/env bash
# Companion RTK path: WiFi/LAN ZMQ to GS PC, or serial RF via UART2 + local ZMQ bridge.
#
# Source this file, then call:
#   companion_rtk_resolve_mode [save]
#   companion_rtk_apply_mode
#
# Mode is remembered in ~/.config/companion-rtk (override with COMPANION_RTK_STATE_FILE).
# Change manually: switch_rtk_WIFI_RF.sh --wifi|--serial [--base-host=IP]
#
# Environment (optional overrides):
#   COMPANION_RTK_MODE          wifi | serial  (explicit one-shot; use save flag to persist)
#   COMPANION_BASE_HOST         GS IP for WiFi (default: 192.168.0.43)
#   COMPANION_BASE_PORT_NUM     GS ZMQ port (default: 5560)
#   COMPANION_RTK_ZMQ_BIND      Local RF bridge (default: tcp://127.0.0.1:5562)
#   COMPANION_RTK_STATE_FILE    Persistence file (default: ~/.config/companion-rtk)
#
# Exports after companion_rtk_apply_mode:
#   COMPANION_RTK_MODE          wifi | serial
#   COMPANION_RTK_ZMQ_URL       rover_zmq --connect target
#   COMPANION_USE_RF_RTK_BRIDGE 1 = ZMQ_to_comm --rtk-zmq-bind; 0 = WiFi direct

: "${COMPANION_BASE_HOST:=192.168.0.43}"
: "${COMPANION_BASE_PORT_NUM:=5560}"
: "${COMPANION_RTK_ZMQ_BIND:=tcp://127.0.0.1:5562}"
: "${COMPANION_RTK_MODE:=}"
: "${COMPANION_RTK_STATE_FILE:=${HOME}/.config/companion-rtk}"

companion_rtk_mode_label() {
    case "${COMPANION_RTK_MODE}" in
        wifi) echo "WiFi/LAN (${COMPANION_BASE_HOST}:${COMPANION_BASE_PORT_NUM})" ;;
        serial) echo "serial RF (${COMPANION_RTK_ZMQ_BIND})" ;;
        *) echo "${COMPANION_RTK_MODE:-unset}" ;;
    esac
}

companion_rtk_show_current_choice() {
    local source="${1:-}"
    local state_file
    state_file="$(companion_rtk_state_file)"
    echo ""
    echo "━━━━━━━━ RTK base connection ━━━━━━━━"
    case "${COMPANION_RTK_MODE}" in
        wifi)
            echo "  Mode:   WiFi/LAN"
            echo "  Target: ${COMPANION_RTK_ZMQ_URL}"
            echo "  GS:     startRtkWiFiGS.sh on ${COMPANION_BASE_HOST}"
            ;;
        serial)
            echo "  Mode:   Serial RF"
            echo "  Path:   UART2 → ZMQ_to_comm → ${COMPANION_RTK_ZMQ_URL} → rover_zmq"
            echo "  GS:     startRtkCommGS.sh (radio link)"
            ;;
        *)
            echo "  Mode:   ${COMPANION_RTK_MODE:-unset}"
            echo "  Target: ${COMPANION_RTK_ZMQ_URL:-<unknown>}"
            ;;
    esac
    case "${source}" in
        saved) echo "  Source: saved preference (${state_file})" ;;
        default) echo "  Source: default (no saved preference yet)" ;;
        explicit) echo "  Source: command-line override (saved)" ;;
        explicit-once) echo "  Source: command-line override (not saved)" ;;
        *) echo "  Source: ${state_file}" ;;
    esac
    echo "  Change: switch_rtk_WIFI_RF.sh --wifi | --serial [--base-host=IP]"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
}

companion_rtk_state_file() {
    printf '%s\n' "${COMPANION_RTK_STATE_FILE}"
}

companion_rtk_load_saved() {
    local state_file
    local saved_mode="" saved_host="" saved_port="" saved_bind=""
    state_file="$(companion_rtk_state_file)"
    if [[ ! -f "${state_file}" ]]; then
        return 1
    fi
    # shellcheck disable=SC1090
    source "${state_file}"
    saved_mode="${COMPANION_RTK_MODE:-}"
    saved_host="${COMPANION_BASE_HOST:-}"
    saved_port="${COMPANION_BASE_PORT_NUM:-}"
    saved_bind="${COMPANION_RTK_ZMQ_BIND:-}"

    if [[ -z "${COMPANION_RTK_MODE:-}" && -n "${saved_mode}" ]]; then
        COMPANION_RTK_MODE="${saved_mode}"
    fi
    if [[ "${COMPANION_RTK_HOST_EXPLICIT:-0}" -ne 1 && -n "${saved_host}" ]]; then
        COMPANION_BASE_HOST="${saved_host}"
    fi
    if [[ -n "${saved_port}" ]]; then
        COMPANION_BASE_PORT_NUM="${saved_port}"
    fi
    if [[ -n "${saved_bind}" ]]; then
        COMPANION_RTK_ZMQ_BIND="${saved_bind}"
    fi
    return 0
}

companion_rtk_save_mode() {
    local state_file
    state_file="$(companion_rtk_state_file)"
    mkdir -p "$(dirname "${state_file}")"
    cat > "${state_file}" <<EOF
# Last companion RTK connection (edit or use switch_rtk_WIFI_RF.sh --wifi|--serial)
COMPANION_RTK_MODE=${COMPANION_RTK_MODE}
COMPANION_BASE_HOST=${COMPANION_BASE_HOST}
COMPANION_BASE_PORT_NUM=${COMPANION_BASE_PORT_NUM}
COMPANION_RTK_ZMQ_BIND=${COMPANION_RTK_ZMQ_BIND}
EOF
    chmod 0644 "${state_file}" 2>/dev/null || true
}

# Resolve mode: explicit env/CLI > saved file > WiFi default.
# Pass "save" to persist explicit COMPANION_RTK_MODE (and BASE_HOST if set).
# Sets COMPANION_RTK_SOURCE for companion_rtk_show_current_choice.
companion_rtk_resolve_mode() {
    local do_save="${1:-}"
    local explicit_mode=""
    local saved=0

    COMPANION_RTK_SOURCE=""

    if [[ -n "${COMPANION_RTK_MODE:-}" ]]; then
        explicit_mode="$(printf '%s' "${COMPANION_RTK_MODE}" | tr '[:upper:]' '[:lower:]')"
        case "${explicit_mode}" in
            ask)
                explicit_mode=""
                ;;
        esac
    fi

    if [[ -z "${explicit_mode}" ]]; then
        if companion_rtk_load_saved; then
            saved=1
            explicit_mode="$(printf '%s' "${COMPANION_RTK_MODE:-}" | tr '[:upper:]' '[:lower:]')"
        fi
    fi

    if [[ -z "${explicit_mode}" ]]; then
        COMPANION_RTK_MODE=wifi
        COMPANION_RTK_SOURCE=default
    elif [[ -n "${COMPANION_RTK_MODE:-}" && "${saved}" -eq 0 ]]; then
        COMPANION_RTK_MODE="${explicit_mode}"
        if [[ "${do_save}" == "save" ]]; then
            COMPANION_RTK_SOURCE=explicit
        else
            COMPANION_RTK_SOURCE=explicit-once
        fi
    else
        COMPANION_RTK_MODE="${explicit_mode}"
        COMPANION_RTK_SOURCE=saved
    fi

    if [[ "${do_save}" == "save" && -n "${COMPANION_RTK_MODE:-}" ]]; then
        companion_rtk_save_mode
        if [[ "${COMPANION_RTK_SOURCE}" != "explicit-once" ]]; then
            COMPANION_RTK_SOURCE=explicit
        fi
    fi
    export COMPANION_RTK_SOURCE
}

companion_rtk_apply_mode() {
    local mode
    mode="$(printf '%s' "${COMPANION_RTK_MODE:-wifi}" | tr '[:upper:]' '[:lower:]')"
    case "${mode}" in
        wifi|lan|zmq|w)
            COMPANION_RTK_MODE=wifi
            COMPANION_RTK_ZMQ_URL="tcp://${COMPANION_BASE_HOST}:${COMPANION_BASE_PORT_NUM}"
            COMPANION_USE_RF_RTK_BRIDGE=0
            ;;
        serial|rf|radio|s)
            COMPANION_RTK_MODE=serial
            COMPANION_RTK_ZMQ_URL="${COMPANION_RTK_ZMQ_BIND}"
            COMPANION_USE_RF_RTK_BRIDGE=1
            ;;
        *)
            echo "companion_rtk_connection: unknown COMPANION_RTK_MODE=${COMPANION_RTK_MODE}; using wifi" >&2
            COMPANION_RTK_MODE=wifi
            COMPANION_RTK_ZMQ_URL="tcp://${COMPANION_BASE_HOST}:${COMPANION_BASE_PORT_NUM}"
            COMPANION_USE_RF_RTK_BRIDGE=0
            ;;
    esac
    export COMPANION_RTK_MODE COMPANION_RTK_ZMQ_URL COMPANION_USE_RF_RTK_BRIDGE \
        COMPANION_BASE_HOST COMPANION_BASE_PORT_NUM COMPANION_RTK_ZMQ_BIND
}
