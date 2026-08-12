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
#   COMPANION_RTK_SINK          mavlink_rtcm | rover_uart  (default rover_uart; persist in state file)
#   COMPANION_MAVLINK_RTCM      mavlink dest for rtcm_to_mavlink (default udpout:127.0.0.1:14580)
#   COMPANION_BASE_HOST         GS IP for WiFi (default: 192.168.0.43)
#   COMPANION_BASE_PORT_NUM     GS ZMQ port (default: 5560)
#   COMPANION_RTK_ZMQ_BIND      Local RF bridge (default: tcp://127.0.0.1:5562)
#   COMPANION_RTK_STATE_FILE    Persistence file (default: ~/.config/companion-rtk)
#
# Exports after companion_rtk_apply_mode:
#   COMPANION_RTK_MODE          wifi | serial
#   COMPANION_RTK_SINK          mavlink_rtcm | rover_uart
#   COMPANION_RTK_ZMQ_URL       rover_zmq --connect target (rover_uart path)
#   COMPANION_USE_RF_RTK_BRIDGE 1 = ZMQ_to_comm --rtk-zmq-bind (serial / RF); 0 = WiFi direct
#                               (mavlink_rtcm + serial also needs :5562 for rf-only bridge)

: "${COMPANION_BASE_HOST:=192.168.0.43}"
: "${COMPANION_BASE_PORT_NUM:=5560}"
: "${COMPANION_RTK_ZMQ_BIND:=tcp://127.0.0.1:5562}"
: "${COMPANION_RTK_MODE:=}"
: "${COMPANION_RTK_SINK:=}"
: "${COMPANION_MAVLINK_RTCM:=udpout:127.0.0.1:14580}"
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
    local sink_label
    state_file="$(companion_rtk_state_file)"
    companion_rtk_resolve_sink
    case "${COMPANION_RTK_SINK}" in
        mavlink_rtcm) sink_label="mavlink_rtcm (rtcm_to_mavlink → ${COMPANION_MAVLINK_RTCM})" ;;
        *) sink_label="rover_uart (rover_zmq + emulate_gps_to_px4)" ;;
    esac
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
    echo "  Sink:   ${sink_label}"
    case "${source}" in
        saved) echo "  Source: saved preference (${state_file})" ;;
        default) echo "  Source: default (no saved preference yet)" ;;
        explicit) echo "  Source: command-line override (saved)" ;;
        explicit-once) echo "  Source: command-line override (not saved)" ;;
        *) echo "  Source: ${state_file}" ;;
    esac
    echo "  Change: switch_rtk_WIFI_RF.sh --wifi | --serial [--base-host=IP] [--sink=mavlink_rtcm|rover_uart]"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
}

companion_rtk_state_file() {
    printf '%s\n' "${COMPANION_RTK_STATE_FILE}"
}

companion_rtk_load_saved() {
    local state_file
    local saved_mode="" saved_host="" saved_port="" saved_bind=""
    local saved_sink="" saved_mavlink=""
    local prior_mode="${COMPANION_RTK_MODE:-}"
    local prior_host="${COMPANION_BASE_HOST:-}"
    local prior_sink="${COMPANION_RTK_SINK:-}"
    local prior_mavlink="${COMPANION_MAVLINK_RTCM:-}"
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
    saved_sink="${COMPANION_RTK_SINK:-}"
    saved_mavlink="${COMPANION_MAVLINK_RTCM:-}"

    if [[ -n "${prior_mode}" ]]; then
        COMPANION_RTK_MODE="${prior_mode}"
    elif [[ -n "${saved_mode}" ]]; then
        COMPANION_RTK_MODE="${saved_mode}"
    fi
    # --base-host= sets COMPANION_RTK_HOST_EXPLICIT; otherwise prefer saved over default.
    if [[ "${COMPANION_RTK_HOST_EXPLICIT:-0}" -eq 1 ]]; then
        COMPANION_BASE_HOST="${prior_host}"
    elif [[ -n "${saved_host}" ]]; then
        COMPANION_BASE_HOST="${saved_host}"
    else
        COMPANION_BASE_HOST="${prior_host}"
    fi
    if [[ -n "${saved_port}" ]]; then
        COMPANION_BASE_PORT_NUM="${saved_port}"
    fi
    if [[ -n "${saved_bind}" ]]; then
        COMPANION_RTK_ZMQ_BIND="${saved_bind}"
    fi
    # Preserve CLI/env sink set before load; otherwise keep saved.
    if [[ -n "${prior_sink}" ]]; then
        COMPANION_RTK_SINK="${prior_sink}"
    elif [[ -n "${saved_sink}" ]]; then
        COMPANION_RTK_SINK="${saved_sink}"
    fi
    # MAVLINK default is always set at file top — only keep prior if caller overrode it
    # (non-default), else prefer saved file value.
    if [[ -n "${prior_mavlink}" && "${prior_mavlink}" != "udpout:127.0.0.1:14580" ]]; then
        COMPANION_MAVLINK_RTCM="${prior_mavlink}"
    elif [[ -n "${saved_mavlink}" ]]; then
        COMPANION_MAVLINK_RTCM="${saved_mavlink}"
    fi
    return 0
}

companion_rtk_save_mode() {
    local state_file
    state_file="$(companion_rtk_state_file)"
    companion_rtk_resolve_sink
    mkdir -p "$(dirname "${state_file}")"
    cat > "${state_file}" <<EOF
# Last companion RTK connection (edit or use switch_rtk_WIFI_RF.sh --wifi|--serial [--sink=])
COMPANION_RTK_MODE=${COMPANION_RTK_MODE}
COMPANION_RTK_SINK=${COMPANION_RTK_SINK}
COMPANION_MAVLINK_RTCM=${COMPANION_MAVLINK_RTCM}
COMPANION_BASE_HOST=${COMPANION_BASE_HOST}
COMPANION_BASE_PORT_NUM=${COMPANION_BASE_PORT_NUM}
COMPANION_RTK_ZMQ_BIND=${COMPANION_RTK_ZMQ_BIND}
EOF
    chmod 0644 "${state_file}" 2>/dev/null || true
}

# When mode was set explicitly (load_saved skipped), still merge sink/mavlink/host from state.
# Preserves saved COMPANION_BASE_HOST unless COMPANION_RTK_HOST_EXPLICIT=1 (--base-host=).
companion_rtk_load_saved_sink_only() {
    local state_file line key val
    state_file="$(companion_rtk_state_file)"
    [[ -f "${state_file}" ]] || return 0
    while IFS= read -r line || [[ -n "${line}" ]]; do
        [[ "${line}" =~ ^[[:space:]]*# ]] && continue
        [[ "${line}" =~ ^[[:space:]]*$ ]] && continue
        key="${line%%=*}"
        val="${line#*=}"
        case "${key}" in
            COMPANION_BASE_HOST)
                if [[ "${COMPANION_RTK_HOST_EXPLICIT:-0}" -ne 1 && -n "${val}" ]]; then
                    COMPANION_BASE_HOST="${val}"
                fi
                ;;
            COMPANION_BASE_PORT_NUM)
                if [[ -n "${val}" ]]; then
                    COMPANION_BASE_PORT_NUM="${val}"
                fi
                ;;
            COMPANION_RTK_ZMQ_BIND)
                if [[ -n "${val}" ]]; then
                    COMPANION_RTK_ZMQ_BIND="${val}"
                fi
                ;;
            COMPANION_RTK_SINK)
                if [[ -z "${COMPANION_RTK_SINK:-}" && -n "${val}" ]]; then
                    COMPANION_RTK_SINK="${val}"
                fi
                ;;
            COMPANION_MAVLINK_RTCM)
                if [[ -n "${val}" && "${COMPANION_MAVLINK_RTCM:-}" == "udpout:127.0.0.1:14580" ]]; then
                    COMPANION_MAVLINK_RTCM="${val}"
                fi
                ;;
        esac
    done < "${state_file}"
}

# Resolve mode: explicit env/CLI > saved file > WiFi default.
# Pass "save" to persist explicit COMPANION_RTK_MODE / sink; BASE_HOST only if --base-host=.
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

    # Mode may be CLI-explicit without load_saved — still merge sink/mavlink from file.
    if [[ "${saved}" -eq 0 ]]; then
        companion_rtk_load_saved_sink_only
    fi

    if [[ "${do_save}" == "save" && -n "${COMPANION_RTK_MODE:-}" ]]; then
        companion_rtk_save_mode
        if [[ "${COMPANION_RTK_SOURCE}" != "explicit-once" ]]; then
            COMPANION_RTK_SOURCE=explicit
        fi
    fi
    export COMPANION_RTK_SOURCE
}

companion_rtk_resolve_sink() {
    local s
    s="$(printf '%s' "${COMPANION_RTK_SINK:-rover_uart}" | tr '[:upper:]' '[:lower:]')"
    case "${s}" in
        mavlink|mavlink_rtcm|fc) COMPANION_RTK_SINK=mavlink_rtcm ;;
        rover|rover_uart|uart|*) COMPANION_RTK_SINK=rover_uart ;;
    esac
    export COMPANION_RTK_SINK
}

companion_rtk_apply_sink_side_effects() {
    companion_rtk_resolve_sink
    # mavlink_rtcm needs local :5562 PUB only for serial/RF mode (not wifi-only).
    if [[ "${COMPANION_RTK_SINK}" == "mavlink_rtcm" && "${COMPANION_RTK_MODE}" == "serial" ]]; then
        COMPANION_USE_RF_RTK_BRIDGE=1
    fi
    export COMPANION_USE_RF_RTK_BRIDGE COMPANION_MAVLINK_RTCM
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
    companion_rtk_apply_sink_side_effects
    export COMPANION_RTK_MODE COMPANION_RTK_ZMQ_URL COMPANION_USE_RF_RTK_BRIDGE \
        COMPANION_BASE_HOST COMPANION_BASE_PORT_NUM COMPANION_RTK_ZMQ_BIND \
        COMPANION_RTK_SINK COMPANION_MAVLINK_RTCM
}
