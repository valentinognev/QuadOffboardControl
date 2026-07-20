#!/usr/bin/env bash
# Companion GPS rover module: LC29H EA on USB, or LC29H DA on UART1.
#
# Source this file, then call:
#   companion_gps_resolve_module [save]
#   companion_gps_apply_module
#   companion_gps_start_in_tmux SESSION RTK_ZMQ_URL [PYTHON] [CATSWARM_ROOT]
#
# Mode is remembered in ~/.config/companion-gps (override with COMPANION_GPS_STATE_FILE).
# Change manually: switch_EAUSB_DAUART.sh --ea | --da
#
# Environment (optional overrides):
#   COMPANION_GPS_MODULE        ea | da  (explicit one-shot; use save flag to persist)
#   ROVER_PORT                  EA USB serial when module=ea (default: /dev/ttyUSB0)
#   ROVER_BAUD                  EA baud when module=ea (default: 460800)
#   ROVER_PORT_UART             DA UART serial when module=da (default: /dev/ttyAMA0)
#   ROVER_BAUD_UART             DA baud when module=da (default: 115200)
#   COMPANION_PX4_GPS_PORT      NMEA to PX4 (default: /dev/ttyAMA0 = UART0 GPIO14 TX)
#   COMPANION_GPS_WINDOW        tmux window override
#   COMPANION_GPS_STATE_FILE    Persistence file (default: ~/.config/companion-gps)
#
# Launcher scripts use --rover-port / --rover-baud (see util/gnss_serial_args.sh).

: "${COMPANION_GPS_MODULE:=}"
: "${ROVER_PORT:=/dev/ttyUSB0}"
: "${ROVER_BAUD:=460800}"
: "${ROVER_PORT_UART:=/dev/ttyAMA4}"
: "${ROVER_BAUD_UART:=115200}"
: "${COMPANION_PX4_GPS_PORT:=/dev/ttyAMA0}"
: "${COMPANION_GPS_STATE_FILE:=${HOME}/.config/companion-gps}"

companion_gps_module_label() {
    case "${COMPANION_GPS_MODULE}" in
        ea) echo "EA USB (${ROVER_PORT} @ ${ROVER_BAUD})" ;;
        da) echo "DA UART (${ROVER_PORT_UART} @ ${ROVER_BAUD_UART})" ;;
        *) echo "${COMPANION_GPS_MODULE:-unset}" ;;
    esac
}

companion_gps_state_file() {
    printf '%s\n' "${COMPANION_GPS_STATE_FILE}"
}

companion_gps_load_saved() {
    local state_file
    local saved_module="" saved_rover_port="" saved_rover_baud=""
    local saved_rover_port_uart="" saved_rover_baud_uart=""
    local legacy_ea_port="" legacy_ea_baud="" legacy_da_port="" legacy_da_baud=""
    state_file="$(companion_gps_state_file)"
    if [[ ! -f "${state_file}" ]]; then
        return 1
    fi
    # shellcheck disable=SC1090
    source "${state_file}"
    saved_module="${COMPANION_GPS_MODULE:-}"
    saved_rover_port="${ROVER_PORT:-}"
    saved_rover_baud="${ROVER_BAUD:-}"
    saved_rover_port_uart="${ROVER_PORT_UART:-}"
    saved_rover_baud_uart="${ROVER_BAUD_UART:-}"
    legacy_ea_port="${COMPANION_EA_PORT:-}"
    legacy_ea_baud="${COMPANION_EA_BAUD:-}"
    legacy_da_port="${COMPANION_DA_PORT:-}"
    legacy_da_baud="${COMPANION_DA_BAUD:-}"

    if [[ -z "${COMPANION_GPS_MODULE:-}" && -n "${saved_module}" ]]; then
        COMPANION_GPS_MODULE="${saved_module}"
    fi
    if [[ -n "${saved_rover_port}" ]]; then
        ROVER_PORT="${saved_rover_port}"
    elif [[ -n "${legacy_ea_port}" ]]; then
        ROVER_PORT="${legacy_ea_port}"
    fi
    if [[ -n "${saved_rover_baud}" ]]; then
        ROVER_BAUD="${saved_rover_baud}"
    elif [[ -n "${legacy_ea_baud}" ]]; then
        ROVER_BAUD="${legacy_ea_baud}"
    fi
    if [[ -n "${saved_rover_port_uart}" ]]; then
        ROVER_PORT_UART="${saved_rover_port_uart}"
    elif [[ -n "${legacy_da_port}" ]]; then
        ROVER_PORT_UART="${legacy_da_port}"
    fi
    if [[ -n "${saved_rover_baud_uart}" ]]; then
        ROVER_BAUD_UART="${saved_rover_baud_uart}"
    elif [[ -n "${legacy_da_baud}" ]]; then
        ROVER_BAUD_UART="${legacy_da_baud}"
    fi
    return 0
}

companion_gps_save_module() {
    local state_file
    state_file="$(companion_gps_state_file)"
    mkdir -p "$(dirname "${state_file}")"
    cat > "${state_file}" <<EOF
# Last companion GPS rover module (edit or use switch_EAUSB_DAUART.sh --ea|--da)
COMPANION_GPS_MODULE=${COMPANION_GPS_MODULE}
ROVER_PORT=${ROVER_PORT}
ROVER_BAUD=${ROVER_BAUD}
ROVER_PORT_UART=${ROVER_PORT_UART}
ROVER_BAUD_UART=${ROVER_BAUD_UART}
COMPANION_PX4_GPS_PORT=${COMPANION_PX4_GPS_PORT}
EOF
    chmod 0644 "${state_file}" 2>/dev/null || true
}

companion_gps_resolve_module() {
    local do_save="${1:-}"
    local explicit_module=""
    local saved=0

    COMPANION_GPS_SOURCE=""

    if [[ -n "${COMPANION_GPS_MODULE:-}" ]]; then
        explicit_module="$(printf '%s' "${COMPANION_GPS_MODULE}" | tr '[:upper:]' '[:lower:]')"
    fi

    if [[ -z "${explicit_module}" ]]; then
        if companion_gps_load_saved; then
            saved=1
            explicit_module="$(printf '%s' "${COMPANION_GPS_MODULE:-}" | tr '[:upper:]' '[:lower:]')"
        fi
    fi

    if [[ -z "${explicit_module}" ]]; then
        COMPANION_GPS_MODULE=ea
        COMPANION_GPS_SOURCE=default
    elif [[ -n "${COMPANION_GPS_MODULE:-}" && "${saved}" -eq 0 ]]; then
        COMPANION_GPS_MODULE="${explicit_module}"
        if [[ "${do_save}" == "save" ]]; then
            COMPANION_GPS_SOURCE=explicit
        else
            COMPANION_GPS_SOURCE=explicit-once
        fi
    else
        COMPANION_GPS_MODULE="${explicit_module}"
        COMPANION_GPS_SOURCE=saved
    fi

    if [[ "${do_save}" == "save" && -n "${COMPANION_GPS_MODULE:-}" ]]; then
        companion_gps_save_module
        if [[ "${COMPANION_GPS_SOURCE}" != "explicit-once" ]]; then
            COMPANION_GPS_SOURCE=explicit
        fi
    fi
    export COMPANION_GPS_SOURCE
}

companion_gps_show_current_choice() {
    local source="${1:-}"
    local state_file
    state_file="$(companion_gps_state_file)"
    echo ""
    echo "━━━━━━━━ GPS rover module ━━━━━━━━"
    case "${COMPANION_GPS_MODULE}" in
        ea)
            echo "  Module: EA (USB bench / primary rover)"
            echo "  Port:   ${ROVER_PORT} @ ${ROVER_BAUD}"
            echo "  PX4:    NMEA → ${COMPANION_PX4_GPS_PORT}"
            echo "  Stack:  startRtkCommPI.sh (EA USB) → rover_zmq + emulate_gps_to_px4"
            ;;
        da)
            echo "  Module: DA (UART flight rover)"
            echo "  Port:   ${ROVER_PORT_UART} @ ${ROVER_BAUD_UART}"
            echo "  PX4:    NMEA → ${COMPANION_PX4_GPS_PORT}"
            echo "  Stack:  startRtkCommPI.sh → rover_zmq + emulate_gps_to_px4"
            ;;
        *)
            echo "  Module: ${COMPANION_GPS_MODULE:-unset}"
            ;;
    esac
    case "${source}" in
        saved) echo "  Source: saved preference (${state_file})" ;;
        default) echo "  Source: default (no saved preference yet)" ;;
        explicit) echo "  Source: command-line override (saved)" ;;
        explicit-once) echo "  Source: command-line override (not saved)" ;;
        *) echo "  Source: ${state_file}" ;;
    esac
    echo "  Change: switch_EAUSB_DAUART.sh --ea | --da"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
}

companion_gps_apply_module() {
    local mode
    mode="$(printf '%s' "${COMPANION_GPS_MODULE:-ea}" | tr '[:upper:]' '[:lower:]')"
    case "${mode}" in
        ea|usb|e)
            COMPANION_GPS_MODULE=ea
            COMPANION_GPS_WINDOW=gps_ea
            COMPANION_ROVER_PORT="${ROVER_PORT}"
            COMPANION_ROVER_BAUD="${ROVER_BAUD}"
            COMPANION_USE_PX4_NMEA=1
            ;;
        da|uart|serial|d)
            COMPANION_GPS_MODULE=da
            COMPANION_GPS_WINDOW=gps_rtk
            COMPANION_ROVER_PORT="${ROVER_PORT_UART}"
            COMPANION_ROVER_BAUD="${ROVER_BAUD_UART}"
            COMPANION_USE_PX4_NMEA=1
            ;;
        *)
            echo "companion_gps_module: unknown COMPANION_GPS_MODULE=${COMPANION_GPS_MODULE}; using ea" >&2
            COMPANION_GPS_MODULE=ea
            COMPANION_GPS_WINDOW=gps_ea
            COMPANION_ROVER_PORT="${ROVER_PORT}"
            COMPANION_ROVER_BAUD="${ROVER_BAUD}"
            COMPANION_USE_PX4_NMEA=1
            ;;
    esac
    export COMPANION_GPS_MODULE COMPANION_GPS_WINDOW COMPANION_ROVER_PORT COMPANION_ROVER_BAUD \
        COMPANION_USE_PX4_NMEA ROVER_PORT ROVER_BAUD ROVER_PORT_UART ROVER_BAUD_UART \
        COMPANION_PX4_GPS_PORT
}

companion_gps_launcher_path() {
    local root="${1:-}"
    companion_gps_apply_module
    # Both EA USB and DA UART use startRtkCommPI.sh (rover port/baud select the module).
    # Legacy name startRtkEaUSB.sh was never shipped in GPS_RTK; keep messages for operators.
    printf '%s/GPS_RTK/startRtkCommPI.sh\n' "${root}"
}

companion_gps_start_in_tmux() {
    local session="$1"
    local rtk_zmq_url="$2"
    local python="${3:-python3}"
    local root="${4:-}"
    local launcher args=()

    if [[ -z "${root}" ]]; then
        echo "companion_gps_start_in_tmux: CATSWARM_ROOT required" >&2
        return 1
    fi

    companion_gps_apply_module
    launcher="$(companion_gps_launcher_path "${root}")"
    if [[ ! -f "${launcher}" ]]; then
        echo "companion_gps_start_in_tmux: launcher not found: ${launcher}" >&2
        return 1
    fi

    args=(
        --rtk-zmq-url="${rtk_zmq_url}"
        --rover-port="${COMPANION_ROVER_PORT}"
        --rover-baud="${COMPANION_ROVER_BAUD}"
        --px4-port="${COMPANION_PX4_GPS_PORT}"
        --join-session
        --no-attach
        --session="${session}"
        --window="${COMPANION_GPS_WINDOW}"
        --python="${python}"
    )
    if [[ "${COMPANION_USE_PX4_NMEA:-1}" == "0" ]]; then
        args+=(--no-px4)
    fi

    echo "Starting GPS stack (${COMPANION_GPS_MODULE}) in ${session}:${COMPANION_GPS_WINDOW}…" >&2
    "${launcher}" "${args[@]}"
}
