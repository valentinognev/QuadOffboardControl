#!/usr/bin/env bash
# Companion GPS rover module: LC29H on USB (ea path) or DA on UART (da path).
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
#   ROVER_PORT                  USB serial when module=ea (default: /dev/ttyUSB0)
#   ROVER_BAUD                  USB baud when module=ea (EA=460800, DA USB=115200)
#   ROVER_PORT_UART             DA UART serial when module=da (default: /dev/ttyAMA4)
#   ROVER_BAUD_UART             DA baud when module=da (default: 115200)
#   DA_RATE_MS                  $PAIR050 NMEA interval ms (DA=1000 → 1 Hz; EA=100 → 10 Hz)
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
: "${DA_RATE_MS:=}"
: "${COMPANION_PX4_GPS_PORT:=/dev/ttyAMA0}"
: "${COMPANION_GPS_STATE_FILE:=${HOME}/.config/companion-gps}"
# Fleet standard: NMEA→PX4 on UART0 (GPIO14 TX). Legacy installs saved ttyAMA4.
: "${COMPANION_PX4_GPS_PORT_REQUIRED:=/dev/ttyAMA0}"
: "${COMPANION_DA_UART_REQUIRED:=/dev/ttyAMA4}"

companion_gps_module_label() {
    case "${COMPANION_GPS_MODULE}" in
        ea) echo "USB rover (${ROVER_PORT} @ ${ROVER_BAUD}, ${DA_RATE_MS:-?} ms)" ;;
        da) echo "DA UART (${ROVER_PORT_UART} @ ${ROVER_BAUD_UART}, ${DA_RATE_MS:-1000} ms)" ;;
        *) echo "${COMPANION_GPS_MODULE:-unset}" ;;
    esac
}

# Pick $PAIR050 interval: DA (115200) → 1 Hz; EA (460800) → 10 Hz.
companion_gps_default_rate_ms() {
    local baud="${1:-}"
    case "${baud}" in
        115200) echo 1000 ;;
        *) echo 100 ;;
    esac
}

companion_gps_state_file() {
    printf '%s\n' "${COMPANION_GPS_STATE_FILE}"
}

# Migrate legacy UART roles and persist ~/.config/companion-gps.
# Returns 0 always unless COMPANION_VALIDATE_UART_STRICT=1 and a required device is missing.
companion_gps_ensure_ports() {
    local migrated=0
    local strict="${COMPANION_VALIDATE_UART_STRICT:-0}"
    local px4_need="${COMPANION_PX4_GPS_PORT_REQUIRED}"
    local da_need="${COMPANION_DA_UART_REQUIRED}"
    local missing=0
    local dev

    : "${COMPANION_PX4_GPS_PORT:=${px4_need}}"
    : "${ROVER_PORT_UART:=${da_need}}"

    # Legacy: NMEA→PX4 was UART4; fleet wiring moved TX to UART0 / GPIO14.
    if [[ "${COMPANION_PX4_GPS_PORT}" == "/dev/ttyAMA4" ]]; then
        echo "companion_gps: migrating COMPANION_PX4_GPS_PORT /dev/ttyAMA4 → ${px4_need}" >&2
        COMPANION_PX4_GPS_PORT="${px4_need}"
        migrated=1
    fi
    if [[ -z "${COMPANION_PX4_GPS_PORT}" ]]; then
        COMPANION_PX4_GPS_PORT="${px4_need}"
        migrated=1
    fi
    # Avoid DA rover and PX4 NMEA sharing UART0.
    if [[ "${ROVER_PORT_UART}" == "/dev/ttyAMA0" && "${COMPANION_PX4_GPS_PORT}" == "/dev/ttyAMA0" ]]; then
        echo "companion_gps: migrating ROVER_PORT_UART /dev/ttyAMA0 → ${da_need} (PX4 owns UART0)" >&2
        ROVER_PORT_UART="${da_need}"
        migrated=1
    fi
    if [[ "${COMPANION_DA_PORT:-}" == "/dev/ttyAMA0" && "${COMPANION_PX4_GPS_PORT}" == "/dev/ttyAMA0" ]]; then
        COMPANION_DA_PORT="${da_need}"
        migrated=1
    fi

    if [[ "${migrated}" -eq 1 ]] || [[ ! -f "$(companion_gps_state_file)" ]]; then
        : "${COMPANION_GPS_MODULE:=ea}"
        companion_gps_save_module
        if [[ "${migrated}" -eq 1 ]]; then
            echo "companion_gps: saved UART roles → $(companion_gps_state_file)" >&2
        else
            echo "companion_gps: wrote defaults → $(companion_gps_state_file)" >&2
        fi
        echo "  PX4 NMEA: ${COMPANION_PX4_GPS_PORT}" >&2
        echo "  DA UART:  ${ROVER_PORT_UART}" >&2
    fi

    for dev in "${COMPANION_PX4_GPS_PORT}" /dev/ttyAMA2 /dev/ttyAMA3; do
        if [[ ! -e "${dev}" ]]; then
            echo "companion_gps: WARNING: ${dev} not present (overlays/reboot/wiring?)" >&2
            missing=1
        else
            echo "companion_gps: OK ${dev}" >&2
        fi
    done
    case "$(printf '%s' "${COMPANION_GPS_MODULE:-ea}" | tr '[:upper:]' '[:lower:]')" in
        da|uart|serial|d)
            if [[ ! -e "${ROVER_PORT_UART}" ]]; then
                echo "companion_gps: WARNING: DA rover ${ROVER_PORT_UART} not present" >&2
                missing=1
            fi
            ;;
        *)
            if [[ ! -e "${ROVER_PORT:-/dev/ttyUSB0}" ]]; then
                echo "companion_gps: NOTE: EA rover ${ROVER_PORT:-/dev/ttyUSB0} not present yet (USB)" >&2
            fi
            ;;
    esac

    if [[ "${COMPANION_PX4_GPS_PORT}" != "${px4_need}" ]]; then
        echo "companion_gps: WARNING: COMPANION_PX4_GPS_PORT=${COMPANION_PX4_GPS_PORT} (fleet standard is ${px4_need})" >&2
    fi

    export COMPANION_PX4_GPS_PORT ROVER_PORT_UART ROVER_PORT ROVER_BAUD ROVER_BAUD_UART DA_RATE_MS
    if [[ "${strict}" == "1" && "${missing}" -eq 1 ]]; then
        echo "companion_gps: ERROR: UART validation failed (COMPANION_VALIDATE_UART_STRICT=1)" >&2
        return 1
    fi
    return 0
}

companion_gps_load_saved() {
    local state_file
    local saved_module="" saved_rover_port="" saved_rover_baud=""
    local saved_rover_port_uart="" saved_rover_baud_uart="" saved_da_rate_ms=""
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
    saved_da_rate_ms="${DA_RATE_MS:-}"
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
    if [[ -n "${saved_da_rate_ms}" ]]; then
        DA_RATE_MS="${saved_da_rate_ms}"
    fi
    # COMPANION_PX4_GPS_PORT is set by sourcing the state file (if present).
    return 0
}

companion_gps_save_module() {
    local state_file
    local rate_baud="${ROVER_BAUD}"
    state_file="$(companion_gps_state_file)"
    mkdir -p "$(dirname "${state_file}")"
    case "$(printf '%s' "${COMPANION_GPS_MODULE:-ea}" | tr '[:upper:]' '[:lower:]')" in
        da|uart|serial|d) rate_baud="${ROVER_BAUD_UART}" ;;
    esac
    if [[ -z "${DA_RATE_MS:-}" ]]; then
        DA_RATE_MS="$(companion_gps_default_rate_ms "${rate_baud}")"
    fi
    cat > "${state_file}" <<EOF
# Last companion GPS rover module (edit or use switch_EAUSB_DAUART.sh --ea|--da)
# USB path: EA @ 460800/10Hz or DA @ 115200/1Hz (set ROVER_BAUD + DA_RATE_MS).
COMPANION_GPS_MODULE=${COMPANION_GPS_MODULE}
ROVER_PORT=${ROVER_PORT}
ROVER_BAUD=${ROVER_BAUD}
ROVER_PORT_UART=${ROVER_PORT_UART}
ROVER_BAUD_UART=${ROVER_BAUD_UART}
DA_RATE_MS=${DA_RATE_MS}
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
    # Always migrate/validate UART roles after resolve (persists AMA4→AMA0 etc.).
    companion_gps_ensure_ports || true
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
            echo "  Module: USB rover (EA @ 460800/10Hz or DA @ 115200/1Hz)"
            echo "  Port:   ${ROVER_PORT} @ ${ROVER_BAUD}"
            echo "  Rate:   ${DA_RATE_MS} ms (\$PAIR050)"
            echo "  PX4:    NMEA → ${COMPANION_PX4_GPS_PORT}"
            echo "  Stack:  startRtkCommPI.sh → rover_zmq + emulate_gps_to_px4"
            ;;
        da)
            echo "  Module: DA (UART flight rover)"
            echo "  Port:   ${ROVER_PORT_UART} @ ${ROVER_BAUD_UART}"
            echo "  Rate:   ${DA_RATE_MS:-1000} ms (\$PAIR050; DA native 1 Hz)"
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
    if [[ -z "${DA_RATE_MS:-}" ]]; then
        DA_RATE_MS="$(companion_gps_default_rate_ms "${COMPANION_ROVER_BAUD}")"
    fi
    export COMPANION_GPS_MODULE COMPANION_GPS_WINDOW COMPANION_ROVER_PORT COMPANION_ROVER_BAUD \
        COMPANION_USE_PX4_NMEA ROVER_PORT ROVER_BAUD ROVER_PORT_UART ROVER_BAUD_UART \
        DA_RATE_MS COMPANION_PX4_GPS_PORT
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
        --da-rate-ms="${DA_RATE_MS}"
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

# Write a full fleet profile into ~/.config/companion-gps (flush, not merge).
# Profiles: ea | da-usb | da-uart
companion_gps_write_fleet_profile() {
    local profile
    profile="$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')"
    ROVER_PORT="${ROVER_PORT:-/dev/ttyUSB0}"
    ROVER_PORT_UART="${ROVER_PORT_UART:-/dev/ttyAMA4}"
    ROVER_BAUD_UART="${ROVER_BAUD_UART:-115200}"
    COMPANION_PX4_GPS_PORT="${COMPANION_PX4_GPS_PORT:-/dev/ttyAMA0}"
    case "${profile}" in
        ea|ea-usb)
            COMPANION_GPS_MODULE=ea
            ROVER_BAUD=460800
            DA_RATE_MS=100
            ;;
        da-usb|da_usb|usb-da)
            COMPANION_GPS_MODULE=ea
            ROVER_BAUD=115200
            DA_RATE_MS=1000
            ;;
        da|da-uart|da_uart|uart-da)
            COMPANION_GPS_MODULE=da
            ROVER_BAUD="${ROVER_BAUD:-460800}"
            ROVER_BAUD_UART=115200
            DA_RATE_MS=1000
            ;;
        *)
            echo "companion_gps_write_fleet_profile: unknown profile '${1:-}' (use ea|da-usb|da-uart)" >&2
            return 1
            ;;
    esac
    companion_gps_save_module
    companion_gps_apply_module
    echo "companion_gps: flushed fleet profile=${profile} → $(companion_gps_state_file)" >&2
    companion_gps_show_current_choice "fleet-flush"
}
