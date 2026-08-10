#!/usr/bin/env bash
# Companion GPS rover module: LC29H USB (ea) / DA UART (da) / u-blox F9P USB (f9p).
#
# Source this file, then call:
#   companion_gps_resolve_module [save]
#   companion_gps_apply_module
#   companion_gps_start_in_tmux SESSION RTK_ZMQ_URL [PYTHON] [CATSWARM_ROOT]
#
# Mode is remembered in ~/.config/companion-gps (override with COMPANION_GPS_STATE_FILE).
# Change manually: switch_EAUSB_DAUART.sh --ea | --da | --f9p
#
# Environment (optional overrides):
#   COMPANION_GPS_MODULE        ea | da | f9p  (explicit one-shot; use save flag to persist)
#   ROVER_TYPE                  lc29h | f9p (default lc29h; set to f9p with module=f9p)
#   ROVER_PORT                  USB serial when module=ea/f9p (default: /dev/ttyUSB0; f9p ACM: /dev/ttyACM0)
#   ROVER_BAUD                  USB baud when module=ea/f9p (EA=460800, DA USB/F9P ACM=115200, F9P ttyUSB*=230400)
#   ROVER_WIRE                  nmea | ubx (f9p fleet → ubx; LC29H → nmea; passed to startRtkCommPI.sh)
#   ROVER_PORT_UART             DA UART serial when module=da (default: /dev/ttyAMA4)
#   ROVER_BAUD_UART             DA baud when module=da (default: 115200)
#   DA_RATE_MS                  NMEA interval ms (EA=100 → 10 Hz; DA/F9P=1000 → 1 Hz)
#   COMPANION_PX4_GPS_PORT      NMEA to PX4 (default: /dev/ttyAMA0 = UART0 GPIO14 TX)
#   COMPANION_GPS_WINDOW        tmux window override
#   COMPANION_GPS_STATE_FILE    Persistence file (default: ~/.config/companion-gps)
#
# Launcher scripts use --rover-port / --rover-baud (see util/gnss_serial_args.sh).

: "${COMPANION_GPS_MODULE:=}"
: "${ROVER_TYPE:=lc29h}"
: "${ROVER_PORT:=/dev/ttyUSB0}"
: "${ROVER_BAUD:=460800}"
: "${ROVER_WIRE:=nmea}"
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
        f9p) echo "F9P USB (${ROVER_PORT} @ ${ROVER_BAUD}, wire=${ROVER_WIRE:-ubx}, ${DA_RATE_MS:-1000} ms)" ;;
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
        # Do not force module=ea on migrate — that wiped a saved F9P profile and made
        # boot wait on /dev/ttyUSB0 (no tmux for ~30s) when only ACM was plugged.
        if [[ ! -f "$(companion_gps_state_file)" ]]; then
            : "${COMPANION_GPS_MODULE:=ea}"
        fi
        companion_gps_save_module
        if [[ "${migrated}" -eq 1 ]]; then
            echo "companion_gps: saved UART roles → $(companion_gps_state_file) (module=${COMPANION_GPS_MODULE:-ea})" >&2
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
        f9p|zed-f9p|ublox)
            if [[ ! -e "${ROVER_PORT:-/dev/ttyACM0}" ]]; then
                echo "companion_gps: NOTE: F9P rover ${ROVER_PORT:-/dev/ttyACM0} not present yet (USB ACM/USB)" >&2
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

    export COMPANION_PX4_GPS_PORT ROVER_PORT_UART ROVER_PORT ROVER_BAUD ROVER_BAUD_UART \
        ROVER_WIRE DA_RATE_MS
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
    local saved_rover_type="" saved_rover_wire=""
    local legacy_ea_port="" legacy_ea_baud="" legacy_da_port="" legacy_da_baud=""
    state_file="$(companion_gps_state_file)"
    if [[ ! -f "${state_file}" ]]; then
        return 1
    fi
    # shellcheck disable=SC1090
    source "${state_file}"
    saved_module="${COMPANION_GPS_MODULE:-}"
    saved_rover_type="${ROVER_TYPE:-}"
    saved_rover_port="${ROVER_PORT:-}"
    saved_rover_baud="${ROVER_BAUD:-}"
    saved_rover_wire="${ROVER_WIRE:-}"
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
    if [[ -n "${saved_rover_type}" ]]; then
        ROVER_TYPE="${saved_rover_type}"
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
    if [[ -n "${saved_rover_wire}" ]]; then
        ROVER_WIRE="${saved_rover_wire}"
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
        da|uart|serial|d)
            rate_baud="${ROVER_BAUD_UART}"
            : "${ROVER_WIRE:=nmea}"
            ;;
        f9p|zed-f9p|ublox)
            ROVER_TYPE=f9p
            # Absolute: file-top default ROVER_WIRE=nmea must not stick on f9p.
            ROVER_WIRE=ubx
            ;;
        *)
            ROVER_TYPE="${ROVER_TYPE:-lc29h}"
            : "${ROVER_WIRE:=nmea}"
            ;;
    esac
    if [[ -z "${DA_RATE_MS:-}" ]]; then
        DA_RATE_MS="$(companion_gps_default_rate_ms "${rate_baud}")"
    fi
    : "${ROVER_TYPE:=lc29h}"
    : "${ROVER_WIRE:=nmea}"
    cat > "${state_file}" <<EOF
# Last companion GPS rover module (edit or use switch_EAUSB_DAUART.sh --ea|--da|--f9p)
# USB: EA @ 460800/10Hz, DA @ 115200/1Hz, F9P ACM @ 115200/1Hz or F9P ttyUSB* @ 230400/1Hz (ROVER_WIRE=ubx).
COMPANION_GPS_MODULE=${COMPANION_GPS_MODULE}
ROVER_TYPE=${ROVER_TYPE}
ROVER_PORT=${ROVER_PORT}
ROVER_BAUD=${ROVER_BAUD}
ROVER_WIRE=${ROVER_WIRE}
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
            echo "  Type:   ${ROVER_TYPE:-lc29h}"
            echo "  Port:   ${ROVER_PORT} @ ${ROVER_BAUD}"
            echo "  Rate:   ${DA_RATE_MS} ms (\$PAIR050)"
            echo "  PX4:    NMEA → ${COMPANION_PX4_GPS_PORT}"
            echo "  Stack:  startRtkCommPI.sh → rover_zmq + emulate_gps_to_px4"
            ;;
        da)
            echo "  Module: DA (UART flight rover)"
            echo "  Type:   ${ROVER_TYPE:-lc29h}"
            echo "  Port:   ${ROVER_PORT_UART} @ ${ROVER_BAUD_UART}"
            echo "  Rate:   ${DA_RATE_MS:-1000} ms (\$PAIR050; DA native 1 Hz)"
            echo "  PX4:    NMEA → ${COMPANION_PX4_GPS_PORT}"
            echo "  Stack:  startRtkCommPI.sh → rover_zmq + emulate_gps_to_px4"
            ;;
        f9p)
            echo "  Module: u-blox ZED-F9P (USB ACM or USB UART)"
            echo "  Type:   f9p"
            echo "  Port:   ${ROVER_PORT} @ ${ROVER_BAUD}"
            echo "  Wire:   ${ROVER_WIRE:-ubx}"
            echo "  Rate:   ${DA_RATE_MS:-1000} ms (meas rate)"
            echo "  PX4:    NMEA → ${COMPANION_PX4_GPS_PORT}"
            echo "  Stack:  startRtkCommPI.sh --rover-type f9p --rover-wire ${ROVER_WIRE:-ubx}"
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
    echo "  Change: switch_EAUSB_DAUART.sh --ea | --da | --f9p"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
}

companion_gps_apply_module() {
    local mode
    mode="$(printf '%s' "${COMPANION_GPS_MODULE:-ea}" | tr '[:upper:]' '[:lower:]')"
    case "${mode}" in
        ea|usb|e)
            COMPANION_GPS_MODULE=ea
            ROVER_TYPE=lc29h
            : "${ROVER_WIRE:=nmea}"
            COMPANION_GPS_WINDOW=gps_ea
            COMPANION_ROVER_PORT="${ROVER_PORT}"
            COMPANION_ROVER_BAUD="${ROVER_BAUD}"
            COMPANION_USE_PX4_NMEA=1
            ;;
        da|uart|serial|d)
            COMPANION_GPS_MODULE=da
            ROVER_TYPE=lc29h
            : "${ROVER_WIRE:=nmea}"
            COMPANION_GPS_WINDOW=gps_rtk
            COMPANION_ROVER_PORT="${ROVER_PORT_UART}"
            COMPANION_ROVER_BAUD="${ROVER_BAUD_UART}"
            COMPANION_USE_PX4_NMEA=1
            ;;
        f9p|zed-f9p|ublox)
            COMPANION_GPS_MODULE=f9p
            ROVER_TYPE=f9p
            # Absolute: file-top default ROVER_WIRE=nmea / ROVER_BAUD=460800 must not stick on f9p.
            ROVER_WIRE=ubx
            COMPANION_GPS_WINDOW=gps_f9p
            : "${ROVER_PORT:=/dev/ttyACM0}"
            case "${ROVER_PORT}" in
                /dev/ttyUSB*) ROVER_BAUD=230400 ;;
                *) ROVER_BAUD=115200 ;;
            esac
            COMPANION_ROVER_PORT="${ROVER_PORT}"
            COMPANION_ROVER_BAUD="${ROVER_BAUD}"
            COMPANION_USE_PX4_NMEA=1
            ;;
        *)
            echo "companion_gps_module: unknown COMPANION_GPS_MODULE=${COMPANION_GPS_MODULE}; using ea" >&2
            COMPANION_GPS_MODULE=ea
            ROVER_TYPE=lc29h
            : "${ROVER_WIRE:=nmea}"
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
        COMPANION_USE_PX4_NMEA ROVER_TYPE ROVER_PORT ROVER_BAUD ROVER_WIRE ROVER_PORT_UART \
        ROVER_BAUD_UART DA_RATE_MS COMPANION_PX4_GPS_PORT
}

# Boot / start helper: keep saved profile when its rover tty exists; otherwise sniff
# F9P ACM → F9P USB@230400 → LC29H EA USB → DA USB → DA UART, persist the hit, and re-apply.
# Returns 0 when a rover char device is available, 1 when nothing usable was found.
companion_gps_boot_resolve_available() {
    local script_dir sniff_py python token_line token port_extra profile
    local usb_port uart_port

    companion_gps_apply_module
    if [[ -n "${COMPANION_ROVER_PORT:-}" && -c "${COMPANION_ROVER_PORT}" ]]; then
        echo "companion_gps: using saved $(companion_gps_module_label) @ ${COMPANION_ROVER_PORT}" >&2
        return 0
    fi

    echo "companion_gps: saved rover ${COMPANION_ROVER_PORT:-<unset>} missing — sniffing available GNSS…" >&2
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    sniff_py="${script_dir}/sniff_companion_gps_profile.py"
    if [[ ! -f "${sniff_py}" ]]; then
        sniff_py="${script_dir}/sniff_lc29h_profile.py"
    fi
    if [[ ! -f "${sniff_py}" ]]; then
        echo "companion_gps: ERROR: sniff helper missing under ${script_dir}" >&2
        return 1
    fi

    python="${COMPANION_PYTHON:-${PYTHON:-/home/pi/miniconda/envs/RL/bin/python}}"
    if [[ ! -x "${python}" ]]; then
        python="$(command -v python3 || true)"
    fi
    if [[ -z "${python}" ]]; then
        echo "companion_gps: ERROR: no python for GNSS sniff" >&2
        return 1
    fi

    usb_port="${ROVER_PORT:-/dev/ttyUSB0}"
    # If saved F9P pointed at ACM, still sniff LC29H on the default USB path.
    if [[ "${usb_port}" == /dev/ttyACM* ]]; then
        usb_port="/dev/ttyUSB0"
    fi
    uart_port="${ROVER_PORT_UART:-/dev/ttyAMA4}"

    set +e
    token_line="$("${python}" "${sniff_py}" --usb-port "${usb_port}" --uart-port "${uart_port}")"
    set -e
    token_line="$(printf '%s' "${token_line}" | tr -d '\r' | tail -n1)"
    token="${token_line%%|*}"
    port_extra=""
    if [[ "${token_line}" == *"|"* ]]; then
        port_extra="${token_line#*|}"
    fi

    profile=""
    case "${token}" in
        usb_f9p) profile=f9p ;;
        usb_ea) profile=ea ;;
        usb_da) profile=da-usb ;;
        uart_da) profile=da-uart ;;
        *)
            # Last-ditch: any ACM → f9p, else USB0 → ea (no VERNO), else fail.
            local acm
            acm="$(ls -1 /dev/ttyACM* 2>/dev/null | head -n1 || true)"
            if [[ -n "${acm}" && -c "${acm}" ]]; then
                profile=f9p
                port_extra="${acm}"
                echo "companion_gps: sniff token empty — using present ${acm} as F9P" >&2
            elif [[ -c /dev/ttyUSB0 ]]; then
                profile=ea
                echo "companion_gps: sniff token empty — using /dev/ttyUSB0 as EA defaults" >&2
            else
                echo "companion_gps: no available GNSS rover found (token=${token:-empty})" >&2
                return 1
            fi
            ;;
    esac

    if [[ "${profile}" == "f9p" ]]; then
        companion_gps_write_fleet_profile "${profile}" "${port_extra:-/dev/ttyACM0}"
    else
        companion_gps_write_fleet_profile "${profile}"
    fi
    companion_gps_apply_module
    if [[ -n "${COMPANION_ROVER_PORT:-}" && -c "${COMPANION_ROVER_PORT}" ]]; then
        echo "companion_gps: boot selected $(companion_gps_module_label) @ ${COMPANION_ROVER_PORT} (persisted)" >&2
        companion_gps_show_current_choice "boot-sniff"
        return 0
    fi
    echo "companion_gps: sniff profile=${profile} but rover ${COMPANION_ROVER_PORT:-?} still missing" >&2
    return 1
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

    # startRtkCommPI.sh reads ROVER_WIRE from the environment.
    export ROVER_WIRE="${ROVER_WIRE:-nmea}"

    args=(
        --rtk-zmq-url="${rtk_zmq_url}"
        --rover-type="${ROVER_TYPE:-lc29h}"
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

# RTCM → GPS_RTCM_DATA (mavlink) path: no rover_zmq / emulate_gps_to_px4.
companion_rtcm_mavlink_start_in_tmux() {
    local session="$1"
    local python="${2:-python3}"
    local root="${3:-}"
    local launcher="${root}/GPS_RTK/startRtcmToMavlinkPI.sh"
    local window="${COMPANION_RTCM_MAV_WINDOW:-rtcm_mav}"

    if [[ -z "${root}" ]]; then
        echo "companion_rtcm_mavlink_start_in_tmux: CATSWARM_ROOT required" >&2
        return 1
    fi
    if [[ ! -x "${launcher}" && ! -f "${launcher}" ]]; then
        echo "companion_rtcm_mavlink_start_in_tmux: missing ${launcher}" >&2
        return 1
    fi

    : "${COMPANION_BASE_HOST:=192.168.0.43}"
    : "${COMPANION_BASE_PORT_NUM:=5560}"
    : "${COMPANION_RTK_ZMQ_BIND:=tcp://127.0.0.1:5562}"
    : "${COMPANION_MAVLINK_RTCM:=udpout:127.0.0.1:14580}"

    tmux kill-window -t "${session}:${window}" 2>/dev/null || true
    echo "Starting rtcm_to_mavlink in ${session}:${window} → ${COMPANION_MAVLINK_RTCM}…" >&2
    tmux new-window -t "${session}" -n "${window}" \
        "export COMPANION_BASE_HOST='${COMPANION_BASE_HOST}' COMPANION_BASE_PORT_NUM='${COMPANION_BASE_PORT_NUM}' COMPANION_RTK_ZMQ_BIND='${COMPANION_RTK_ZMQ_BIND}' COMPANION_MAVLINK_RTCM='${COMPANION_MAVLINK_RTCM}' PYTHON='${python}'; exec '${launcher}'"
}

# Write a full fleet profile into ~/.config/companion-gps (flush, not merge).
# Profiles: ea | da-usb | da-uart | f9p
# Optional 2nd arg: rover device path (used by f9p sniff for detected ttyACM*/ttyUSB*).
companion_gps_write_fleet_profile() {
    local profile
    local port_override="${2:-}"
    profile="$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')"
    ROVER_PORT_UART="${ROVER_PORT_UART:-/dev/ttyAMA4}"
    ROVER_BAUD_UART="${ROVER_BAUD_UART:-115200}"
    COMPANION_PX4_GPS_PORT="${COMPANION_PX4_GPS_PORT:-/dev/ttyAMA0}"
    case "${profile}" in
        ea|ea-usb)
            COMPANION_GPS_MODULE=ea
            ROVER_TYPE=lc29h
            # Always USB0 for EA — do not keep a prior F9P ACM path.
            ROVER_PORT="${port_override:-/dev/ttyUSB0}"
            ROVER_BAUD=460800
            ROVER_WIRE=nmea
            DA_RATE_MS=100
            ;;
        da-usb|da_usb|usb-da)
            COMPANION_GPS_MODULE=ea
            ROVER_TYPE=lc29h
            ROVER_PORT="${port_override:-/dev/ttyUSB0}"
            ROVER_BAUD=115200
            ROVER_WIRE=nmea
            DA_RATE_MS=1000
            ;;
        da|da-uart|da_uart|uart-da)
            COMPANION_GPS_MODULE=da
            ROVER_TYPE=lc29h
            ROVER_BAUD="${ROVER_BAUD:-460800}"
            ROVER_BAUD_UART=115200
            ROVER_WIRE=nmea
            DA_RATE_MS=1000
            ;;
        f9p|zed-f9p|ublox)
            COMPANION_GPS_MODULE=f9p
            ROVER_TYPE=f9p
            ROVER_PORT="${port_override:-/dev/ttyACM0}"
            case "${ROVER_PORT}" in
                /dev/ttyUSB*) ROVER_BAUD=230400 ;;
                *) ROVER_BAUD=115200 ;;
            esac
            ROVER_WIRE=ubx
            # 1 Hz — lower rate for flight; emulate_gps still heartbeats GGA so PX4 stays alive.
            DA_RATE_MS=1000
            ;;
        *)
            echo "companion_gps_write_fleet_profile: unknown profile '${1:-}' (use ea|da-usb|da-uart|f9p)" >&2
            return 1
            ;;
    esac
    companion_gps_save_module
    companion_gps_apply_module
    echo "companion_gps: flushed fleet profile=${profile} → $(companion_gps_state_file)" >&2
    companion_gps_show_current_choice "fleet-flush"
}
