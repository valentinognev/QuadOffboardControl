#!/usr/bin/env bash
# Unified GNSS serial parameters for startup_scripts and GPS_RTK launchers.
#
# | Role         | Environment variable | CLI flag      |
# |--------------|----------------------|---------------|
# | Base serial  | BASE_PORT            | --base-port   |
# | Base baud    | BASE_BAUD            | --base-baud   |
# | Rover serial | ROVER_PORT           | --rover-port  |
# | Rover baud   | ROVER_BAUD           | --rover-baud  |
#
# Source this file, call gnss_serial_migrate_legacy_env, then use
# gnss_serial_parse_arg in your option loop (or gnss_serial_parse_argv).

: "${BASE_PORT:=}"
: "${BASE_BAUD:=}"
: "${ROVER_PORT:=}"
: "${ROVER_BAUD:=}"

gnss_serial_migrate_legacy_env() {
    local _legacy=0

    if [[ -n "${BS_PORT:-}" ]]; then
        if [[ -z "${BASE_PORT}" ]]; then
            BASE_PORT="${BS_PORT}"
        fi
        _legacy=1
    fi
    if [[ -n "${BS_BAUD:-}" ]]; then
        if [[ -z "${BASE_BAUD}" ]]; then
            BASE_BAUD="${BS_BAUD}"
        fi
        _legacy=1
    fi
    if [[ -n "${DA_PORT:-}${EA_PORT:-}" ]]; then
        if [[ -z "${ROVER_PORT}" ]]; then
            ROVER_PORT="${EA_PORT:-${DA_PORT}}"
        fi
        _legacy=1
    fi
    if [[ -n "${DA_BAUD:-}${EA_BAUD:-}" ]]; then
        if [[ -z "${ROVER_BAUD}" ]]; then
            ROVER_BAUD="${EA_BAUD:-${DA_BAUD}}"
        fi
        _legacy=1
    fi
    if [[ -n "${COMPANION_EA_PORT:-}" && -z "${ROVER_PORT}" ]]; then
        ROVER_PORT="${COMPANION_EA_PORT}"
        _legacy=1
    fi
    if [[ -n "${COMPANION_EA_BAUD:-}" && -z "${ROVER_BAUD}" ]]; then
        ROVER_BAUD="${COMPANION_EA_BAUD}"
        _legacy=1
    fi
    if [[ -n "${COMPANION_DA_PORT:-}" && -z "${ROVER_PORT}" ]]; then
        ROVER_PORT="${COMPANION_DA_PORT}"
        _legacy=1
    fi
    if [[ -n "${COMPANION_DA_BAUD:-}" && -z "${ROVER_BAUD}" ]]; then
        ROVER_BAUD="${COMPANION_DA_BAUD}"
        _legacy=1
    fi

    if [[ "${_legacy}" -eq 1 ]]; then
        echo "gnss_serial_args: deprecated env name (BS_*/DA_*/EA_*/COMPANION_*_PORT); use BASE_PORT, BASE_BAUD, ROVER_PORT, ROVER_BAUD" >&2
    fi
    export BASE_PORT BASE_BAUD ROVER_PORT ROVER_BAUD
}

# Parse one argv token (and optional value). Returns 0 if consumed.
# Sets GNSS_SERIAL_SHIFT to 1 or 2 for the caller's shift count.
gnss_serial_parse_arg() {
    local arg="${1:-}"
    local next="${2:-}"
    GNSS_SERIAL_SHIFT=1

    case "${arg}" in
        --base-port=*|--bs-port=*)
            BASE_PORT="${arg#*=}"
            return 0
            ;;
        --base-baud=*|--bs-baud=*)
            BASE_BAUD="${arg#*=}"
            return 0
            ;;
        --rover-port=*|--da-port=*|--ea-port=*)
            ROVER_PORT="${arg#*=}"
            return 0
            ;;
        --rover-baud=*|--da-baud=*|--ea-baud=*)
            ROVER_BAUD="${arg#*=}"
            return 0
            ;;
        --base-port|--bs-port)
            [[ -n "${next}" ]] || return 2
            BASE_PORT="${next}"
            GNSS_SERIAL_SHIFT=2
            return 0
            ;;
        --base-baud|--bs-baud)
            [[ -n "${next}" ]] || return 2
            BASE_BAUD="${next}"
            GNSS_SERIAL_SHIFT=2
            return 0
            ;;
        --rover-port|--da-port|--ea-port|--port)
            [[ -n "${next}" ]] || return 2
            ROVER_PORT="${next}"
            GNSS_SERIAL_SHIFT=2
            return 0
            ;;
        --rover-baud|--da-baud|--ea-baud|--baud)
            [[ -n "${next}" ]] || return 2
            ROVER_BAUD="${next}"
            GNSS_SERIAL_SHIFT=2
            return 0
            ;;
    esac
    return 1
}

gnss_serial_help_options() {
    cat <<'EOF'
  --base-port=DEVICE      Base (BS) serial device (env: BASE_PORT)
  --base-baud=RATE        Base UART baud (env: BASE_BAUD)
  --rover-port=DEVICE     Rover (EA/DA) serial device (env: ROVER_PORT)
  --rover-baud=RATE       Rover UART baud (env: ROVER_BAUD)
EOF
}

gnss_serial_help_env() {
    printf '%s\n' "Environment: BASE_PORT, BASE_BAUD, ROVER_PORT, ROVER_BAUD"
}
