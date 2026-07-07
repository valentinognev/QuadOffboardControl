#!/bin/bash
# Wrapper script to run mavlink-server with configuration file
# This script reads the config file and converts it to command-line arguments

set -e

usage() {
    cat <<EOF
Usage: $0 [config-file]
       $0 -h|--help

Convert mavlink-server TOML config to CLI args and exec mavlink-server.

Arguments:
  config-file   Path to config (default: /etc/mavlink-server/mavlink-server.conf)

Binary: /usr/bin/mavlink-server

Configure on Pi:
  sudo ~/deploy_pi5/mavlink-server-configuration.sh --fleet-preset
EOF
}

case "${1:-}" in
    -h|--help|help) usage; exit 0 ;;
esac

CONFIG_FILE="${1:-/etc/mavlink-server/mavlink-server.conf}"
BINARY_PATH="/usr/bin/mavlink-server"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file '$CONFIG_FILE' not found!" >&2
    exit 1
fi

# Function to trim whitespace
trim() {
    echo "$1" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//'
}

# Function to parse TOML value
parse_toml_value() {
    local line="$1"
    line="${line%%#*}"
    line=$(trim "$line")
    
    if [ -z "$line" ]; then
        return
    fi
    
    if [[ "$line" =~ ^([^=]+)=(.*)$ ]]; then
        local key="${BASH_REMATCH[1]}"
        local value="${BASH_REMATCH[2]}"
        key=$(trim "$key")
        value=$(trim "$value")
        value="${value#\"}"
        value="${value%\"}"
        echo "$key|$value"
    fi
}

# Function to build endpoint string from table data
build_endpoint_from_table() {
    case "$current_table_type" in
        serial)
            if [ -n "${current_table[device]:-}" ] && [ -n "${current_table[baudrate]:-}" ]; then
                ENDPOINTS+=("serial://${current_table[device]}?baudrate=${current_table[baudrate]}")
            fi
            ;;
        udp_server)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                ENDPOINTS+=("udpserver://${current_table[address]}:${current_table[port]}")
            fi
            ;;
        udp_client)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                local direction_param=""
                if [ -n "${current_table[direction]:-}" ]; then
                    direction_param="?direction=${current_table[direction]}"
                fi
                ENDPOINTS+=("udpclient://${current_table[address]}:${current_table[port]}${direction_param}")
            fi
            ;;
        tcp_server)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                ENDPOINTS+=("tcpserver://${current_table[address]}:${current_table[port]}")
            fi
            ;;
        tcp_client)
            if [ -n "${current_table[address]:-}" ] && [ -n "${current_table[port]:-}" ]; then
                ENDPOINTS+=("tcpclient://${current_table[address]}:${current_table[port]}")
            fi
            ;;
    esac
}

# Initialize arrays
ENDPOINTS=()
ARGS=()
in_table_array=false
current_table_type=""
declare -A current_table

# Parse configuration file
while IFS= read -r line || [ -n "$line" ]; do
    clean_line="${line%%#*}"
    clean_line=$(trim "$clean_line")
    
    # Check for TOML array-of-tables: [[section]]
    if [[ "$clean_line" =~ ^\[\[(.+)\]\] ]]; then
        # Save previous table if exists
        if [ "$in_table_array" = true ] && [ ${#current_table[@]} -gt 0 ]; then
            build_endpoint_from_table
        fi
        
        # Start new table array
        current_table_type="${BASH_REMATCH[1]}"
        in_table_array=true
        unset current_table
        declare -A current_table
        continue
    fi
    
    # Check if we're in a table array, parse key=value pairs
    if [ "$in_table_array" = true ]; then
        # Check if we hit another table or section (end of current table)
        if [[ "$clean_line" =~ ^\[ ]] || [ -z "$clean_line" ]; then
            # End of current table, build endpoint
            if [ ${#current_table[@]} -gt 0 ]; then
                build_endpoint_from_table
            fi
            unset current_table
            declare -A current_table
            
            # If it's a new table array, continue processing
            if [[ "$clean_line" =~ ^\[\[ ]]; then
                if [[ "$clean_line" =~ ^\[\[(.+)\]\] ]]; then
                    current_table_type="${BASH_REMATCH[1]}"
                fi
                continue
            fi
            in_table_array=false
        else
            # Parse key=value within table
            result=$(parse_toml_value "$line")
            if [ -n "$result" ]; then
                IFS='|' read -r key value <<< "$result"
                current_table["$key"]="$value"
            fi
            continue
        fi
    fi
    
    # Parse regular key=value pairs
    result=$(parse_toml_value "$line")
    if [ -n "$result" ]; then
        IFS='|' read -r key value <<< "$result"
        
        # Store config value
        case "$key" in
            web_server)
                ARGS+=("--web-server" "$value")
                ;;
            default_api_version)
                ARGS+=("--default-api-version" "$value")
                ;;
            verbose)
                if [ "$value" = "true" ]; then
                    ARGS+=("-v")
                fi
                ;;
            log_path)
                ARGS+=("--log-path" "$value")
                ;;
            enable_tracing_level_log_file)
                if [ "$value" = "true" ]; then
                    ARGS+=("--enable-tracing-level-log-file")
                fi
                ;;
            streamreq_disable)
                if [ "$value" = "true" ]; then
                    ARGS+=("--streamreq-disable")
                fi
                ;;
            udp_server_timeout)
                ARGS+=("--udp-server-timeout" "$value")
                ;;
            mavlink_system_id)
                ARGS+=("--mavlink-system-id" "$value")
                ;;
            mavlink_component_id)
                ARGS+=("--mavlink-component-id" "$value")
                ;;
            mavlink_heartbeat_frequency)
                ARGS+=("--mavlink-heartbeat-frequency" "$value")
                ;;
            send_initial_heartbeats)
                if [ "$value" = "true" ]; then
                    ARGS+=("--send-initial-heartbeats")
                fi
                ;;
            zenoh_config_file)
                ARGS+=("--zenoh-config-file" "$value")
                ;;
        esac
    fi
done < "$CONFIG_FILE"

# Handle last table if file ends while in table array
if [ "$in_table_array" = true ] && [ ${#current_table[@]} -gt 0 ]; then
    build_endpoint_from_table
fi

# Add endpoints to arguments
ARGS+=("${ENDPOINTS[@]}")

# Execute mavlink-server
exec "$BINARY_PATH" "${ARGS[@]}"
