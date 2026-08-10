#!/bin/bash
# Script to run mavlink_to_ZMQ and zmq_commands_mavlink pairs for each drone in tmux
# Each drone gets its own window with vertical split

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default values
POSITIONS_FILE="${SCRIPT_DIR}/positions.txt"
TMUX_SESSION="multidrone_bridges"
HARDWARE_ADAPTER_DIR="${SCRIPT_DIR}/../hardware_adapter"
MAVLINK_TO_ZMQ_BIN="${HARDWARE_ADAPTER_DIR}/bin/mavlink_to_ZMQ"
ZMQ_COMMANDS_MAVLINK_BIN="${HARDWARE_ADAPTER_DIR}/bin/zmq_commands_mavlink"
LOG_BASE_DIR="${SCRIPT_DIR}/../logs"

# Default ports
MAVLINK_TO_ZMQ_UDP_BASE=14540
ZMQ_COMMANDS_MAVLINK_UDP_BASE=14030
MAVLINK_TO_ZMQ_ZMQ_BASE=9900
ZMQ_COMMANDS_MAVLINK_ZMQ_BASE=7700

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--positions)
            POSITIONS_FILE="$2"
            shift 2
            ;;
        -s|--session)
            TMUX_SESSION="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -p, --positions FILE    Positions file (default: positions.txt)"
            echo "  -s, --session NAME       Tmux session name (default: multidrone_bridges)"
            echo "  -h, --help              Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Check if positions file exists
if [ ! -f "$POSITIONS_FILE" ]; then
    echo "Error: Positions file '$POSITIONS_FILE' not found!"
    exit 1
fi

# Check if binaries exist
if [ ! -f "$MAVLINK_TO_ZMQ_BIN" ]; then
    echo "Error: mavlink_to_ZMQ binary not found at '$MAVLINK_TO_ZMQ_BIN'"
    exit 1
fi

if [ ! -f "$ZMQ_COMMANDS_MAVLINK_BIN" ]; then
    echo "Error: zmq_commands_mavlink binary not found at '$ZMQ_COMMANDS_MAVLINK_BIN'"
    exit 1
fi

# Count number of drones from positions file
NUM_DRONES=0
while IFS=' ' read -r x y z || [ -n "$x" ]; do
    # Skip empty lines and comments
    [[ -z "$x" || "$x" =~ ^# ]] && continue
    NUM_DRONES=$((NUM_DRONES + 1))
done < "$POSITIONS_FILE"

if [ $NUM_DRONES -eq 0 ]; then
    echo "Error: No valid positions found in '$POSITIONS_FILE'"
    exit 1
fi

echo "Found $NUM_DRONES drones in $POSITIONS_FILE"
echo "Creating tmux session '$TMUX_SESSION' with $NUM_DRONES windows..."

# Kill existing session if it exists
tmux has-session -t "$TMUX_SESSION" 2>/dev/null && tmux kill-session -t "$TMUX_SESSION"

# Create new tmux session (detached)
tmux new-session -d -s "$TMUX_SESSION" -x 200 -y 50

# Function to create a window for a drone pair
create_drone_window() {
    local drone_id=$1
    local mavlink_udp_port=$((MAVLINK_TO_ZMQ_UDP_BASE + drone_id))
    local zmq_commands_udp_port=$((ZMQ_COMMANDS_MAVLINK_UDP_BASE + drone_id))
    local mavlink_zmq_port=$((MAVLINK_TO_ZMQ_ZMQ_BASE + drone_id))
    local zmq_commands_zmq_port=$((ZMQ_COMMANDS_MAVLINK_ZMQ_BASE + drone_id))
    
    local window_name="drone${drone_id}"
    
    # Create new window
    if [ $drone_id -eq 1 ]; then
        # First window - rename the default window
        tmux rename-window -t "$TMUX_SESSION:0" "$window_name"
    else
        # Create new window
        tmux new-window -t "$TMUX_SESSION" -n "$window_name"
    fi
    
    # Split window vertically
    tmux split-window -h -t "$TMUX_SESSION:$window_name"
    
    # Create log directories
    mkdir -p "${LOG_BASE_DIR}/drone${drone_id}/mavlink_to_ZMQ"
    mkdir -p "${LOG_BASE_DIR}/drone${drone_id}/zmq_commands"
    
    # Left pane: mavlink_to_ZMQ
    tmux send-keys -t "$TMUX_SESSION:$window_name.0" \
        "$MAVLINK_TO_ZMQ_BIN --udp:${mavlink_udp_port} --zmq:${mavlink_zmq_port} --log:${LOG_BASE_DIR}/drone${drone_id}/mavlink_to_ZMQ" C-m
    
    # Right pane: zmq_commands_mavlink
    # NOTE: this launcher runs the *C* bridges, whose arg parser rejects any unknown
    # `-`-prefixed option (main_zmq_commands_mavlink.c). Do NOT add --zmqFlightData
    # here: the C bridge has no GPS-quality override PUSH, so the flag only makes the
    # process exit rc=1. GPS-quality control is Python-bridge only — see
    # hardware_adapter/hardware_adapter_multi.sh (--version=PY).
    tmux send-keys -t "$TMUX_SESSION:$window_name.1" \
        "$ZMQ_COMMANDS_MAVLINK_BIN --udp:${zmq_commands_udp_port} --zmq:${zmq_commands_zmq_port} --log:${LOG_BASE_DIR}/drone${drone_id}/zmq_commands" C-m
    
    # Set pane titles (optional, for better visibility - requires tmux 3.0+)
    # Note: -T option for pane titles may not be available in older tmux versions
    # If it fails, it's safe to ignore
    tmux select-pane -t "$TMUX_SESSION:$window_name.0" -T "mavlink_to_ZMQ (UDP:${mavlink_udp_port}, ZMQ:${mavlink_zmq_port})" 2>/dev/null || true
    tmux select-pane -t "$TMUX_SESSION:$window_name.1" -T "zmq_commands_mavlink (UDP:${zmq_commands_udp_port}, ZMQ:${zmq_commands_zmq_port})" 2>/dev/null || true
}

# Create windows for each drone
for i in $(seq 1 $NUM_DRONES); do
    echo "Creating window for drone $i..."
    create_drone_window $i
done

# Select first window
tmux select-window -t "$TMUX_SESSION:drone1"

# Set some tmux options for better experience
tmux set-option -t "$TMUX_SESSION" -g mouse on
tmux set-option -t "$TMUX_SESSION" -g status-interval 1

echo ""
echo "Tmux session '$TMUX_SESSION' created successfully!"
echo ""
echo "Port assignments:"
for i in $(seq 1 $NUM_DRONES); do
    mavlink_udp=$((MAVLINK_TO_ZMQ_UDP_BASE + i))
    zmq_commands_udp=$((ZMQ_COMMANDS_MAVLINK_UDP_BASE + i))
    mavlink_zmq=$((MAVLINK_TO_ZMQ_ZMQ_BASE + i))
    zmq_commands_zmq=$((ZMQ_COMMANDS_MAVLINK_ZMQ_BASE + i))
    echo "  Drone $i:"
    echo "    mavlink_to_ZMQ:      UDP ${mavlink_udp}, ZMQ ${mavlink_zmq}"
    echo "    zmq_commands_mavlink: UDP ${zmq_commands_udp}, ZMQ ${zmq_commands_zmq}"
done
echo ""
echo "To attach to the session, run:"
echo "  tmux attach -t $TMUX_SESSION"
echo ""
echo "To kill the session, run:"
echo "  tmux kill-session -t $TMUX_SESSION"
echo ""

# Optionally attach to the session
read -p "Attach to tmux session now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    tmux attach -t "$TMUX_SESSION"
fi
