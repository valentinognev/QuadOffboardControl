#!/bin/bash

# Script to start mavlink_to_ZMQ and zmq_commands_mavlink in parallel tmux panes

PROJECT_PATH="/home/lahav/rl_dockerfile/QuadOffboardControl"
SESSION_NAME="hardware_adapter"
SERIAL_PORT="/dev/ttyUSB0"
DRONE_ID=1

# Check if session already exists, kill it if it does
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Session $SESSION_NAME already exists. Killing it..."
    tmux kill-session -t "$SESSION_NAME"
fi

# Create new tmux session in detached mode
tmux new-session -d -s "$SESSION_NAME"
sleep 1

# Split window horizontally (creates left/right panes)
tmux split-window -h
sleep 1
# Split right pane vertically (creates top-right/bottom-right)
tmux select-pane -t 1
tmux split-window -v
sleep 1

# Run first program in left pane (pane 0)
tmux select-pane -t 0
tmux send-keys "cd $PROJECT_PATH" ENTER
tmux send-keys "$PROJECT_PATH/hardware_adapter/bin/mavlink_to_ZMQ" ENTER

# Run second program in top-right pane (pane 1)
tmux select-pane -t 1
tmux send-keys "cd $PROJECT_PATH" ENTER
tmux send-keys "$PROJECT_PATH/hardware_adapter/bin/zmq_commands_mavlink" ENTER

# Run bridge in bottom-right pane (pane 2)
tmux select-pane -t 2
tmux send-keys "python3 $PROJECT_PATH/hardware_adapter/zmq_to_serial_bridge.py --serial-port $SERIAL_PORT --drone-id $DRONE_ID --target-hz 50" ENTER

# Select the first pane and attach to session
tmux select-pane -t 0
tmux attach-session -t "$SESSION_NAME"
