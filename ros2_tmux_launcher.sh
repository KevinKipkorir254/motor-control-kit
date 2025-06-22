#!/bin/bash

# ROS2 Tmux Multi-Pane Launcher
# This script creates a tmux session with multiple panes for different ROS2 nodes

# Session name
SESSION_NAME="ros2_nodes"

# Check if tmux is installed
check_tmux() {
    if ! command -v tmux &> /dev/null; then
        echo "tmux not found. Installing..."
        sudo apt update && sudo apt install -y tmux
    fi
}

# Function to setup ROS2 environment in each pane
setup_ros_env() {
    echo "source /opt/ros/humble/setup.bash"
    # Add your workspace setup if needed
    echo "source ~/robotics_inc/motor-kit-ws/src/motor-control-kit/install/setup.bash"
}

# Check tmux installation
check_tmux

echo "Creating tmux session: $SESSION_NAME"

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new session with first window
tmux new-session -d -s $SESSION_NAME -n "ROS2_NODES"

# Split the window into 4 panes
tmux split-window -h -t $SESSION_NAME:0     # Split horizontally
tmux split-window -v -t $SESSION_NAME:0.0   # Split left pane vertically  
tmux split-window -v -t $SESSION_NAME:0.1   # Split right pane vertically

# Setup ROS environment and run commands in each pane

# Pane 0 (top-left): Hardware Interface
tmux send-keys -t $SESSION_NAME:0.0 "$(setup_ros_env)" Enter
tmux send-keys -t $SESSION_NAME:0.0 "echo 'Starting Hardware Interface...'" Enter
tmux send-keys -t $SESSION_NAME:0.0 "ros2 launch encoded_dc_motor_kit_description motor_kit_server.launch.py" Enter

# Pane 1 (bottom-left): Controller
tmux send-keys -t $SESSION_NAME:0.1 "$(setup_ros_env)" Enter
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Starting Controller...'" Enter
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run encoded_dc_motor_kit_PID p_controller" Enter

# Pane 2 (top-right): Sensor Node
tmux send-keys -t $SESSION_NAME:0.2 "$(setup_ros_env)" Enter
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Starting Sensor Node...'" Enter
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run encoded_dc_motor_kit_control_and_feedback velocity_and_frequency_monitor" Enter

# Pane 3 (bottom-right): Navigation
tmux send-keys -t $SESSION_NAME:0.3 "$(setup_ros_env)" Enter
tmux send-keys -t $SESSION_NAME:0.3 "echo 'Starting Navigation...'" Enter
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run encoded_dc_motor_kit_control_and_feedback velocity_commands_publisher" Enter

# Optional: Create additional window for monitoring
tmux new-window -t $SESSION_NAME -n "Monitor"
tmux send-keys -t $SESSION_NAME:Monitor "$(setup_ros_env)" Enter
tmux send-keys -t $SESSION_NAME:Monitor "echo 'ROS2 System Monitor'" Enter
tmux send-keys -t $SESSION_NAME:Monitor "ros2 topic list" Enter

# Attach to the session
echo "Attaching to tmux session..."
echo "Use 'Ctrl+b d' to detach, 'tmux attach -t $SESSION_NAME' to reattach"
sleep 1
tmux attach-session -t $SESSION_NAME