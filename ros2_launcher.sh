#!/bin/bash

# Multi-Terminal ROS2 Launcher Script
# This script works both on desktop (GUI terminals) and headless servers (tmux)

# Session name for tmux
SESSION_NAME="ros2_nodes"

# Function to check if we're in a GUI environment
check_gui_environment() {
    if [ -n "$DISPLAY" ] || [ -n "$WAYLAND_DISPLAY" ]; then
        return 0  # GUI available
    else
        return 1  # No GUI (headless)
    fi
}

# Function to check available GUI terminals
check_terminal() {
    if command -v gnome-terminal &> /dev/null && pgrep -f gnome-session &> /dev/null; then
        TERMINAL_CMD="gnome-terminal"
        echo "Using gnome-terminal"
    elif command -v xfce4-terminal &> /dev/null; then
        TERMINAL_CMD="xfce4-terminal"
        echo "Using xfce4-terminal"
    elif command -v xterm &> /dev/null; then
        TERMINAL_CMD="xterm"
        echo "Using xterm"
    elif command -v konsole &> /dev/null; then
        TERMINAL_CMD="konsole"
        echo "Using konsole"
    else
        echo "No suitable GUI terminal found. Installing xterm..."
        sudo apt update && sudo apt install -y xterm
        TERMINAL_CMD="xterm"
    fi
}

# Function to check if tmux is available
check_tmux() {
    if ! command -v tmux &> /dev/null; then
        echo "tmux not found. Installing..."
        sudo apt update && sudo apt install -y tmux
    fi
}

# Function to setup ROS2 environment
setup_ros_env() {
    echo "source /opt/ros/humble/setup.bash && source ~/robotics_inc/motor-kit-ws/install/setup.bash"
}

# Function to launch a command in a new GUI terminal
launch_terminal() {
    local title="$1"
    local command="$2"
    local hold_option="$3"
    
    case $TERMINAL_CMD in
        "gnome-terminal")
            if [ "$hold_option" = "hold" ]; then
                gnome-terminal --title="$title" --window -- bash -c "$command; echo 'Press Enter to close...'; read" &
            else
                gnome-terminal --title="$title" --window -- bash -c "$command" &
            fi
            ;;
        "xfce4-terminal")
            xfce4-terminal --title="$title" --hold --command="bash -c '$command'" &
            ;;
        "xterm")
            if [ "$hold_option" = "hold" ]; then
                xterm -title "$title" -hold -e bash -c "$command" &
            else
                xterm -title "$title" -e bash -c "$command" &
            fi
            ;;
        "konsole")
            konsole --title "$title" --hold -e bash -c "$command" &
            ;;
    esac
    
    # Small delay to prevent terminals from opening too quickly
    sleep 0.8
}

# Function to launch using tmux (for headless)
launch_tmux_session() {
    echo "Creating tmux session for headless operation..."
    
    # Kill existing session if it exists
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    
    # Create new session with first window
    tmux new-session -d -s $SESSION_NAME -n "ROS2_Nodes"
    
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
    tmux send-keys -t $SESSION_NAME:0.1 "echo 'Starting P Controller...'" Enter
    tmux send-keys -t $SESSION_NAME:0.1 "ros2 run encoded_dc_motor_kit_PID p_controller" Enter
    
    # Pane 2 (top-right): Sensor Node
    tmux send-keys -t $SESSION_NAME:0.2 "$(setup_ros_env)" Enter
    tmux send-keys -t $SESSION_NAME:0.2 "echo 'Starting Velocity Monitor...'" Enter
    tmux send-keys -t $SESSION_NAME:0.2 "ros2 run encoded_dc_motor_kit_control_and_feedback velocity_and_frequency_monitor" Enter
    
    # Pane 3 (bottom-right): Command Publisher
    tmux send-keys -t $SESSION_NAME:0.3 "$(setup_ros_env)" Enter
    tmux send-keys -t $SESSION_NAME:0.3 "echo 'Starting Velocity Commands Publisher...'" Enter
    tmux send-keys -t $SESSION_NAME:0.3 "ros2 run encoded_dc_motor_kit_control_and_feedback velocity_commands_publisher" Enter
    
    # Optional: Create additional window for monitoring
    tmux new-window -t $SESSION_NAME -n "Monitor"
    tmux send-keys -t $SESSION_NAME:Monitor "$(setup_ros_env)" Enter
    tmux send-keys -t $SESSION_NAME:Monitor "echo 'ROS2 System Monitor - Use this pane for debugging'" Enter
    tmux send-keys -t $SESSION_NAME:Monitor "echo 'Try: ros2 topic list, ros2 node list, ros2 topic echo /topic_name'" Enter
    
    # Show session info
    echo "═══════════════════════════════════════════════════════════════"
    echo "✅ ROS2 nodes launched in tmux session: $SESSION_NAME"
    echo "═══════════════════════════════════════════════════════════════"
    echo "📋 TMUX COMMANDS:"
    echo "   • Attach to session:  tmux attach -t $SESSION_NAME"
    echo "   • Detach from session: Ctrl+b then d"
    echo "   • Switch panes: Ctrl+b then arrow keys"
    echo "   • Switch windows: Ctrl+b then 0-9"
    echo "   • Kill session: tmux kill-session -t $SESSION_NAME"
    echo "═══════════════════════════════════════════════════════════════"
    echo ""
    
    # Ask if user wants to attach immediately
    read -p "Attach to tmux session now? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Attaching to session... (Use Ctrl+b d to detach later)"
        sleep 1
        tmux attach-session -t $SESSION_NAME
    else
        echo "Session created! Use 'tmux attach -t $SESSION_NAME' to attach later."
    fi
}

# Main execution logic
echo "ROS2 Multi-Node Launcher"
echo "========================"

# Check if GUI is available
if check_gui_environment; then
    echo "GUI environment detected - using separate terminal windows"
    
    # Check available terminal
    check_terminal
    
    echo "Launching ROS2 nodes in separate terminals..."
    
    # Terminal 1: Hardware Interface
    launch_terminal "Hardware Interface" "source /opt/ros/humble/setup.bash && source ~/robotics_inc/motor-kit-ws/install/setup.bash && ros2 launch encoded_dc_motor_kit_description motor_kit_server.launch.py" "hold"
    
    # Terminal 2: Controller Node
    launch_terminal "Controller" "source /opt/ros/humble/setup.bash && source ~/robotics_inc/motor-kit-ws/install/setup.bash && ros2 run encoded_dc_motor_kit_PID p_controller" "hold"
    
    # Terminal 3: Sensor Node
    launch_terminal "Sensor Node" "source /opt/ros/humble/setup.bash && source ~/robotics_inc/motor-kit-ws/install/setup.bash && ros2 run encoded_dc_motor_kit_control_and_feedback velocity_and_frequency_monitor" "hold"
    
    # Terminal 4: Navigation Node
    launch_terminal "Navigation" "source /opt/ros/humble/setup.bash && source ~/robotics_inc/motor-kit-ws/install/setup.bash && ros2 run encoded_dc_motor_kit_control_and_feedback velocity_commands_publisher" "hold"
    
    # Optional: Terminal 5 for monitoring (uncomment if needed)
    # launch_terminal "ROS2 Monitor" "source /opt/ros/humble/setup.bash && source ~/robotics_inc/motor-kit-ws/install/setup.bash && ros2 topic list && ros2 node list" "hold"
    
    echo "All terminals launched successfully!"
    echo "Note: Each terminal will stay open after the command finishes."
    
else
    echo "Headless environment detected - using tmux session"
    
    # Check tmux availability
    check_tmux
    
    # Launch tmux session
    launch_tmux_session
fi

# Keep the script running briefly to show the message
sleep 2