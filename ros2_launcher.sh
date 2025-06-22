#!/bin/bash

# Multi-Terminal ROS2 Launcher Script
# This script opens multiple terminals and runs different ROS2 nodes in each

# Function to check available terminals and select one
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
        echo "No suitable terminal found. Installing xterm..."
        sudo apt update && sudo apt install -y xterm
        TERMINAL_CMD="xterm"
    fi
}

# Function to launch a command in a new terminal
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
# launch_terminal "ROS2 Monitor" "source /opt/ros/humble/setup.bash && ros2 topic list && ros2 node list" "hold"

echo "All terminals launched successfully!"
echo "Note: Each terminal will stay open after the command finishes."

# Keep the script running briefly to show the message
sleep 2