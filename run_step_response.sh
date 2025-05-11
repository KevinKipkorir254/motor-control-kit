#!/bin/bash

# Define the base directory where the files are stored
BASE_DIR_files=~/robotics_inc/motor_kit_ws/src/motor-control-kit
BASE_DIR_files_record=~/robotics_inc/motor_kit_ws/P_70_controller

#/home/hp/robotics_inc/motor_kit_ws/src/motor-control-kit/encoded_dc_motor_kit_PID

for i in $(seq 1 20); do
    echo "Running ROS 2 step response analyzer (iteration $i)..."

    # Run the ROS 2 command
    ros2 run encoded_dc_motor_kit_response_analyzer step_response_analyzer
    
    echo "Waiting for process to finish..."
    
    # Wait until the user kills the process (e.g., Ctrl+C)
    wait

    # Create a numbered directory for filtered data
    dir_name="${BASE_DIR_files_record}/filtered_velocity_${i}"
    mkdir -p "$dir_name"

    # Move the files into the numbered directory
    mv "${BASE_DIR_files}/step_response_data.csv" "$dir_name/" 2>/dev/null
    mv "${BASE_DIR_files}/step_response_plot.png" "$dir_name/" 2>/dev/null

    echo "Saved results in $dir_name"
done

echo "Process completed."
