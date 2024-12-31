# INTRODUCTION
## RUNNING THE KIT WITH THE RVIZ VISUALISATION.
1. Go to the 
```bash
control_kit_ws
```

2. Run 
```bash
source install/setup.bash
```
3. Run 
```bash
ros2 launch encoded_dc_motor_kit_description rviz.launch.py
```
4. Added an effort controller
```bash
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

5. Added a simple gui

```bash
ros2 run encoded_dc_motor_kit_gui voltage_publisher_gui.py
```
