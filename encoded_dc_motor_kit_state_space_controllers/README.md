## RUNNING THE KIT WITH THE STATE-SPACE CONTROLLERS.

Start RVIZ
```bash
ros2 launch encoded_dc_motor_kit_description  rviz.launch.py
```

or without rviz visualisation;
```bash
ros2 launch encoded_dc_motor_kit_description  motor_kit_server.launch.py
```

start controller to determine velocity
```bash
ros2 run encoded_dc_motor_kit_gui velocity_publisher_gui.py
```

start the lead compensator
```bash
ros2 run encoded_dc_motor_kit_state_space_controllers pole_placement_contoller
ros2 run encoded_dc_motor_kit_state_space_controllers LQR_contoller
```

start data visualisation
```bash
ros2 run plotjuggler plotjuggler
```
