
## RUNNING THE KIT WITH THE P, PI, AND PID CONTROLLERS.

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
ros2 run encoded_dc_motor_kit_PID p_controller
ros2 run encoded_dc_motor_kit_PID pi_controller
ros2 run encoded_dc_motor_kit_PID pid_controller
ros2 run encoded_dc_motor_kit_PID two_dof_pid_controller
```

start data visualisation
```bash
ros2 run plotjuggler plotjuggler
```
