
## RUNNING THE KIT WITH THE LEAD controller

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
ros2 run encoded_dc_motor_kit_compensator lead_compensator
```

start data visualisation
```bash
ros2 run plotjuggler plotjuggler
```

the topics to subscribe to are in the picture below;

![PLOT JUGGLER IMAGE](/encoded_dc_motor_kit_compensator/documentation/images/velocity_publisher.png)
