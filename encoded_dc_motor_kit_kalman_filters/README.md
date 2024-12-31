## RUNNING THE KIT WITH THE KALMAN FILTERS WITH DIFERENT MODELS

Start RVIZ
```bash
ros2 launch encoded_dc_motor_kit_description  rviz.launch.py
```

or without rviz visualisation;
```bash
ros2 launch encoded_dc_motor_kit_description  motor_kit_server.launch.py
```

start kalman filter but with different models
```bash
ros2 launch encoded_dc_motor_kit_kalman_filters arx_kalman.launch.py
ros2 launch encoded_dc_motor_kit_kalman_filters armax_kalman.launch.py
ros2 launch encoded_dc_motor_kit_kalman_filters bj_kalman.launch.py
```

start data visualisation
```bash
ros2 run plotjuggler plotjuggler
```
start the effort controller to input voltage so we can move the motor
```bash
ros2 run encoded_dc_motor_kit_gui voltage_publisher_gui.py
```


This picture shows the topics to subscribe to:

![TOPIC VIEWR](/encoded_dc_motor_kit_kalman_filters/documentation/images/view_topics.png)
