# GUIDE TO RUNNING THE KIT FOR MY PC AT THE MOMENT.
1. Make sure everything on the hardware is connected properly.
2. Connect the Motor Kit to the PC.
3. Upload any code you want from arduino IDE on windows.

## SETTING UP WSL TO ACCESS THE USB PORT.
1. Open an ubuntu terminal and a windows terminal.
2. On the windows terminal run 

```bash
usbipd list
``` 
with this you will know the avaible connected devices.

3. Then run 
```bash
usbipd bind --busid 4-4
```
 the 4-4 should be the appropriate bus ID.

4. Run 
```bash
usbipd attach --wsl --busid \<busid\>
```
.
5. On WSL command prompt run 
```bash
lsusb
```
 this will ensure you have the USB can be accessed properly.

6. In case we are to dettach for example to run some arduino code from windows run 
```bash
usbipd detach --busid \<busid\>
```
.

7. Run 
```bash
dmesg | grep tty
```
 to see the available ports.

8. To test the data feed from the port run 
```bash
screen /dev/ttyUSB0 115200
```
.

## RUNNING THE KIT WITH THE RVIZ VISUALISATION.
1. Go to the 
```bash
control_kit_ws
```
.

2. Run 
```bash
source install/setup.bash
```
.
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
.


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





