<<<<<<< HEAD
# KALMAN FILTER 

The Kalman filter is a powerful recursive algorithm used for estimating the state of a dynamic system. By combining a mathematical model of the system with noisy measurements, it provides an optimal estimate of the system's state, minimizing the effects of noise and uncertainty. Kalman filters are widely applied in control systems, navigation, and signal processing due to their ability to handle uncertainties effectively.

In this package, we employ three different models during the prediction stage:

 - **ARX (Auto-Regressive with Exogenous inputs)**: A simple model that relates the current output to past outputs and current inputs.
 - **ARMAX (Auto-Regressive Moving Average with Exogenous inputs)**: An extension of ARX, incorporating a moving average term for better noise modeling.
 - **BJ (Box-Jenkins)**: A flexible model that combines ARMAX with additional noise dynamics, providing enhanced performance in systems with significant noise.

![KALMAN FILTER](/encoded_dc_motor_kit_kalman_filters/documentation/images/kalman%20filter%20flowchart.png)


=======
>>>>>>> parent of 681ba09 ("added a kalman filter README")
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
