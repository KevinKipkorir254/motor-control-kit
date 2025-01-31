# P, PI, AND PID CONTROLLERS

The Motor Kit supports various types of classical controllers, including Proportional (P), Proportional-Integral (PI), and Proportional-Integral-Derivative (PID) controllers, as well as advanced variations like the Two Degrees of Freedom (2-DOF) PID controller. These controllers allow you to experiment with different control strategies to achieve desired system performance.

 - **P Controller (Proportional)**:
The P controller applies a correction proportional to the current error. It improves response speed but may result in steady-state error.

![Proportional](/encoded_dc_motor_kit_PID/documentation/images/Proportional.png)

## matlab design parameter expectations

![P controller design](/encoded_dc_motor_kit_PID/documentation/images/p_controller/p_controller.png)


Gain controller equation:
<div align="center">

$$
Gain controller = 302.19
$$



| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.0986 | |
| Settling time | 0.3168 | |
| Settling min | 2.3527 | |
| Settling max | 2.7977 | |
| overshoot | 7.5244 | |
| undershoot| 0 | |
| Peak| 2.7977 | |
| Peak Time| 0.2100 | |
| Steady state error| 2.6 | |

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_PID/documentation/images/p_controller/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.07615 | |
| Settling time | 9.76602 | |
| Settling min | 2.0809 | |
| Settling max | 3.85374 | |
| overshoot | 47.20678 | |
| undershoot| 0 | |
| Peak| 3.85374 | |
| Peak Time| 0.1559 | |
| Steady state error| 2.6179 | |

*N/B: the step test is carried out from 2-3 since the lead compensator does not behave well at reference of one also since the the range 0-90 V of the motor has no effect in increasing the velocity if the motor is at zero. Inquire if this is correct?*
</div>

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.0986 | 0.07615 | |
| Settling time | 0.3168 | 9.76602 | since oscillatory it would not settle |
| Settling min | 2.3527 | 2.0809 | |
| Settling max | 2.7977 | 3.85374 | |
| overshoot | 7.5244 | 47.20678 | |
| undershoot| 0 | 0 | |
| Peak| 2.7977 | 3.85374 | |
| Peak Time| 0.2100 | 0.1559 | |
| Steady state| 2.6 | 2.6179 | |

</div>


 - **PI Controller (Proportional-Integral)**:
The PI controller combines proportional action with integral action, which accumulates past errors to eliminate steady-state error, making it ideal for systems requiring high accuracy.
## matlab design parameter expectations

![PI controller design](/encoded_dc_motor_kit_PID/documentation/images/pi_controller/pi_controller.png)


Gain controller equation:
<div align="center">

$$
PI controller = \frac{205.25*(s + 0.06618)}{s}
$$



| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.1428 | |
| Settling time | 3.3341 | |
| Settling min | 2.4267| |
| Settling max | 2.6651 | |
| overshoot | 0.2645 | |
| undershoot| 0 | |
| Peak| 2.6651 | |
| Peak Time| 0.2700 | |
| Steady state error| N/A | eventually gets there|

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_PID/documentation/images/pi_controller/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.068955 | |
| Settling time | 7.50920 | |
| Settling min | 2.23946 | |
| Settling max | 3.81484 | |
| overshoot | 39.9950 | |
| undershoot| 0 | |
| Peak| 3.81484 | |
| Peak Time| 0.15904 | |
| Steady state error| NA | Eventually gets there due to the integrating action|

</div>

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.1428 | 0.068955 | |
| Settling time | 3.3341 | 7.50920 | since oscillatory it would not settle |
| Settling min | 2.4267 | 2.23946 | |
| Settling max | 2.6651 | 3.81484 | |
| overshoot | 0.2645 | 39.9950 | |
| undershoot| 0 | 0 | |
| Peak| 2.6651 | 3.81484 | |
| Peak Time| 0.2700 | 0.15904 | |
| Steady state| N/A | N/A | |

</div>


 - **PID Controller (Proportional-Integral-Derivative)**:
The PID controller adds derivative action to predict future errors and dampen oscillations, providing a balanced approach for systems requiring stability and precision.

## matlab design parameter expectations

![PI controller design](/encoded_dc_motor_kit_PID/documentation/images/pi_controller/pi_controller.png)


Gain controller equation:
<div align="center">

$$
PI controller = \frac{205.25*(s + 0.06618)}{s}
$$



| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.1428 | |
| Settling time | 3.3341 | |
| Settling min | 2.4267| |
| Settling max | 2.6651 | |
| overshoot | 0.2645 | |
| undershoot| 0 | |
| Peak| 2.6651 | |
| Peak Time| 0.2700 | |
| Steady state error| N/A | eventually gets there|

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_PID/documentation/images/pi_controller/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.068955 | |
| Settling time | 7.50920 | |
| Settling min | 2.23946 | |
| Settling max | 3.81484 | |
| overshoot | 39.9950 | |
| undershoot| 0 | |
| Peak| 3.81484 | |
| Peak Time| 0.15904 | |
| Steady state error| NA | Eventually gets there due to the integrating action|

</div>

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.1428 | 0.068955 | |
| Settling time | 3.3341 | 7.50920 | since oscillatory it would not settle |
| Settling min | 2.4267 | 2.23946 | |
| Settling max | 2.6651 | 3.81484 | |
| overshoot | 0.2645 | 39.9950 | |
| undershoot| 0 | 0 | |
| Peak| 2.6651 | 3.81484 | |
| Peak Time| 0.2700 | 0.15904 | |
| Steady state| N/A | N/A | |

</div>



- **Two-DOF PID Controller** :
The Two-Degrees-of-Freedom PID Controller is an advanced variation of the traditional PID controller, designed to provide greater flexibility in control system performance. Unlike a standard PID controller, which uses a single tuning parameter set, the 2-DOF PID controller separates the tuning for setpoint tracking and disturbance rejection.

![two degree PID controller](/encoded_dc_motor_kit_PID/documentation/images/two_degrees_of_freedom.png)

These controllers are simple yet effective, making them a cornerstone of control system design and an excellent starting point for experimenting with the Motor Kit.

## Steps to Run the Controllers

## 1. Start RViz (Optional Visualization)

Launch RViz to visualize the motor and its performance in real time

```bash
ros2 launch encoded_dc_motor_kit_description  rviz.launch.py
```
If visualization is not required, you can launch the motor kit server directly:

```bash
ros2 launch encoded_dc_motor_kit_description  motor_kit_server.launch.py
```
## 2. Start the Velocity Publisher GUI
Use the GUI to set target velocities for the motor:

```bash
ros2 run encoded_dc_motor_kit_gui velocity_publisher_gui.py
```
## 3. Start the Controllers
Run the desired controllers to test their performance:

- **P controller**
```bash
ros2 run encoded_dc_motor_kit_PID p_controller
```
 - **PI Controller**

```bash
ros2 run encoded_dc_motor_kit_PID pi_controller
```

 - **PID Controller**

```bash
ros2 run encoded_dc_motor_kit_PID pid_controller
```

 - **Two-DOF PID Controller**

```bash
ros2 run encoded_dc_motor_kit_PID two_dof_pid_controller
```

Each controller adjusts the motor's response based on the error signal, with:

 - *P controller* focusing on proportional error correction.
 - *PI controller* adding integral action to eliminate steady-state errors.
 - *PID controller* combining proportional, integral, and derivative actions for precise control.
 - *Two-DOF PID controller* offering better control flexibility by separately tuning setpoint and feedback responses.


## 4. Start Data Visualization

For real-time performance monitoring and advanced data visualization, launch PlotJuggler:
```bash
ros2 run plotjuggler plotjuggler
```
