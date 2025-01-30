# Compensators

Compensators are used in control systems to modify the dynamic behavior of the system, enhancing its performance in terms of stability, speed, or steady-state error. They work by shaping the open-loop transfer function to meet specific performance requirements.

## Lead compensator

The Lead Compensator improves the system's transient response by adding phase lead to the system. This compensator increases the system's stability margin, enhances the speed of response, and reduces rise time. It is particularly effective for systems with sluggish responses or poor phase margins.

Key features of a lead compensator:

 - **Phase Lead**: Improves phase margin and system stability.
 - **Faster Response**: Reduces rise and settling times.
 - **Increased Bandwidth**: Improves the system's ability to respond to higher-frequency inputs

![LEAD COMPENSTOR](/encoded_dc_motor_kit_compensator/documentation/images/lead%20compensator.png)

## matlab design parameter expectations

![Matlab design](/encoded_dc_motor_kit_compensator/documentation/images/lead_compensator/matlab_design.png)


lead compensator equation:
<div align="center">

$$
lead-compensator = \frac{ 136.9*(s + 20.95)}{s + 16.8}
$$



| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.1430 | |
| Settling time | 0.6708 | |
| Settling min | 2.1559 | |
| Settling max | 2.4842 | |
| overshoot | 5.2680 | |
| undershoot| 0 | |
| Peak| 2.4842 | |
| Peak Time| 0.3100 | |
| Steady state error| 0.8 | |

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_compensator/documentation/images/lead_compensator/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.0844 | |
| Settling time | 15.9894 | |
| Settling min | 1.9377 | |
| Settling max | 3.2485 | |
| overshoot | 41.44079 | |
| undershoot| 0 | |
| Peak| 3.248591 | |
| Peak Time| 4.43633 | |
| Steady state error| 2.28 | |

*N/B: the step test is carried out from 2-3 since the lead compensator does not behave well at reference of one also since the the range 0-90 V of the motor has no effect in increasing the velocity if the motor is at zero. Inquire if this is correct?*
</div>

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.1430 | 0.0844 | |
| Settling time | 0.6708 | 15.9894 | since oscillatory it would not settle |
| Settling min | 2.1559 | 1.9377 | |
| Settling max | 2.4842 | 3.2485 | |
| overshoot | 5.2680 | 41.44079 | |
| undershoot| 0 | 0 | |
| Peak| 2.4842 | 3.248591 | |
| Peak Time| 0.3100 | 4.43633 | |
| Steady state| 2.4 | 2.28 | |

</div>

## RUNNING THE KIT WITH THE LEAD COMPENSATOR

## 1. Start RViz (Optional Visualization)

Launch RViz to visualize the motor and its responses in real time:

```bash
ros2 launch encoded_dc_motor_kit_description  rviz.launch.py
```

If visualization is not required, you can directly launch the motor kit server:

```bash
ros2 launch encoded_dc_motor_kit_description  motor_kit_server.launch.py
```

## 2. Start the Velocity Publisher GUI

The GUI allows you to define and publish velocity commands:

```bash
ros2 run encoded_dc_motor_kit_gui velocity_publisher_gui.py
```

## 3. Start the Lead Compensator

Run the lead compensator node to control the motor's velocity based on the desired characteristics:

```bash
ros2 run encoded_dc_motor_kit_compensator lead_compensator
```

## 4. Start Data Visualization

Use PlotJuggler to monitor real-time data and analyze system performance:

```bash
ros2 run plotjuggler plotjuggler
```

## Data Monitoring
The topics to subscribe to in PlotJuggler are provided in the image below:

![PLOT JUGGLER IMAGE](/encoded_dc_motor_kit_compensator/documentation/images/velocity_publisher.png)

This setup allows you to observe critical parameters such as velocity commands, motor responses, and compensator actions, enabling thorough performance analysis
