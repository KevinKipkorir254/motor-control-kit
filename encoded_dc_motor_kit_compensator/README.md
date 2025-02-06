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
lead-compensator = \frac{ 185.59*(s + 16.05)}{s + 25.1}
$$



| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.1461 | |
| Settling time | 0.2191 | |
| Settling min | 1.9557 | |
| Settling max | 2.1916 | |
| overshoot | 1.5168 | |
| undershoot| 0 | |
| Peak| 2.1916 | |
| Peak Time| 0.3500 | |
| Steady state error| 2.6 | |

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_compensator/documentation/images/lead_compensator/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.1994 | |
| Settling time | 15.714 | |
| Settling min | 1.3292 | |
| Settling max | 3.0100 | |
| overshoot | 73.58 | |
| undershoot| 0 | |
| Peak| 3.010 | |
| Peak Time| 0.4486 | |
| Steady state error| 1.7340 | |

*N/B: the step test is carried out from 2-3 since the lead compensator does not behave well at reference of one also since the the range 0-90 V of the motor has no effect in increasing the velocity if the motor is at zero. Inquire if this is correct?*
</div>

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.1461 | 0.1994 | |
| Settling time | 0.2191 | 15.9894 | since oscillatory it would not settle |
| Settling min | 1.9557 | 1.3292 | |
| Settling max | 2.1916 | 3.0100 | |
| overshoot | 1.5168 | 73.58 | |
| undershoot| 0 | 0 | |
| Peak| 2.1916 | 3.0100 | |
| Peak Time| 0.3500 | 0.4486 | |
| Steady state| 2.6 | 1.7340 | |

</div>

## Lag compensator

## matlab design parameter expectations
![LAG_COMPENSATOR](/encoded_dc_motor_kit_compensator/documentation/images/lag_compensator/matlab_design.png)

Lag compensator equation:
<div align="center">

$$
lag-compensator = \frac{ 16.416*(s + 0.08774)}{s + 0.01213}
$$


| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.0889 | |
| Settling time | 2.4150 | |
| Settling min | 2.2401 | |
| Settling max | 2.7582 | |
| overshoot | 11.2518 | |
| undershoot| 0 | |
| Peak| 2.7582 | |
| Peak Time| 0.2100 | |
| Steady state | 3.0 | |

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_compensator/documentation/images/lag_compensator/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.07970 | |
| Settling time | 7.2662 | |
| Settling min | 2.07267 | |
| Settling max | 3.8804 | |
| overshoot |  46.1953 | |
| undershoot| 0 | |
| Peak| 3.88048 | |
| Peak Time| 0.15697 | |
| Steady state error| 2.6543 | |

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.0889 | 0.07970 | |
| Settling time | 2.4150 | 7.2662 | |
| Settling min | 2.2401 | 2.07267 | |
| Settling max | 2.7582 | 3.8804 | |
| overshoot | 11.2518 | 46.1953 | |
| undershoot| 0 | 0 | |
| Peak| 2.7582 | 3.88048 | |
| Peak Time| 0.2100 | 0.15697 | |
| Steady state| 3.0 | 2.6543 | |

</div>


## Lead-lag compensator
![LEAD_LAG_COMPENSATOR](/encoded_dc_motor_kit_compensator/documentation/images/lead_lag_compensator/matlab_design.png)
Lead-lag compensator equation:
<div align="center">

$$
lead-lag-compensator = \frac{ 59.449*(s + 16.05)*(s + 0.08774)}{(s + 25.1)*(s + 0.00911)}
$$


| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.0732 | |
| Settling time | 3.8688 | |
| Settling min | 2.3160 | |
| Settling max | 2.9246 | |
| overshoot | 17.2034 | |
| undershoot| 0 | |
| Peak| 2.9246 | |
| Peak Time| 0.1900 | |
| Steady state error| N/A | |

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_compensator/documentation/images/lead_lag_compensator/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.097873 | |
| Settling time | 3.0068 | |
| Settling min | 2.01522 | |
| Settling max | 3.05244 | |
| overshoot | 35.42694 | |
| undershoot| 0 | |
| Peak|  3.05244 | |
| Peak Time| 0.21667 | |
| Steady state error| 2.26  | |

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.0732 | 0.097873 | |
| Settling time | 3.8688 | 3.0068 | |
| Settling min | 2.3160 | 2.01522 | |
| Settling max | 2.9246 | 3.05244 | |
| overshoot | 17.2034 | 35.42694 | |
| undershoot| 0 | 0 | |
| Peak| 2.9246 | 3.05244 | |
| Peak Time| 0.1900 | 0.21667 | |
| Steady state| N/A | 2.26 | |

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

