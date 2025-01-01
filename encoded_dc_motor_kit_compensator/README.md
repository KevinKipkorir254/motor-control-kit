# Compensators

Compensators are used in control systems to modify the dynamic behavior of the system, enhancing its performance in terms of stability, speed, or steady-state error. They work by shaping the open-loop transfer function to meet specific performance requirements.

## Lead compensator

The Lead Compensator improves the system's transient response by adding phase lead to the system. This compensator increases the system's stability margin, enhances the speed of response, and reduces rise time. It is particularly effective for systems with sluggish responses or poor phase margins.

Key features of a lead compensator:

 - **Phase Lead**: Improves phase margin and system stability.
 - **Faster Response**: Reduces rise and settling times.
 - **Increased Bandwidth**: Improves the system's ability to respond to higher-frequency inputs

![LEAD COMPENSTOR](/encoded_dc_motor_kit_compensator/documentation/images/lead%20compensator.png)

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
