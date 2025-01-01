# STATE OBSERVERS
State observers are algorithms used to estimate the internal state variables of a dynamic system when they cannot be directly measured. They play a critical role in modern control systems, enabling state-feedback controllers and providing robust performance even in the presence of measurement noise or missing data.

![STATE OBSERVER](/encoded_dc_motor_kit_state_observers/documentation/images/state%20observers.png)


## RUNNING THE KIT WITH THE STATE OBSERVER

## 1. Start RViz (Optional Visualization)

Launch RViz to visualize the motor and its responses in real time:

```bash
ros2 launch encoded_dc_motor_kit_description  rviz.launch.py
```

If visualization is not required, you can directly launch the motor kit server:

```bash
ros2 launch encoded_dc_motor_kit_description  motor_kit_server.launch.py
```

## 2. Start the State observer


```bash
ros2 run encoded_dc_motor_kit_state_observers state_observer
```

## 3. Start Data Visualization

Use PlotJuggler to monitor real-time data and analyze system performance:

```bash
ros2 run plotjuggler plotjuggler
```

## Benefits of Using State Observers:
 - **Reduced Sensor Dependency**: Observers reduce the need for physical sensors, lowering costs and complexity.
 - **Noise Filtering**: Effectively filters out measurement noise to provide clean estimates.
 - **Robust Control**: Enables state-feedback control even in partially observable systems.
  
This setup enables you to implement and evaluate state observer-based control strategies effectively. Let me know if you'd like additional technical details or example applications!
