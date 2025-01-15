# MODEL PREDICTIVE CONTROL




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

## 2. Start the model predictive control


```bash
ros2 launch encoded_dc_motor_kit_model_predictive_control model_predictive_control.launch.py
```


## Topics involved

**Debug mode off**
 - filtered_velocity

## 3. Start Data Visualization

Use PlotJuggler to monitor real-time data and analyze system performance:

```bash
ros2 run plotjuggler plotjuggler
```

  
This setup enables you to implement and evaluate state observer-based control strategies effectively. Let me know if you'd like additional technical details or example applications!

*Worked on chapter one of Model Predictive Control System design and Implementation Using Matlab*
