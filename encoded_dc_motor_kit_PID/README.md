# P, PI, AND PID CONTROLLERS

The Motor Kit supports various types of classical controllers, including Proportional (P), Proportional-Integral (PI), and Proportional-Integral-Derivative (PID) controllers, as well as advanced variations like the Two Degrees of Freedom (2-DOF) PID controller. These controllers allow you to experiment with different control strategies to achieve desired system performance.

 - **P Controller (Proportional)**:
The P controller applies a correction proportional to the current error. It improves response speed but may result in steady-state error.

 - **PI Controller (Proportional-Integral)**:
The PI controller combines proportional action with integral action, which accumulates past errors to eliminate steady-state error, making it ideal for systems requiring high accuracy.

 - **PID Controller (Proportional-Integral-Derivative)**:
The PID controller adds derivative action to predict future errors and dampen oscillations, providing a balanced approach for systems requiring stability and precision.

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
