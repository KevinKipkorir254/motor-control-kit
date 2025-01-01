# STATE SPACE CONTROLLERS

The Motor Kit includes advanced state-space control techniques to enable precise and robust control of dynamic systems.

## Features
 - *Pole Placement Design*: Customize the location of system poles to meet desired performance criteria.
 - *Quadratic Optimal Regulator Systems (LQR)*: Minimize a quadratic cost function for optimal system performance.

## Pole Placement design,

Pole placement is a state-feedback control technique where the system poles (eigenvalues of the closed-loop system) are strategically placed in the desired locations in the complex plane. This approach allows engineers to:

![POLE PLACEMENT IMAGE](/encoded_dc_motor_kit_state_space_controllers/documentation/images/pole_placement.png)

 - *Customize Performance*: Adjust system characteristics like settling time, overshoot, and damping.
 - *Achieve Stability*: Ensure the system operates within a stable region by moving poles to specific locations.
 - *Direct State Feedback*: Utilize full-state feedback to compute the required control input.


The controller gain matrix, $K_e$, is computed using linear algebraic methods, such as Ackermann's formula, to meet desired specifications.


## Quadratic Optimal Regulator Systems (LQR)

The Linear Quadratic Regulator (LQR) is an optimal control technique that minimizes a quadratic cost function.

![LQR](/encoded_dc_motor_kit_state_space_controllers/documentation/images/lqr.png)

Where:

- x: State vector
- u: Control input
- Q: State weight matrix
- R: Control weighting matrix

Key Benefits:

- *Optimal Control*: Balances minimizing state error (performance) and control effort (energy cost).
- *Robustness*:  Provides good performance even in the presence of small disturbances or parameter uncertainties.
- *Ease of Tuning*: Adjust Q and R to prioritize performance or minimize control effort.

LQR is especially useful for systems with multiple inputs and outputs (MIMO), as it provides an elegant way to handle these complexities.


## RUNNING THE KIT WITH THE STATE-SPACE CONTROLLERS.

## 1. Start RVIZ (Optional visualization)
Launch RViz for real-time visualization of the motor and its performance:
```bash
ros2 launch encoded_dc_motor_kit_description  rviz.launch.py
```
If visualization is not required, skip RViz and directly launch the motor kit server:
```bash
ros2 launch encoded_dc_motor_kit_description  motor_kit_server.launch.py
```
## 2. Start the Velocity Publisher GUI

Use the velocity GUI to set target velocities:
```bash
ros2 run encoded_dc_motor_kit_gui velocity_publisher_gui.py
```

![VELOCITY GUI](/encoded_dc_motor_kit_state_space_controllers/documentation/images/velocity_gui.png)

## 3. Run State-Space Controllers

Start the desired controllers for the system:

 - **Pole Placement Controller**

```bash
ros2 run encoded_dc_motor_kit_state_space_controllers pole_placement_contoller
```

 - **LQR Controller**

```bash
ros2 run encoded_dc_motor_kit_state_space_controllers LQR_contoller
```

## 4. Start Data Visualization

For advanced data monitoring and visualization, launch PlotJuggler:

```bash
ros2 run plotjuggler plotjuggler
```

