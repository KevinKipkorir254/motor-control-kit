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


The controller gain matrix, $K_e$, is computed using linear algebraic methods, such as Ackermann's formula, to meet desired specifications.## matlab design parameter expectations

![pole placement controller design](/encoded_dc_motor_kit_state_space_controllers/documentation/images/pole_placement_controller/design_output.png)


Gain controller equation:
<div align="center">

$$
K = [ -34.3179  -14.3974   17.2185]
$$



| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.9535 | |
| Settling time | 4.2835 | |
| Settling min | 2.7073 | |
| Settling max | 3.2242 | |
| overshoot | 7.5137 | |
| undershoot| 0 | |
| Peak| 3.2242 | |
| Peak Time| 1.6800 | |
| Steady state error| 3.0 | |

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_state_space_controllers/documentation/images/pole_placement_controller/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 3.27242 | |
| Settling time | 10.09793 | |
| Settling min | 2.6599 | |
| Settling max | 3.06337 | |
| overshoot | 3.46984 | |
| undershoot| 0 | |
| Peak| 3.06337 | |
| Peak Time| 8.83445 | |
| Steady state error| 2.96064 | |

*N/B: the step test is carried out from 2-3 since the lead compensator does not behave well at reference of one also since the the range 0-90 V of the motor has no effect in increasing the velocity if the motor is at zero. Inquire if this is correct?*
</div>

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.9535 | 3.27242 | |
| Settling time | 4.2835 | 10.09793 | since oscillatory it would not settle |
| Settling min | 2.7073 | 2.6599 | |
| Settling max | 3.2242 | 3.06337 | |
| overshoot | 7.5137 | 3.46984 | |
| undershoot| 0 | 0 | |
| Peak| 3.2242 | 3.06337 | |
| Peak Time| 1.6800 | 8.83445 | |
| Steady state| 3.0 | 2.96064 | |

</div>




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
![pole placement controller design](/encoded_dc_motor_kit_state_space_controllers/documentation/images/LQR_controller/lqr_design.png)


Gain controller equation:
<div align="center">

$$
K = [ 102.5500   10.0746]
$$



| Parameter | value | comment |
| :-------- | :--------: | :--------: |
| Rise time | 0.3226 | |
| Settling time | 0.5621 | |
| Settling min | 0.0183 | |
| Settling max | 0.0202 | |
| overshoot | 0 | |
| undershoot| 1.6811e-17 | |
| Peak| 0.0202 | |
| Peak Time|1 | |
| Steady state error| 0.2 | |

</div>

## motor kit step response

![Motor kit step response](/encoded_dc_motor_kit_state_space_controllers/documentation/images/LQR_controller/step_response_plot.png)

<div align="center">

| Function | Value | comment|
| :-------- | :--------: | :--------: |
| Rise time | 0.25697 | |
| Settling time | 18.8233 | |
| Settling min | 1.9348 | |
| Settling max | 2.5627 | |
| overshoot | 3.46984 | |
| undershoot| 0 | |
| Peak| 2.5627 | |
| Peak Time| 0.51303 | |
| Steady state error| 2.07346 | |

</div>

## Comparison step response

<div align="center">

| Function | Expected | Reality| comment|
| :-------- | :--------: | :--------: |:--------: |
| Rise time | 0.3226 | 0.25697 | |
| Settling time | 0.5621 | 18.8233 | since oscillatory it would not settle |
| Settling min | 0.0183 | 1.9348 | |
| Settling max | 0.0202 | 2.5627 | |
| overshoot | 0 | 3.46984 | |
| undershoot| 0 | 0 | |
| Peak| 0.0202 | 2.5627 | |
| Peak Time| 1 | 0.51303 | |
| Steady state| 0.2 | 2.26 | |

</div>




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

