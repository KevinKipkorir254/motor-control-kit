# MOTOR KIT
Motor kit a tool to practice control theory, system Identification, filter design and implementations in microcontrollers and in ROS2. 

![MOTR KIT](/encoded_dc_motor_kit_kalman_filters/documentation/images/MOTOR%20KIT.png)

## Features


 - *Control Systems*: Design and implement PID controllers, compensators, and state-space controllers.
 - *System Identification*: Tools to aid in identifying dynamic systems from input-output data.
 - *Filtering*: Implement Kalman filters, state observers, and other filtering techniques.
 - *Integration with ROS 2*: Seamlessly interfaces with ROS 2 for advanced robotics and control applications.

## Architecture

The architecture of the Motor Kit is modular and organized for easy customization. Below is an overview:

 - *Motor Kit*: Acts as the main physical interface, housing the motor and encoder.
 - *Hardware Interface*: Facilitates communication between the microcontroller and ROS 2 through serial data exchange.
 - *Controllers*: Includes:
    - *PID*: Proportional-Integral-Derivative controllers for precise control.
    - *Compensators*: To modify system dynamics.
    - *State-Space Controllers*: Advanced control for multivariable systems.
 - *Filters*: Tools for signal processing and noise reduction.
    - *Classical filters*: For implementation of low pass filters.
    - *Kalman Filters*: For optimal estimation of system states.
    - *State Observers*: To reconstruct unmeasured states.
  
![ROS2 ARCHITECTURE](/encoded_dc_motor_kit_kalman_filters/documentation/images/DESIGN%20SCHEME.png)

## Design Documentation

Detailed design documentation is available at [DOCUMENTATION](https://github.com/KevinKipkorir254/study_notes_control_kit.git)



