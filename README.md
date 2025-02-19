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

## System Interview

 - *Velocity GUI*: Provides a user-friendly interface for setting setpoint velocities to be used by the controllers.
 - *Controllers*: Handles the core logic for control systems designed to be used one at a time, each comes with a simple low pass filter. Within each package there is a readme describing how it works and how to use it.
 - *Feedback Loop*: Real-time communication between the Motor Kit, Hardware Interface, and controllers.

## Design Documentation

Detailed design documentation is available at documentation folder in each package matlab code .md

## Usage
### Prerequisites


 - A ROS 2-compatible environment.
 - A motor kit that can be order from (link to be provided).
 - Dependencies:
     - ROS 2 Humble
     - Python 3.8+
     - CMake 3.10+

### Installation

1. Clone the repository in a ROS2 workspace:

```bash
git clone https://github.com/KevinKipkorir254/control_kit_ws.git
```
2. Build and source the ROS2 workspace:

```bash
colcon build
source install/setup.bash
```

### Running the System

In each package, there is a readme on how to implement the system.

## Contributions

We welcome contributions! Please refer to the CONTRIBUTING.md file for guidelines.




