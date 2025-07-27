# Autonomous Driving Platform for a 1/10 Scale Vehicle

This repository contains the complete software stack for a 1/10 scale autonomous vehicle, integrating ROS, hardware control, and advanced path planning algorithms. The project is designed to perform autonomous navigation using sensor data, global path planning, and local path tracking controllers.

![Hybrid A* Animation](hybrid_a_star_animation.gif)
***Note: To display the animation, create a GIF named `hybrid_a_star_animation.gif` from the simulation and upload it to the root of this repository.***

## Project Overview

The architecture is built around the Robot Operating System (ROS) and uses a combination of off-the-shelf and custom components to achieve autonomous navigation.

### Core Components:
*   **Vehicle:** A 1/10 scale RC car platform.
*   **High-Level Computing:** Raspberry Pi 4 running ROS for perception, planning, and control logic.
*   **Low-Level Control:** Arduino Mega 2560 for real-time sensor reading (IMU) and actuator control (ESC and Servo) via PWM signals.
*   **Global Path Planner:** A **Hybrid A* planner** (`hybrid_a_star.py`) that generates smooth, kinematically feasible long-range paths.
*   **Local Path Tracking:** Implementations of **Stanley** and **LQR** controllers that follow the global path with high precision.
*   **Simulation & Modeling:** Vehicle dynamics and controller performance were modeled and tested using **MATLAB, Simulink, and CARSIM**.

## System Architecture and Data Flow

The system operates on a hierarchical control loop where a global plan is generated and then followed by a local controller.

1.  **Sensing & Localization**:
    *   A LIDAR sensor provides environmental scans (`/scan`) and an IMU provides orientation data (`/imu/data`).
    *   A SLAM algorithm (like `hector_slam`) uses this data to generate a map (`/map`) and determine the vehicle's position (`/slam_out_pose`).

2.  **Global Planning**:
    *   The **Hybrid A* planner** node subscribes to the map, the vehicle's pose, and a goal set in RViz (`/move_base_simple/goal`).
    *   It computes an optimal path and publishes it as a series of waypoints to the `/globalpath` topic.

3.  **Local Path Tracking**:
    *   A local path tracking node (e.g., `stanley.py`) subscribes to `/globalpath` and the vehicle's current state.
    *   It calculates the required steering angle and speed in real-time, publishing them as `/cmd_vel` messages.

4.  **Actuation**:
    *   The Arduino, connected to the Raspberry Pi via `rosserial`, subscribes to `/cmd_vel`.
    *   It translates these velocity commands into low-level PWM signals to drive the steering servo and the motor's ESC.

The data flow is as follows:
**Sensors → SLAM (`/map`, `/slam_out_pose`) → Hybrid A* (`/globalpath`) → Stanley/LQR Controller (`/cmd_vel`) → Arduino → Actuators**


