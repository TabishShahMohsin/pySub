# Remotely Operated Underwater Vehicle (ROV) Control System

This project contains the software to control a Remotely Operated Underwater Vehicle (ROV). It consists of two main components: a base station control program and a program that runs on the ROV's Raspberry Pi.

## System Architecture

- **Base Station (`base_station/`):**
  - Runs on a laptop or desktop computer.
  - Takes input from a keyboard or game controller (Xbox, PS).
  - Calculates the necessary thruster forces for 6-DOF movement (surge, sway, heave, roll, pitch, yaw).
  - Sends control commands to the ROV over the network (UDP).
  - Receives and displays telemetry data and video from the ROV.

- **ROV/Pi (`pi/`):**
  - Runs on a Raspberry Pi mounted on the ROV.
  - Receives control commands from the base station.
  - Controls 8 thrusters via PWM signals using the `pigpio` library.
  - Reads data from an IMU (for roll, pitch, yaw) and a pressure sensor (for depth).
  - Streams video from a RealSense camera and a standard Pi camera back to the base station.

## Core Logic

1.  **Input:** The `base_station` captures user input for desired movement.
2.  **Control:**
    - Optional PID controllers can be enabled to automatically maintain a target depth, roll, pitch, or yaw.
    - A Kalman filter is available to smooth the depth measurement.
3.  **Kinematics:** The desired movements are translated into individual forces required for each of the 8 thrusters.
4.  **Communication:**
    - The base station sends thruster PWM commands to the Pi.
    - The Pi sends sensor telemetry (IMU, pressure, temperature) back to the base station.
    - The Pi streams video from its cameras to the base station.

## How to Run

1.  **On the ROV's Raspberry Pi:**
    - Ensure `pigpiod` service is running (`sudo pigpiod`).
    - Run the main Pi script: `python3 pi/main.py`.

2.  **On the Base Station Computer:**
    - Run the main base station script: `python3 base_station/main.py`.

## Configuration

- **Network:** IP addresses for the Pi and base station are set in `pi/main.py` and `base_station/config.py`.
- **Controller:** The input device (Keyboard, Xbox, PS) can be selected in `base_station/config.py`.
- **Tuning:** PID gains, sensor offsets, and thruster inversion/enable flags are located in `base_station/config.py`.
- **Hardware Pins:** GPIO pin assignments for the thrusters are defined in `pi/main.py`.
