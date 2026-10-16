
# Remotely Operated Underwater Vehicle (ROV) Control System

This project contains the software to control an Underwater Remotely Operated Underwater Vehicle (ROV) with 8 thrusters in V configuration. It consists of two main components: a base station control program and a program that runs on the ROV's Raspberry Pi.

![Banner](Banner.png)

## System Architecture

- **Base Station (`base_station/`):**
  - Runs on a laptop or desktop computer.
  - Takes input from a keyboard or game controller (Xbox, PS).
  - Calculates the necessary thruster forces for 6-DOF movement (surge, sway, heave, roll, pitch, yaw).
  - Sends control commands to the ROV over the network (UDP).
  - Receives and displays telemetry data and video from the ROV.

- **ROV/Pi (`pi/`):**
  - Runs on a Raspberry Pi mounted on the ROV, connected to a Pixhawk running ArduSub over USB (`/dev/ttyACM0`).
  - Receives control commands from the base station and forwards them to the 8 thrusters as MAVLink RC overrides (Pixhawk outputs in RC passthrough mode).
  - Reads attitude (roll, pitch, yaw) and the external pressure sensor (Bar30, `SCALED_PRESSURE2`) from the Pixhawk.
  - Streams video from a RealSense camera and a USB camera back to the base station.
  - Watchdog: if no command arrives for 1 s, all thrusters are set to neutral.

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

Install the dependencies:

```bash
pip install -r requirements-base-station.txt   # on the laptop
pip install -r requirements-pi.txt             # on the Pi
```

1.  **On the ROV's Raspberry Pi:**
    - Run the main Pi script: `python3 pi/main.py`.

2.  **On the Base Station Computer:**
    - Run the main base station script: `python3 base_station/main.py`.
    - Quit with the window close button or `q` in a video window; thrusters are sent neutral on exit.

## Configuration

- **Network:** The Pi's hostname defaults to `auv.local` and the base station's to `mba.local`. Override them with the `ROV_PI_HOST` (base station) and `ROV_BASE_HOST` (Pi) environment variables. Ports are in `base_station/config.py` and `pi/main.py`.
- **Controller:** The input device (Keyboard, Xbox, PS) is selected in `base_station/config.py`. Use `tests/controller_test.py` to check axis/button indices.
- **Tuning:** PID gains, sensor offsets, and thruster inversion/enable flags are in `base_station/config.py`.
- **Hardware:** The logical-to-physical Pixhawk pin mapping (`THRUSTER_MAP`) is in `pi/main.py`.

## Development

Code is formatted and linted with [ruff](https://docs.astral.sh/ruff/) (settings in `pyproject.toml`):

```bash
ruff format . && ruff check .
```
