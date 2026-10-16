# Remotely Operated Underwater Vehicle (ROV) Control System

[![CI](https://github.com/TabishShahMohsin/pySub/actions/workflows/ci.yml/badge.svg)](https://github.com/TabishShahMohsin/pySub/actions/workflows/ci.yml)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

Software to control a Remotely Operated Underwater Vehicle (ROV) with 8 thrusters in V configuration. It consists of two programs: a base-station control program and a program that runs on the ROV's Raspberry Pi.

![Banner](Banner.png)

## System Architecture

- **Base Station (`base_station/`):**
  - Runs on a laptop or desktop computer.
  - Takes input from a keyboard or game controller (Xbox, PS).
  - Calculates the thruster forces for 6-DOF movement (surge, sway, heave, roll, pitch, yaw).
  - Sends control commands to the ROV over UDP at 20 Hz.
  - Receives and displays telemetry and video from the ROV.

- **ROV/Pi (`pi/`):**
  - Runs on a Raspberry Pi mounted on the ROV, connected to a Pixhawk running ArduSub over USB (`/dev/ttyACM0`).
  - Forwards the commands to the 8 thrusters as MAVLink RC overrides (Pixhawk outputs in RC passthrough mode).
  - Reads attitude (roll, pitch, yaw) and the external Bar30 pressure sensor (`SCALED_PRESSURE2`) from the Pixhawk.
  - Streams video from a RealSense camera and a USB camera back to the base station.
  - Watchdog: if no command arrives for 1 s, all thrusters are set to neutral.

```
 Base station                                   ROV (Raspberry Pi)
┌─────────────────────────────┐   UDP 5006   ┌──────────────────────────────┐
│ input → PID/KF → allocation ├─────────────►│ command_receiver ─► Pixhawk  │
│        (30 Hz loop)         │  PWM @ 20 Hz │ watchdog (neutral after 1 s) │
│                             │   UDP 5005   │                              │
│ telemetry_listener          │◄─────────────┤ sensor_sender (20 Hz)        │
│                             │  TCP 5555    │                              │
│ video_receiver (imagezmq)   │◄─────────────┤ video_stream_loop            │
└─────────────────────────────┘              └──────────────────────────────┘
```

## Core Logic

1.  **Input:** The base station captures the desired movement from the controller.
2.  **Control:** Optional PID controllers hold a target depth, roll, pitch, or yaw. A Kalman filter can smooth the depth measurement. Both run only on fresh telemetry samples and are reset if telemetry is lost.
3.  **Kinematics:** The desired forces and torques are allocated to the 8 thrusters (pseudo-inverse of the allocation matrix), with lateral and vertical groups saturated independently.
4.  **Communication:**
    - The base station sends thruster PWM commands to the Pi.
    - The Pi sends sensor telemetry (attitude, pressure) back to the base station.
    - The Pi streams video from its cameras to the base station.
    - On exit, the base station sends neutral and the Pi disarms.

## Repository Layout

| Path | Contents |
| --- | --- |
| `base_station/` | Base-station program (`main.py`), configuration, controllers, PID, Kalman filter, thrust allocation |
| `pi/` | ROV-side program (`main.py`) |
| `tests/` | Automated test suite (runs in CI, no hardware needed) |
| `hardware_tests/` | Manual bring-up scripts for thrusters, IMU, pressure sensor, cameras and network (need the real hardware) |
| `deploy/`, `scripts/` | systemd unit and deploy script for the Pi |
| `.github/workflows/` | CI and release pipelines |

## How to Run

Install the dependencies:

```bash
pip install -r requirements-base-station.txt   # on the laptop
scripts/deploy_pi.sh --install-deps            # from the laptop: copy to the Pi and install there
```

1.  **On the ROV's Raspberry Pi:** `python3 pi/main.py` (or `sudo systemctl start rov-pi` after deploying).
2.  **On the base station:** `python3 base_station/main.py`.
    Quit with the window close button or `q` in a video window; thrusters are sent neutral on exit.

> **Safety:** the Pi program arms the vehicle when it starts. Keep the propellers clear before starting it. The systemd service is deliberately not enabled at boot.

## Configuration

- **Network:** The Pi's hostname defaults to `auv.local` and the base station's to `mba.local`. Override them with the `ROV_PI_HOST` (base station) and `ROV_BASE_HOST` (Pi) environment variables. On the Pi, the service also reads them from `rov.env` in the install directory.
- **Controller:** The input device (Keyboard, Xbox, PS) is selected in `base_station/config.py`. Use `hardware_tests/controller_test.py` to check axis/button indices.
- **Tuning:** PID gains, sensor offsets, and thruster inversion/enable flags are in `base_station/config.py`.
- **Hardware:** The logical-to-physical Pixhawk pin mapping (`THRUSTER_MAP`) is in `pi/main.py`.

### Keyboard controls

| Keys | Motion |
| --- | --- |
| W / S | Surge |
| A / D | Sway |
| Space / Left Shift | Heave |
| Q / E | Yaw |
| I / K | Pitch |
| J / L | Roll |

## Development

```bash
pip install -r requirements-dev.txt
pre-commit install            # format + lint on every commit
python -m pytest              # run the test suite
ruff format . && ruff check . # format and lint by hand
```

The test suite covers the PID, Kalman filter, thrust allocation (including golden values from the original implementation), controller input, and both programs' threads over real localhost sockets. A fake MAVLink connection checks that the Pixhawk is never accessed from two threads at once.

## CI/CD

- **CI** (`.github/workflows/ci.yml`) runs on every push and pull request: ruff format check, ruff lint, shellcheck, and the test suite on Python 3.10 to 3.13.
- **Release** (`.github/workflows/release.yml`): pushing a tag such as `v1.0.0` runs CI and publishes a GitHub release with separate base-station and Pi bundles.
- **Deploy to the ROV:** `scripts/deploy_pi.sh [--install-deps] [--restart]` syncs the Pi code over SSH, installs dependencies into a virtualenv and installs the `rov-pi` systemd unit.
- **Dependabot** keeps GitHub Actions and Python dependencies up to date.
