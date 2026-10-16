"""Base-station configuration: network, geometry, controller and tuning.

Thruster layout (top view)::

            ---------------------------width----------------------------
        |    T1 (CW)                                         T2 (CW)
        |
        |           T5 (CCW)                          T6 (CW)
        |                               x
        |                               |
      length                            o ----- y
        |                                \\
        |                                 z (into the page / downwards)
        |
        |           T7 (CW)                            T8 (CCW)
        |
        |   T3 (CCW)                                           T4 (CCW)

Force exerted by a thruster is taken +ve when it points along +x for the
lateral thrusters (T1-T4) and downwards (+z) for the vertical thrusters
(T5-T8), irrespective of CW/CCW propeller.

Units: lengths are in mm and thrust in the units of ``MAX_THRUST``. Torques
are therefore in thrust*mm. Positions and limits use the same units, so the
allocation is consistent.
"""

import math
import os

import numpy as np

# ---------------------------------------------------------------------------
# Network
# ---------------------------------------------------------------------------
# Hostname (or IP) of the ROV's Raspberry Pi. Override with ROV_PI_HOST.
# It is resolved lazily (with retries) by the command sender, so a missing
# mDNS entry no longer crashes the program at import time.
PI_HOST = os.environ.get("ROV_PI_HOST", "auv.local")
UDP_PORT_DATA = 5005  # Pi -> base station telemetry
UDP_PORT_CMD = 5006  # Base station -> Pi thruster commands
VIDEO_PORT = 5555  # imagezmq video stream

COMMAND_RATE_HZ = 20  # Rate at which PWM commands are sent to the Pi
CONTROL_RATE_HZ = 30  # Base-station main-loop rate
TELEMETRY_TIMEOUT_S = 1.0  # Telemetry older than this is treated as lost

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
ROV_WIDTH_MM = 262.629
ROV_LENGTH_MM = 195.311

# "V" configuration: T1(45), T2(-45), T3(135), T4(-135)
# Angle from the +x axis of the force exerted on the vehicle.
THRUSTER_ANGLES_DEG = [45, -45, 135, -135]

SIN_45 = math.sin(math.radians(45))

# Max thrust allowed from an individual thruster. Limits are expressed in
# thrust (not PWM) so that moments cancel correctly.
MAX_THRUST = 2.35

PWM_NEUTRAL = 1500

# ---------------------------------------------------------------------------
# Input device
# ---------------------------------------------------------------------------
XBOX = "XBOX"
PS = "PS"
KEYBOARD = "KEYBOARD"
CONTROLLER_TYPE = KEYBOARD

# ---------------------------------------------------------------------------
# Thruster direction inversion
# ---------------------------------------------------------------------------
# Swapping two ESC phase wires inverts a thruster's direction. Four propellers
# must spin one way and four the other to cancel the counter-rotor torque.
I1 = False
I2 = True
I3 = True
I4 = True
I5 = True
I6 = True
I7 = False
I8 = False

INVERTED_THRUSTERS = [I1, I2, I3, I4, I5, I6, I7, I8]


def invert_pwm(pwm, invert=True):
    """Mirror a PWM value about neutral when ``invert`` is set."""
    if invert:
        return PWM_NEUTRAL - (pwm - PWM_NEUTRAL)
    return pwm


# ---------------------------------------------------------------------------
# Working thrusters
# ---------------------------------------------------------------------------
# Six thrusters (3 lateral + 3 vertical) are enough for 6 DOF. Cancelling the
# counter-rotor torque with one thruster missing is impossible, so that is not
# compensated for when a thruster is disabled here.
W1 = True
W2 = True
W3 = True
W4 = True
W5 = True
W6 = True
W7 = True
W8 = True

WORKING_THRUSTERS = np.array([W1, W2, W3, W4, W5, W6, W7, W8])

# These assume all thrusters work. Recompute them for a reduced set, otherwise
# the thrusters saturate before the joystick reaches its extremes.
MAX_AXIAL_FORCE = 4 * SIN_45 * MAX_THRUST
MAX_YAW_TORQUE = 2 * SIN_45 * (ROV_LENGTH_MM + ROV_WIDTH_MM) * MAX_THRUST
MAX_HEAVE_FORCE = 4 * MAX_THRUST
MAX_ROLL_TORQUE = 4 * (ROV_WIDTH_MM / 2) * MAX_THRUST
MAX_PITCH_TORQUE = 4 * (ROV_LENGTH_MM / 2) * MAX_THRUST

# To be implemented
RECORD_SENSORS = False
RECORD_FOOTAGE = False

# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
DEPTH_KF = False

SURFACE_PRESSURE_MBAR = 1013.25
WATER_DENSITY = 1025  # kg/m^3 (use ~997 for fresh water)
GRAVITY = 9.81  # m/s^2

# Difference for location and weather (mbar)
PRESSURE_OFFSET = 988 - 1013.25 + 1020.91 - 1013.25
ROLL_OFFSET = 0
PITCH_OFFSET = 0
YAW_OFFSET = 50  # Set this so that yaw = 0 points towards the gate

# ---------------------------------------------------------------------------
# PID gains
# ---------------------------------------------------------------------------
# Depth: metres -> command in [-1, 1]
DEPTH_PID = False
DEPTH_KP = 1.2
DEPTH_KI = 0.0
DEPTH_KD = 0.0

# Pitch: degrees -> command in [-1, 1]
PITCH_PID = False
PITCH_KP = 0.04
PITCH_KI = 0.005
PITCH_KD = 0.03

# Roll: degrees -> command in [-1, 1]
ROLL_PID = False
ROLL_KP = 0.04
ROLL_KI = 0.005
ROLL_KD = 0.03

# Yaw: degrees -> command in [-1, 1]
YAW_PID = False
YAW_KP = 0.0014
YAW_KI = 0.000
YAW_KD = 0.0

# Setpoint slew rates while the stick is held (per second)
DEPTH_SETPOINT_RATE = 0.5  # m/s
ANGLE_SETPOINT_RATE = 20.0  # deg/s
