"""Thrust allocation for the 8-thruster V-configuration ROV."""

import numpy as np

from config import (
    MAX_AXIAL_FORCE,
    MAX_HEAVE_FORCE,
    MAX_PITCH_TORQUE,
    MAX_ROLL_TORQUE,
    MAX_THRUST,
    MAX_YAW_TORQUE,
    PWM_NEUTRAL,
    ROV_LENGTH_MM,
    ROV_WIDTH_MM,
    THRUSTER_ANGLES_DEG,
    WORKING_THRUSTERS,
)

# Cubic fits of PWM vs thrust for the T200 at 14 V: [t^3, t^2, t, 1]
_REVERSE_COEFFS = np.array([4.58585333, 35.21660561, 169.73509491, 1464.33710736])
_FORWARD_COEFFS = np.array([2.22716503, -22.41358258, 135.44774899, 1535.90291842])

# Thruster positions (x forward, y right) in mm
_HALF_L = ROV_LENGTH_MM / 2
_HALF_W = ROV_WIDTH_MM / 2
POSITIONS = np.array(
    [
        [_HALF_L, -_HALF_W],  # T1 (front-left)
        [_HALF_L, _HALF_W],  # T2 (front-right)
        [-_HALF_L, -_HALF_W],  # T3 (rear-left)
        [-_HALF_L, _HALF_W],  # T4 (rear-right)
        [_HALF_L, -_HALF_W],  # T5 (front-left)
        [_HALF_L, _HALF_W],  # T6 (front-right)
        [-_HALF_L, -_HALF_W],  # T7 (rear-left)
        [-_HALF_L, _HALF_W],  # T8 (rear-right)
    ]
)


def _build_allocation():
    """Build the pseudo-inverses of the lateral and vertical allocation matrices.

    Solves B @ t = v for the thrust column t, i.e. t = pinv(B) @ v. The geometry
    is constant, so this is done once at import instead of on every call.
    """
    angles = np.deg2rad(THRUSTER_ANGLES_DEG)

    b_lateral = np.zeros((3, 4))
    for i, ((x, y), angle) in enumerate(zip(POSITIONS[:4], angles)):
        b_lateral[0, i] = np.cos(angle)  # Fx (surge)
        b_lateral[1, i] = np.sin(angle)  # Fy (sway)
        b_lateral[2, i] = x * np.sin(angle) - y * np.cos(angle)  # Tz (yaw)

    b_vertical = np.zeros((3, 4))
    for i, (x, y) in enumerate(POSITIONS[4:]):
        b_vertical[0, i] = -x  # Ty (pitch)
        b_vertical[1, i] = y  # Tx (roll)
        b_vertical[2, i] = 1  # Fz (heave)

    # Zero the columns of failed thrusters. At least 3 lateral and 3 vertical
    # thrusters are required for 6 DOF.
    b_lateral = b_lateral * WORKING_THRUSTERS[:4]
    b_vertical = b_vertical * WORKING_THRUSTERS[4:]

    return np.linalg.pinv(b_lateral), np.linalg.pinv(b_vertical)


_PINV_LATERAL, _PINV_VERTICAL = _build_allocation()


def map_force_to_pwm(thrust):
    """Map a thrust to a PWM value using a regression of the T200 curve at 14 V."""
    if not np.isfinite(thrust) or abs(thrust) < 1e-2:
        return PWM_NEUTRAL

    coeffs = _REVERSE_COEFFS if thrust < 0 else _FORWARD_COEFFS
    pwm_value = float(np.array([thrust**3, thrust**2, thrust, 1.0]) @ coeffs)
    return int(round(pwm_value))


def _saturate(forces):
    """Scale a thruster group down uniformly so no thruster exceeds MAX_THRUST."""
    max_force = np.max(np.abs(forces))
    if max_force > MAX_THRUST:
        forces = forces * (MAX_THRUST / max_force)
    return forces


def compute_thruster_forces(raw_surge, raw_sway, raw_heave, raw_roll, raw_pitch, raw_yaw):
    """Convert normalised [-1, 1] motion commands into the 8 thruster forces.

    Returns forces for T1..T8 (T1-T4 lateral, T5-T8 vertical).
    """
    # The achievable planar force depends on direction (it is lower at 45 deg
    # than along x or y), so scale by the direction to avoid saturating before
    # the stick reaches its extremes.
    direction = np.arctan2(abs(raw_sway), abs(raw_surge))

    desired_surge = raw_surge * MAX_AXIAL_FORCE * np.cos(direction)
    desired_sway = raw_sway * MAX_AXIAL_FORCE * np.sin(direction)
    desired_yaw = raw_yaw * MAX_YAW_TORQUE
    desired_roll = raw_roll * MAX_ROLL_TORQUE
    desired_pitch = raw_pitch * MAX_PITCH_TORQUE
    desired_heave = raw_heave * MAX_HEAVE_FORCE

    lateral = _PINV_LATERAL @ np.array([desired_surge, desired_sway, desired_yaw])
    vertical = _PINV_VERTICAL @ np.array([desired_pitch, desired_roll, desired_heave])

    # Saturate each group separately so vertical PID effort does not reduce
    # the lateral thrust (and vice versa).
    return np.concatenate([_saturate(lateral), _saturate(vertical)])
