"""Input devices. Every controller returns a 6-element numpy array:

[surge, sway, heave, roll, pitch, yaw], each in [-1, 1]
"""

import math

import numpy as np
import pygame

from config import CONTROLLER_TYPE, KEYBOARD, PS, XBOX


def _clip_to_unit_circle(a, b):
    """Scale (a, b) back onto the unit circle if a stick reports r > 1."""
    r = math.hypot(a, b)
    if r > 1:
        return a / r, b / r
    return a, b


def _open_joystick(label):
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError(f"Error: No {label} controller detected.")
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"{label} Controller Connected: {joystick.get_name()}")
    return joystick


class XboxController:
    """Reads an Xbox controller (pygame 2 / SDL2 axis layout)."""

    def __init__(self, deadzone=0.1):
        self.joystick = _open_joystick("Xbox")
        self.deadzone = deadzone

    def get_input_vector(self):
        js = self.joystick
        surge = -js.get_axis(1)  # Left stick Y (inverted)
        sway = js.get_axis(0)  # Left stick X

        heave = 0.0
        if js.get_button(5):  # RB
            heave += 1.0
        if js.get_button(4):  # LB
            heave -= 1.0

        yaw = (js.get_axis(5) - js.get_axis(2)) / 2  # RT - LT
        pitch = -js.get_axis(4)  # Right stick Y (inverted for nose up)
        roll = js.get_axis(3)  # Right stick X

        surge, sway = _clip_to_unit_circle(surge, sway)
        pitch, roll = _clip_to_unit_circle(pitch, roll)

        inputs = np.array([surge, sway, heave, roll, pitch, yaw])
        inputs[np.abs(inputs) < self.deadzone] = 0.0
        return inputs


class PSController:
    """Reads a PS4/PS5 controller (pygame 2 / SDL2 axis layout).

    Untested on hardware: run hardware_tests/controller_test.py to confirm the axis and
    button indices for your controller before relying on it.
    """

    def __init__(self, deadzone=0.1):
        self.joystick = _open_joystick("PS")
        self.deadzone = deadzone

    def get_input_vector(self):
        js = self.joystick
        surge = -js.get_axis(1)  # Left stick Y (inverted)
        sway = js.get_axis(0)  # Left stick X

        heave = 0.0
        if js.get_button(10):  # R1
            heave += 1.0
        if js.get_button(9):  # L1
            heave -= 1.0

        yaw = (js.get_axis(5) - js.get_axis(4)) / 2  # R2 - L2
        pitch = -js.get_axis(3)  # Right stick Y (inverted for nose up)
        roll = js.get_axis(2)  # Right stick X

        surge, sway = _clip_to_unit_circle(surge, sway)
        pitch, roll = _clip_to_unit_circle(pitch, roll)

        inputs = np.array([surge, sway, heave, roll, pitch, yaw])
        inputs[np.abs(inputs) < self.deadzone] = 0.0
        return inputs


class KeyboardController:
    """Mimics a controller with the keyboard for 6-DOF control.

    The pygame window must have focus for key presses to register.
    """

    def __init__(self, deadzone=0.0):
        print("6-DoF Keyboard Control Mode Active:")
        print(" - W / S: Surge")
        print(" - A / D: Sway")
        print(" - Q / E: Yaw")
        print(" - Space / L-Shift: Heave")
        print(" - I / K: Pitch")
        print(" - J / L: Roll")

        # Smoothed state, each in [-1, 1]
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.ramp_speed = 0.1  # Change per call (called at the control rate)

    @staticmethod
    def _axis(keys, positive, negative):
        return float(keys[positive]) - float(keys[negative])

    def get_input_vector(self):
        keys = pygame.key.get_pressed()

        target_surge = self._axis(keys, pygame.K_w, pygame.K_s)
        target_sway = self._axis(keys, pygame.K_d, pygame.K_a)
        target_heave = self._axis(keys, pygame.K_SPACE, pygame.K_LSHIFT)
        target_yaw = self._axis(keys, pygame.K_q, pygame.K_e)
        target_pitch = self._axis(keys, pygame.K_i, pygame.K_k)
        target_roll = self._axis(keys, pygame.K_l, pygame.K_j)

        step = self.ramp_speed
        self.surge = self._approach(self.surge, target_surge, step)
        self.sway = self._approach(self.sway, target_sway, step)
        self.heave = self._approach(self.heave, target_heave, step)
        self.roll = self._approach(self.roll, target_roll, step)
        self.pitch = self._approach(self.pitch, target_pitch, step)
        self.yaw = self._approach(self.yaw, target_yaw, step)

        return np.array([self.surge, self.sway, self.heave, self.roll, self.pitch, self.yaw])

    @staticmethod
    def _approach(current, target, step):
        """Move current towards target by at most step."""
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current


_CONTROLLERS = {
    XBOX: XboxController,
    PS: PSController,
    KEYBOARD: KeyboardController,
}

JoystickController = _CONTROLLERS.get(CONTROLLER_TYPE.upper())
if JoystickController is None:
    print(f"Warning: Unknown controller '{CONTROLLER_TYPE}'. Falling back to keyboard.")
    JoystickController = KeyboardController
