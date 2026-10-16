import math

import numpy as np
import pygame
import pytest

import input_handler as ih


class FakeJoystick:
    def __init__(self, axes, buttons=None):
        self.axes = axes
        self.buttons = buttons or {}

    def get_axis(self, i):
        return self.axes[i]

    def get_button(self, i):
        return self.buttons.get(i, 0)


def make(cls, axes, buttons=None, deadzone=0.1):
    controller = cls.__new__(cls)
    controller.joystick = FakeJoystick(axes, buttons)
    controller.deadzone = deadzone
    return controller


@pytest.mark.parametrize(
    ("cls", "axes", "buttons"),
    [
        # Xbox: LX, LY, LT, RX, RY, RT; RB = 5
        (ih.XboxController, [1, -1, -1, 0.0, 0.0, 1], {5: 1}),
        # PS: LX, LY, RX, RY, L2, R2; R1 = 10
        (ih.PSController, [1, -1, 0.0, 0.0, -1, 1], {10: 1}),
    ],
)
def test_gamepads_return_six_dof(cls, axes, buttons):
    v = make(cls, axes, buttons).get_input_vector()
    assert v.shape == (6,)
    surge, sway, heave, roll, pitch, yaw = v
    assert math.hypot(surge, sway) == pytest.approx(1.0)  # Clipped to unit circle
    assert surge > 0 and sway > 0
    assert heave == 1.0
    assert yaw == pytest.approx(1.0)


def test_triggers_at_rest_give_no_yaw():
    v = make(ih.XboxController, [0, 0, -1, 0, 0, -1]).get_input_vector()
    assert np.allclose(v, 0)


def test_deadzone_zeroes_small_inputs():
    v = make(ih.XboxController, [0.05, -0.05, -1, 0.05, 0.05, -1]).get_input_vector()
    assert np.allclose(v, 0)


def test_keyboard_ramps_towards_target(monkeypatch):
    pressed = {pygame.K_w}

    class Keys:
        def __getitem__(self, key):
            return key in pressed

    monkeypatch.setattr(pygame.key, "get_pressed", lambda: Keys())
    kb = ih.KeyboardController()
    values = [kb.get_input_vector()[0] for _ in range(15)]
    assert values[0] == pytest.approx(0.1)
    assert values[-1] == 1.0  # Saturates at the target

    pressed.clear()
    for _ in range(15):
        v = kb.get_input_vector()
    assert v[0] == 0.0
