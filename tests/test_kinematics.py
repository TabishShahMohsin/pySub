import numpy as np
import pytest

import rov_kinematics as rk
from config import MAX_THRUST, PWM_NEUTRAL
from golden_kinematics import GOLDEN_CASES


@pytest.mark.parametrize(("inputs", "forces", "pwms"), GOLDEN_CASES)
def test_matches_original_implementation(inputs, forces, pwms):
    out = rk.compute_thruster_forces(*inputs)
    assert np.allclose(out, forces, atol=1e-12)
    assert [rk.map_force_to_pwm(f) for f in out] == pwms


def test_zero_input_gives_zero_thrust():
    assert np.allclose(rk.compute_thruster_forces(0, 0, 0, 0, 0, 0), 0)


@pytest.mark.parametrize("seed", range(20))
def test_forces_never_exceed_max_thrust(seed):
    rng = np.random.default_rng(seed)
    forces = rk.compute_thruster_forces(*rng.uniform(-1, 1, 6))
    assert np.all(np.abs(forces) <= MAX_THRUST + 1e-12)


def test_pure_surge_uses_only_lateral_thrusters():
    forces = rk.compute_thruster_forces(0.3, 0, 0, 0, 0, 0)
    assert np.allclose(forces[4:], 0)
    assert np.all(np.abs(forces[:4]) > 0)


def test_pure_heave_uses_only_vertical_thrusters_equally():
    forces = rk.compute_thruster_forces(0, 0, 0.3, 0, 0, 0)
    assert np.allclose(forces[:4], 0)
    assert np.allclose(forces[4:], forces[4])


def test_unsaturated_allocation_reproduces_requested_wrench():
    # Small inputs never saturate, so pinv must solve B @ t = v exactly.
    lateral = rk._PINV_LATERAL @ np.array([1.0, 0.5, 20.0])
    b_lateral = np.linalg.pinv(rk._PINV_LATERAL)
    assert np.allclose(b_lateral @ lateral, [1.0, 0.5, 20.0])


@pytest.mark.parametrize("thrust", [0.0, 0.005, -0.005, float("nan"), float("inf")])
def test_pwm_is_neutral_for_tiny_or_invalid_thrust(thrust):
    assert rk.map_force_to_pwm(thrust) == PWM_NEUTRAL


def test_pwm_is_monotonic_and_within_esc_range():
    thrusts = np.linspace(-MAX_THRUST, MAX_THRUST, 200)
    pwms = [rk.map_force_to_pwm(t) for t in thrusts if abs(t) >= 1e-2]
    assert pwms == sorted(pwms)
    assert min(pwms) >= 1150 and max(pwms) <= 1850
