import pytest

from pid import PID, wrap_angle


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0, 0), (180, -180), (-180, -180), (190, -170), (-190, 170), (360, 0), (540, -180)],
)
def test_wrap_angle(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected)


def test_zero_dt_returns_previous_output():
    pid = PID(1.0, 0.0, 1.0, 1, -1)
    assert pid.compute(0.0, 0.5, 0.0) == 0.0
    out = pid.compute(0.0, 0.5, 0.1)
    assert pid.compute(0.0, 0.9, 0.0) == out


def test_no_derivative_kick_on_first_sample():
    pid = PID(1.0, 0.0, 10.0, 100, -100)
    assert pid.compute(0.0, 0.5, 0.05) == pytest.approx(0.5)


def test_proportional_and_saturation():
    pid = PID(2.0, 0.0, 0.0, 1, -1)
    assert pid.compute(0.0, 0.25, 0.1) == pytest.approx(0.5)
    assert pid.compute(0.0, 10.0, 0.1) == 1
    assert pid.compute(0.0, -10.0, 0.1) == -1


def test_angle_error_takes_shortest_path():
    pid = PID(1.0, 0.0, 0.0, 1000, -1000, is_angle=True)
    assert pid.compute(170.0, -170.0, 0.1) == pytest.approx(20.0)
    assert pid.compute(-170.0, 170.0, 0.1) == pytest.approx(-20.0)


def test_angle_derivative_does_not_jump_across_180():
    pid = PID(0.0, 0.0, 1.0, 1e9, -1e9, is_angle=True)
    pid.compute(0.0, 179.0, 0.05)
    # Setpoint moves 2 degrees (179 -> -179), not 358 degrees
    assert pid.compute(0.0, -179.0, 0.05) == pytest.approx(2.0 / 0.05)


def test_integral_contribution_is_limited_to_half_the_output_range():
    pid = PID(0.0, 0.005, 0.0, 1, -1)
    for _ in range(10_000):
        pid.compute(0.0, 50.0, 0.05)
    assert pid.output == pytest.approx(0.5)


def test_integral_unwinds_after_sign_change():
    pid = PID(0.0, 1.0, 0.0, 1, -1)
    for _ in range(100):
        pid.compute(0.0, 1.0, 0.1)
    assert pid.output == pytest.approx(0.5)
    for _ in range(100):
        pid.compute(0.0, -1.0, 0.1)
    assert pid.output == pytest.approx(-0.5)


def test_reset_clears_state():
    pid = PID(1.0, 1.0, 1.0, 1, -1)
    pid.compute(0.0, 0.3, 0.1)
    pid.reset()
    assert (pid.integral, pid.prev_error, pid.output) == (0.0, None, 0.0)
