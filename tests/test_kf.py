import numpy as np
import pytest

from kf import DepthKalmanFilter


def test_converges_to_constant_depth():
    kf = DepthKalmanFilter()
    rng = np.random.default_rng(0)
    for _ in range(300):
        est = kf.update(2.0 + rng.normal(0, 0.05), 0.05)
    assert est == pytest.approx(2.0, abs=0.03)


def test_tracks_constant_descent_rate():
    kf = DepthKalmanFilter()
    dt, rate = 0.05, 0.2
    for k in range(400):
        kf.update(rate * k * dt, dt)
    assert kf.x[1, 0] == pytest.approx(rate, abs=0.01)


def test_predict_only_extrapolates_velocity():
    kf = DepthKalmanFilter(initial_depth=1.0)
    kf.x[1, 0] = 0.5
    assert kf.predict(0.2) == pytest.approx(1.1)


def test_covariance_stays_symmetric_positive_definite():
    kf = DepthKalmanFilter()
    for k in range(200):
        kf.update(0.01 * k, 0.05)
    assert np.allclose(kf.P, kf.P.T)
    assert np.all(np.linalg.eigvalsh(kf.P) > 0)
