"""Confidence-weighted mecanum odometry math. Node-free for unit tests.

Same conventions as wheel_consistency.py and cmd_vel_serial's IK:
x forward, y left, CCW positive, wheel surface speeds in m/s,
wL = omega_z * lxy, lxy = half-track + half-wheelbase.

The estimate is a ridge-regularised weighted least squares:

    x = argmin  sum_i w_i (v_i - a_i . x)^2 + ridge * |x - x_prior|^2

With four trusted wheels A^T W A = 4 I, so the ridge is negligible. When
wheels lose confidence, directions the remaining wheels cannot observe
fall back to x_prior instead of an arbitrary minimum-norm split.
"""

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

WHEELS = ('FL', 'FR', 'BL', 'BR')

# Rows follow WHEELS order; columns are (vx, vy, wL).
A = np.array([[1.0, -1.0, -1.0],
              [1.0, 1.0, 1.0],
              [1.0, 1.0, -1.0],
              [1.0, -1.0, 1.0]])


@dataclass
class TwistEstimate:
    vx: float
    vy: float
    wL: float
    covariance: np.ndarray   # 3x3, (vx, vy, wL) in (m/s)^2
    min_eig: float           # smallest eigenvalue of A^T W A (data only)
    degraded: bool           # fewer than ~3 effective wheels


def estimate_twist(v: Dict[str, float],
                   confidence: Dict[str, float],
                   prior: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                   weight_floor: float = 0.05,
                   ridge: float = 0.05,
                   wheel_noise_mps: float = 0.02,
                   degraded_min_eig: float = 0.5) -> TwistEstimate:
    """Weighted LS twist from measured wheel speeds and confidences.

    weight_floor keeps a distrusted wheel from being exactly ignored
    (and the system from being exactly singular). min_eig is 4 with four
    full-confidence wheels, about 1 with three, and near 0 with two, so
    degraded_min_eig = 0.5 flags the two-wheel case.
    """
    vec = np.array([v[w] for w in WHEELS])
    wts = np.array([max(weight_floor, min(1.0, confidence[w]))
                    for w in WHEELS])
    x0 = np.array(prior, dtype=float)

    h_data = A.T @ (wts[:, None] * A)
    h = h_data + ridge * np.eye(3)
    x = np.linalg.solve(h, A.T @ (wts * vec) + ridge * x0)

    cov = (wheel_noise_mps ** 2) * np.linalg.inv(h)
    min_eig = float(np.linalg.eigvalsh(h_data)[0])
    return TwistEstimate(float(x[0]), float(x[1]), float(x[2]), cov,
                         min_eig, min_eig < degraded_min_eig)


def integrate_pose(x: float, y: float, yaw: float,
                   vx: float, vy: float, omega: float,
                   dt: float) -> Tuple[float, float, float]:
    """Midpoint integration of a body-frame twist into the odom frame."""
    yaw_mid = yaw + 0.5 * omega * dt
    c, s = math.cos(yaw_mid), math.sin(yaw_mid)
    return (x + (vx * c - vy * s) * dt,
            y + (vx * s + vy * c) * dt,
            yaw + omega * dt)
