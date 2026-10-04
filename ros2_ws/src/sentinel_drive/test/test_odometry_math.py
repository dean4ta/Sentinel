"""Unit tests for odometry_math. Run: python3 test/test_odometry_math.py"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sentinel_drive.odometry_math import (  # noqa: E402
    WHEELS, estimate_twist, integrate_pose)
from sentinel_drive.wheel_consistency import ik_all  # noqa: E402

FULL = {w: 1.0 for w in WHEELS}
MOTIONS = {
    'forward': (1.0, 0.0, 0.0),
    'reverse': (-0.8, 0.0, 0.0),
    'strafe': (0.0, 0.6, 0.0),
    'rotate': (0.0, 0.0, 0.4),
    'combined': (0.7, -0.3, 0.25),
}


def close(a, b, tol):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def got(est):
    return (est.vx, est.vy, est.wL)


def test_all_trusted_recovers_twist():
    for name, tw in MOTIONS.items():
        est = estimate_twist(ik_all(*tw), FULL, prior=tw, ridge=0.05)
        assert close(got(est), tw, 1e-9), name
        # Zero prior only biases by ridge/(4+ridge), about 1.2 percent.
        est0 = estimate_twist(ik_all(*tw), FULL)
        assert close(got(est0), tw, 0.02 * max(map(abs, tw))), name
        assert not est0.degraded
        assert abs(est0.min_eig - 4.0) < 1e-9


def test_one_distrusted_wheel_is_rejected():
    tw = MOTIONS['combined']
    for bad in WHEELS:
        v = ik_all(*tw)
        v[bad] += 0.5            # slipping wheel overspeeds
        conf = dict(FULL)
        conf[bad] = 0.0
        est = estimate_twist(v, conf, prior=tw, weight_floor=0.01)
        assert close(got(est), tw, 0.02), bad
        assert not est.degraded

        # Same data with equal weights is visibly wrong.
        naive = estimate_twist(v, FULL, prior=tw)
        assert not close(got(naive), tw, 0.05), bad


def test_two_distrusted_wheels_degrade_and_hold_prior():
    tw = (0.5, 0.2, 0.1)
    v = ik_all(*tw)
    # Only the front pair is trusted: vx and (vy + wL) observable,
    # vy - wL is not. Corrupt the rear wheels so they cannot help.
    v['BL'] += 0.4
    v['BR'] -= 0.4
    conf = {'FL': 1.0, 'FR': 1.0, 'BL': 0.0, 'BR': 0.0}
    est = estimate_twist(v, conf, prior=tw, weight_floor=0.001)
    assert est.degraded
    assert abs(est.vx - tw[0]) < 0.02
    assert abs((est.vy + est.wL) - (tw[1] + tw[2])) < 0.02
    # Unobservable split stays near the prior instead of drifting.
    assert abs((est.vy - est.wL) - (tw[1] - tw[2])) < 0.05
    # Uncertainty on the unobservable direction is larger than healthy.
    healthy = estimate_twist(ik_all(*tw), FULL, prior=tw)
    assert est.covariance.trace() > 3.0 * healthy.covariance.trace()


def test_degraded_flag_levels():
    v = ik_all(0.5, 0.0, 0.0)
    three = {'FL': 1.0, 'FR': 1.0, 'BL': 1.0, 'BR': 0.0}
    two = {'FL': 1.0, 'FR': 1.0, 'BL': 0.0, 'BR': 0.0}
    assert not estimate_twist(v, three, weight_floor=0.0).degraded
    assert estimate_twist(v, two, weight_floor=0.0).degraded


def test_noise_is_averaged_down():
    random.seed(3)
    tw = MOTIONS['combined']
    errs = []
    for _ in range(300):
        v = {w: s + random.gauss(0.0, 0.02)
             for w, s in ik_all(*tw).items()}
        est = estimate_twist(v, FULL, prior=tw)
        errs.append(est.vx - tw[0])
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    # Four wheels average vx noise to sigma/2 = 0.01.
    assert rms < 0.015


def test_integration_straight_and_arc():
    x = y = yaw = 0.0
    for _ in range(100):
        x, y, yaw = integrate_pose(x, y, yaw, 1.0, 0.0, 0.0, 0.01)
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9

    # Left strafe while facing +y (yaw = 90 deg) moves toward -x.
    x, y, yaw = integrate_pose(0.0, 0.0, math.pi / 2, 0.0, 1.0, 0.0, 1.0)
    assert abs(x + 1.0) < 1e-9 and abs(y) < 1e-6

    # Full circle: v = 1 m/s, omega = 1 rad/s, returns to start.
    x = y = yaw = 0.0
    n = 2000
    dt = 2 * math.pi / n
    for _ in range(n):
        x, y, yaw = integrate_pose(x, y, yaw, 1.0, 0.0, 1.0, dt)
    assert abs(x) < 1e-3 and abs(y) < 1e-3


TESTS = [v for k, v in sorted(globals().items()) if k.startswith('test_')]

if __name__ == '__main__':
    for t in TESTS:
        t()
        print(f'ok  {t.__name__}')
    print(f'{len(TESTS)}/{len(TESTS)} passed')
