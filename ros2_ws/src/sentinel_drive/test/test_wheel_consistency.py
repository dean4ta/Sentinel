"""Unit tests for wheel_consistency.py.

pytest-compatible, also runnable standalone:
    python3 test_wheel_consistency.py
"""

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sentinel_drive.wheel_consistency import (  # noqa: E402
    WHEELS, ConfidenceFilter, fk_three, ik_all, leave_one_out,
    predict, suspicion)

T = 0.05   # residual threshold m/s (matches drive_params.yaml)
S = 0.15   # residual scale
TS = 0.30  # tracking scale
M = 0.03   # peer margin


def assert_close(a, b, tol=1e-9):
    assert abs(a - b) < tol, f'{a} != {b} (tol {tol})'


def sus(w, loo, measured, targets, errs, thresh=T, scale=S,
        track=TS, margin=M):
    peers = tuple(p for p in WHEELS if p != w)
    return suspicion(loo[w]['residual'], measured[w], targets[w],
                     tuple(errs[p] for p in peers),
                     tuple(measured[p] for p in peers),
                     tuple(targets[p] for p in peers),
                     thresh, scale, track, margin)


def test_ik_fk_roundtrip_forward():
    v = ik_all(1.0, 0.0, 0.0)
    for w in WHEELS:
        vx, vy, wL = fk_three(v, w)
        assert_close(vx, 1.0)
        assert_close(vy, 0.0)
        assert_close(wL, 0.0)
        assert_close(predict(vx, vy, wL, w), v[w])


def test_ik_fk_roundtrip_strafe():
    v = ik_all(0.0, 0.5, 0.0)
    for w in WHEELS:
        vx, vy, wL = fk_three(v, w)
        assert_close(vx, 0.0)
        assert_close(vy, 0.5)
        assert_close(wL, 0.0)


def test_ik_fk_roundtrip_rotation():
    v = ik_all(0.0, 0.0, 0.3)
    for w in WHEELS:
        vx, vy, wL = fk_three(v, w)
        assert_close(vx, 0.0)
        assert_close(vy, 0.0)
        assert_close(wL, 0.3)


def test_ik_fk_roundtrip_combined():
    v = ik_all(1.0, -0.4, 0.2)
    for w in WHEELS:
        vx, vy, wL = fk_three(v, w)
        assert_close(vx, 1.0)
        assert_close(vy, -0.4)
        assert_close(wL, 0.2)


def test_normal_consistent_no_suspicion():
    # 1.0 m/s forward, all wheels tracking closely.
    v = ik_all(1.0, 0.0, 0.0)
    loo = leave_one_out(v)
    errs = {w: 0.05 for w in WHEELS}
    for w in WHEELS:
        assert abs(loo[w]['residual']) < 1e-9
        assert sus(w, loo, v, v, errs) == 0.0


def test_one_wheel_overspeeding():
    # Command 1.0 m/s: three wheels at ~0.75, BR at 1.10 -> suspicious.
    # BR also tracks the command much better than the loaded wheels.
    measured = {'FL': 0.75, 'FR': 0.75, 'BL': 0.75, 'BR': 1.10}
    targets = {w: 1.0 for w in WHEELS}
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)

    s_br = sus('BR', loo, measured, targets, errs)
    assert s_br > 0.3  # fast decay + tracking advantage -> clearly flagged

    # Peers: smear can give them positive residuals (the 3-wheel fits
    # absorb BR's offset), but healthy peers lag their targets while BR
    # beats its, so the target-excess gate keeps them at zero.
    for w in ('FL', 'FR', 'BL'):
        assert sus(w, loo, measured, targets, errs) == 0.0


def test_one_wheel_slower_stays_trusted():
    # Asymmetry: a wheel slower than predicted must not be suspicious.
    measured = {'FL': 0.75, 'FR': 0.75, 'BL': 0.75, 'BR': 0.55}
    targets = {w: 1.0 for w in WHEELS}
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)
    assert loo['BR']['residual'] < 0.0
    assert sus('BR', loo, measured, targets, errs) == 0.0


def test_reverse_overspeeding():
    # -1.0 m/s, BR slips: |measured| up to 1.3. Residual is negative in
    # reverse (measured - predicted = -0.3); sign projection must still
    # flag BR, and the smear on the other wheels (positive residuals)
    # must be killed by the sign projection alone.
    targets = ik_all(-1.0, 0.0, 0.0)
    measured = dict(targets)
    measured['BR'] = -1.3
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)

    assert loo['BR']['residual'] < 0.0
    assert sus('BR', loo, measured, targets, errs) > 0.3
    # Smear hits all three peers (wrong-way for FL/FR, same-way for BL);
    # sign projection kills the first, the target-excess gate the second.
    for w in ('FL', 'FR', 'BL'):
        assert sus(w, loo, measured, targets, errs) == 0.0


def test_rotation_slip_negative_axle():
    # CCW rotation: FL and BL spin backwards (-0.3), FR/BR forwards.
    # FL slips to -0.45. Its forward partner FR gets a same-direction
    # smear residual, but the margin gate must keep FR at zero.
    targets = ik_all(0.0, 0.0, 0.3)
    measured = dict(targets)
    measured['FL'] = -0.45
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)

    assert sus('FL', loo, measured, targets, errs) > 0.3
    assert sus('FR', loo, measured, targets, errs) == 0.0
    assert sus('BL', loo, measured, targets, errs) == 0.0
    assert sus('BR', loo, measured, targets, errs) == 0.0


def test_strafe_negative_slip():
    # Strafe left (vy = -0.5): FR and BL spin backwards. FR slips to
    # -0.7 -> flagged; healthy negative partner BL gets a wrong-way
    # smear residual and must stay trusted.
    targets = ik_all(0.0, -0.5, 0.0)
    measured = dict(targets)
    measured['FR'] = -0.7
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)

    assert sus('FR', loo, measured, targets, errs) > 0.3
    assert sus('BL', loo, measured, targets, errs) == 0.0
    assert sus('FL', loo, measured, targets, errs) == 0.0
    assert sus('BR', loo, measured, targets, errs) == 0.0


def test_combined_negative_slip():
    # All four wheels backwards with different magnitudes. BL slips:
    # its magnitude grows but stays below the fast commanded BR (-1.3),
    # so the margin gate must compare target-excess, not raw |speed|.
    targets = ik_all(-0.8, 0.3, -0.2)
    assert targets['BL'] < 0.0 and targets['BR'] < 0.0
    measured = dict(targets)
    measured['BL'] = -0.55
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)

    assert sus('BL', loo, measured, targets, errs) > 0.3
    # BR gets a large wrong-direction smear residual (its 3-wheel fit
    # absorbs BL's slip) but loses the excess gate -> zero.
    assert loo['BR']['residual'] < 0.0
    assert sus('BR', loo, measured, targets, errs) == 0.0
    assert sus('FL', loo, measured, targets, errs) == 0.0
    assert sus('FR', loo, measured, targets, errs) == 0.0


def test_spec_example_confidence_below_half():
    # Reviewer spec example: command 1.0, FL .81, FR .78, BL .80,
    # BR .99 (one wheel barely overspeeding loaded wheels). The filter
    # must settle well below 0.5 for BR, zero suspicion elsewhere.
    measured = {'FL': 0.81, 'FR': 0.78, 'BL': 0.80, 'BR': 0.99}
    targets = {w: 1.0 for w in WHEELS}
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)

    s = sus('BR', loo, measured, targets, errs)
    assert s > 0.7
    for w in ('FL', 'FR', 'BL'):
        assert sus(w, loo, measured, targets, errs) == 0.0

    f = ConfidenceFilter(alpha_suspicious=0.2, alpha_recover=0.02)
    conf = f.confidence
    for _ in range(20):
        conf = f.update(s)
    assert conf < 0.5


def test_noise_below_threshold_ignored():
    random.seed(7)
    v = ik_all(1.0, 0.0, 0.0)
    for _ in range(100):
        noisy = {w: v[w] + random.uniform(-0.01, 0.01) for w in WHEELS}
        loo = leave_one_out(noisy)
        errs = {w: abs(v[w] - noisy[w]) for w in WHEELS}
        for w in WHEELS:
            # 3-wheel FK sums three noisy terms, worst case ~4x the
            # per-wheel noise = 0.04 < threshold 0.05 -> zero.
            assert abs(loo[w]['residual']) < T
            assert sus(w, loo, noisy, v, errs) == 0.0


def test_slip_drops_confidence_and_recovery_is_gradual():
    f = ConfidenceFilter(alpha_suspicious=0.2, alpha_recover=0.02)
    measured = {'FL': 1.0, 'FR': 1.0, 'BL': 1.0, 'BR': 1.3}
    targets = {w: 1.0 for w in WHEELS}
    errs = {w: abs(targets[w] - measured[w]) for w in WHEELS}
    loo = leave_one_out(measured)
    s = sus('BR', loo, measured, targets, errs)
    assert 0.0 < s < 1.0  # real slip -> positive but not full score

    # Consistent samples: confidence stays ~1.
    for _ in range(10):
        conf = f.update(0.0)
    assert conf > 0.999

    # Slip: confidence decays fast but not instantly to 0 (filtering).
    conf = f.update(s)
    assert conf < 1.0
    for _ in range(10):
        conf = f.update(s)
    assert conf < 1.0 - 0.8 * s  # most of the way to the target

    # Recovery: gradual, monotonic, not a single-sample jump.
    conf = f.update(0.0)
    assert conf < 1.0
    for _ in range(200):
        prev = conf
        conf = f.update(0.0)
        assert conf >= prev
    assert conf > 0.9


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'{name}: ok')
    print('all tests passed')
