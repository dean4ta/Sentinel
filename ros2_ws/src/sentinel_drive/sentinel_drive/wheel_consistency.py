"""Mecanum leave-one-out wheel consistency math. Node-free for unit tests.

Sign conventions match cmd_vel_serial's mecanum IK: x forward, y left,
CCW positive, and the IK wheel terms
    fl = vx - vy - wL, fr = vx + vy + wL,
    bl = vx + vy - wL, br = vx - vy + wL
where wL = omega_z * (half-track + half-wheelbase). All velocities are
wheel surface speeds in m/s.
"""

from dataclasses import dataclass
from statistics import median
from typing import Dict, Tuple

WHEELS = ('FL', 'FR', 'BL', 'BR')

# v_i = vx + vy_sign_i * vy + w_sign_i * wL
_VY_SIGN = {'FL': -1.0, 'FR': 1.0, 'BL': 1.0, 'BR': -1.0}
_W_SIGN = {'FL': -1.0, 'FR': 1.0, 'BL': -1.0, 'BR': 1.0}


def ik_all(vx: float, vy: float, wL: float) -> Dict[str, float]:
    """Inverse kinematics: twist -> per-wheel surface speeds (m/s)."""
    return {w: vx + _VY_SIGN[w] * vy + _W_SIGN[w] * wL for w in WHEELS}


def fk_three(v: Dict[str, float], excluded: str) -> Tuple[float, float, float]:
    """Forward kinematics from the three non-excluded wheels.

    Returns (vx, vy, wL). Closed form per exclusion, derived from the IK
    above; each of the three remaining wheels gives one exact equation.
    """
    fl, fr, bl, br = (v['FL'], v['FR'], v['BL'], v['BR'])
    if excluded == 'FL':      # use FR, BL, BR
        vy = (fr - br) / 2.0
        wL = (fr - bl) / 2.0
        vx = fr - vy - wL
    elif excluded == 'FR':    # use FL, BL, BR
        vy = (bl - fl) / 2.0
        wL = (br - fl) / 2.0
        vx = fl + vy + wL
    elif excluded == 'BL':    # use FL, FR, BR
        vy = (fr - br) / 2.0
        wL = (br - fl) / 2.0
        vx = br + vy - wL
    else:                     # excluded == 'BR': use FL, FR, BL
        vy = (bl - fl) / 2.0
        wL = (fr - bl) / 2.0
        vx = bl - vy + wL
    return vx, vy, wL


def predict(vx: float, vy: float, wL: float, wheel: str) -> float:
    """IK prediction of one wheel's speed from a twist estimate."""
    return vx + _VY_SIGN[wheel] * vy + _W_SIGN[wheel] * wL


def leave_one_out(v: Dict[str, float]) -> Dict[str, Dict[str, float]]:
    """For each wheel: FK from the other three, predict it, residual.

    Returns {wheel: {'predicted': .., 'residual': .., 'vx': ..,
    'vy': .., 'wL': ..}}. residual = measured - predicted, so positive
    means the wheel spins faster than its peers imply.
    """
    out = {}
    for w in WHEELS:
        vx, vy, wL = fk_three(v, w)
        out[w] = {
            'vx': vx, 'vy': vy, 'wL': wL,
            'predicted': predict(vx, vy, wL, w),
            'residual': v[w] - predict(vx, vy, wL, w),
        }
    return out


def suspicion(residual: float, measured: float, target: float,
              peer_errors: Tuple[float, ...],
              peer_measured: Tuple[float, ...],
              peer_targets: Tuple[float, ...], threshold: float,
              scale: float, tracking_scale: float,
              margin: float) -> float:
    """Asymmetric slip suspicion in [0, 1], sign-aware.

    Everything is projected onto the wheel's direction of travel:
    overspeed means larger MAGNITUDE, so a slipping wheel gets a
    negative residual when driving in reverse. Gates, in order:
    1. Target-excess attribution: the wheel's speed excess over its own
       target (|measured| - |target|) must exceed every peer's excess
       by at least margin. A slipping wheel beats its target while
       loaded peers lag theirs, so it wins this comparison. This also
       kills leave-one-out smear: an outlier distorts the 3-wheel fits
       so a healthy partner can show a same-direction residual, but a
       healthy wheel does not beat its own target by more than the
       outlier does. (Raw |measured| vs max peer does NOT work here:
       with heterogeneous targets, e.g. combined motion, a slipped slow
       wheel can stay below a fast commanded peer.)
    2. Direction-projected residual above threshold.
    3. Tracking evidence scales the score: a wheel that also tracks its
       command much better than its peers (slipping lifts load, so the
       controller nails the target while loaded wheels lag) is more
       suspicious.
    """
    excess = abs(measured) - abs(target)
    peer_excess = max(abs(m) - abs(t)
                      for m, t in zip(peer_measured, peer_targets))
    if excess - peer_excess <= margin:
        return 0.0
    pred = measured - residual
    if pred > 0.0:
        s = 1.0
    elif pred < 0.0:
        s = -1.0
    else:
        s = 1.0 if target >= 0.0 else -1.0
    over = residual * s - threshold
    if over <= 0.0:
        return 0.0
    over = min(1.0, over / scale)
    err = abs(target - measured)
    peer_med = median(peer_errors)
    adv = min(1.0, max(0.0, (peer_med - err) / tracking_scale))
    return over * (0.5 + 0.5 * adv)


@dataclass
class ConfidenceFilter:
    """One-pole confidence filter with asymmetric rates.

    Suspicion drops confidence fast (alpha_suspicious), recovery is
    slow (alpha_recover) — hysteresis against single noisy samples.
    """
    alpha_suspicious: float = 0.2
    alpha_recover: float = 0.02
    confidence: float = 1.0

    def update(self, suspicion: float) -> float:
        target = 1.0 - min(1.0, max(0.0, suspicion))
        a = (self.alpha_suspicious if target < self.confidence
             else self.alpha_recover)
        self.confidence += a * (target - self.confidence)
        return self.confidence
