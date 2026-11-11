#!/usr/bin/env python3
"""T003 selftest — synthetic calibration round trip (Mac-side).

Needs only numpy + cv2: it injects synthetic detection corners, so
dt-apriltags is NOT required and is never imported.

Pipeline exercised (the SAME code path as calibrate_cam.py):
  load_grid -> match_detections -> calibrate_from_corners

Scenario: 8 synthetic views of the 5x4 grid under a known K (fx=fy=700,
cx=640, cy=360, 1280x720) with zero distortion. Poses are random but
valid: rx/ry in +-30 deg (total tilt < 45 deg), distance 24-40 in,
pattern kept inside the frame. Each tag's projected corners are passed
in a fixed permuted order, simulating a detector whose corner-index
convention is unknown — match_detections must recover the assignment.

Assertions:
  fx, fy, cx, cy within 5% of truth
  RMS < 0.5 px
  all 8 views used, 20/20 tags matched per view
  matched corners equal the true projected corners (assignment recovery)
  plain-YAML fallback writer emits the expected keys

Exit 0 on pass, 1 on fail.
"""

import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from calibrate_cam import (  # noqa: E402
    calibrate_from_corners,
    calibration_data,
    dump_yaml,
    load_grid,
    match_detections,
    plain_yaml,
    tag_object_corners,
)

REPO_GRID = HERE.parents[1] / "generated" / "grid_5x4_tag36h11_ids18-37.json"
ORIN_GRID = HERE / "grid_5x4_tag36h11_ids18-37.json"

TRUTH_K = np.array([[700.0, 0.0, 640.0],
                    [0.0, 700.0, 360.0],
                    [0.0, 0.0, 1.0]])
IMAGE_SIZE = (1280, 720)
N_VIEWS = 8


def euler_rotation(rx, ry, rz):
    """Rotation matrix from Euler angles (rad), R = Rz @ Ry @ Rx."""
    cx_, sx_ = np.cos(rx), np.sin(rx)
    cy_, sy_ = np.cos(ry), np.sin(ry)
    cz_, sz_ = np.cos(rz), np.sin(rz)
    rx_m = np.array([[1, 0, 0], [0, cx_, -sx_], [0, sx_, cx_]])
    ry_m = np.array([[cy_, 0, sy_], [0, 1, 0], [-sy_, 0, cy_]])
    rz_m = np.array([[cz_, -sz_, 0], [sz_, cz_, 0], [0, 0, 1]])
    return rz_m @ ry_m @ rx_m


def make_view(grid_tags, rng, perm):
    """One synthetic view.

    Returns (detections, obj_pts (80,3), true_img_pts (80,2)) where the
    detection corners carry the fixed permutation `perm` inside each tag.
    """
    ids = sorted(grid_tags)
    obj_pts = np.vstack([
        np.column_stack([np.asarray(tag_object_corners(*grid_tags[i]),
                                    dtype=float), np.zeros(4)])
        for i in ids])

    tilt = 30.0  # deg; keeps total tilt below 45
    rvec, _ = cv2.Rodrigues(euler_rotation(
        np.deg2rad(rng.uniform(-tilt, tilt)),
        np.deg2rad(rng.uniform(-tilt, tilt)),
        np.deg2rad(rng.uniform(-20.0, 20.0))))
    tvec = np.array([rng.uniform(-14.0, 14.0),
                     rng.uniform(-9.0, 9.0),
                     rng.uniform(24.0, 40.0)]).reshape(3, 1)
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, TRUTH_K,
                                np.zeros(5))
    true_img = proj.reshape(-1, 2)

    detections = []
    for k, i in enumerate(ids):
        corners = true_img[4 * k:4 * k + 4][perm]  # unknown convention
        detections.append(SimpleNamespace(tag_id=i, corners=corners,
                                          center=corners.mean(axis=0)))
    return detections, obj_pts, true_img


def check(cond, label, detail=""):
    print(f"CHECK {label}: {'PASS' if cond else 'FAIL'} — {detail}")
    return bool(cond)


def main():
    grid_path = REPO_GRID if REPO_GRID.is_file() else ORIN_GRID
    if not grid_path.is_file():
        print(f"ERROR: grid JSON not found (tried {REPO_GRID} and "
              f"{ORIN_GRID})", file=sys.stderr)
        return 1
    grid_tags = load_grid(grid_path)
    print(f"grid: {grid_path} ({len(grid_tags)} tags)")

    rng = np.random.default_rng(3)
    perm = rng.permutation(4)
    print(f"synthetic detector corner convention (permutation): "
          f"{perm.tolist()}")

    obj_list, img_list = [], []
    tags_per_view = []
    max_match_err = 0.0
    for v in range(N_VIEWS):
        detections, _, true_img = make_view(grid_tags, rng, perm)
        obj_pts, img_pts, n_tags = match_detections(grid_tags, detections)
        tags_per_view.append(n_tags)
        if obj_pts is None:
            print(f"view {v}: match FAILED ({n_tags} tags)")
            continue
        err = float(np.abs(img_pts - true_img).max())
        max_match_err = max(max_match_err, err)
        obj_list.append(obj_pts)
        img_list.append(img_pts)

    ok = True
    ok &= check(all(n == len(grid_tags) for n in tags_per_view),
                "all views matched 20/20 tags",
                f"tags per view: {tags_per_view}")
    if not obj_list:
        print("RESULT: FAIL")
        return 1

    rms, K, dist, _, _, errors = calibrate_from_corners(
        obj_list, img_list, IMAGE_SIZE)

    ok &= check(max_match_err < 1e-6, "corner assignment recovered",
                f"max match err {max_match_err:.2e} px")
    ok &= check(len(obj_list) == N_VIEWS, "all 8 views used",
                f"{len(obj_list)}/{N_VIEWS}")
    ok &= check(rms < 0.5, "RMS < 0.5 px", f"RMS {rms:.5f} px")
    for name, truth, got in (("fx", 700.0, K[0, 0]),
                             ("fy", 700.0, K[1, 1]),
                             ("cx", 640.0, K[0, 2]),
                             ("cy", 360.0, K[1, 2])):
        pct = 100.0 * abs(got - truth) / truth
        ok &= check(pct < 5.0, f"{name} within 5%",
                    f"truth {truth}, got {got:.3f}, err {pct:.3f}%")
    for v, e in enumerate(errors):
        print(f"view {v}: reproj {e:.4f} px")
    print(f"dist: {np.asarray(dist).ravel().round(8)}")

    data = calibration_data(rms, K, dist, IMAGE_SIZE, errors)
    text = plain_yaml(data)
    ok &= check(all(key in text for key in
                    ("camera_matrix:", "dist_coeffs:", "image_size:",
                     "rms:", "per_image_errors:")),
                "plain-YAML fallback emits expected keys", "")
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "camera_matrix.yaml"
        writer = dump_yaml(out, data)
        ok &= check("camera_matrix" in out.read_text(),
                    "dump_yaml writes camera_matrix.yaml",
                    f"via {writer}")

    print("RESULT: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
