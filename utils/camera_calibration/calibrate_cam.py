#!/usr/bin/env python3
"""T003 — calibrate the Sentinel 1 C270 from apriltag-grid photos.

Runs on the Orin, pure python3 + cv2 + numpy + dt-apriltags (+ PyYAML
if available). No ROS.

Detected corners lie on the outer edge of the tag's black border, i.e.
8/10 of the printed side (printed side includes the 1-cell white quiet
zone; verified empirically in T002). Object corners are therefore placed
at center +- QUIET_ZONE_FRACTION * side / 2. A uniform scale of the
object pattern does not change the recovered intrinsics; the factor only
keeps the extrinsics physically true.

Corner correspondence: detection corner indices follow a library-
specific convention relative to the tag orientation. Instead of relying
on it, each image's matched tag centers define a homography from the
board plane (inches) to image pixels — exact for a planar pattern. Each
tag's four object corners are projected through it and assigned to the
nearest detection corner, so object corner k always pairs with the same
physical corner across every tag and image. selftest.py proves this by
recovering truth corners through a permuted detector convention.

YAML output: prefers PyYAML (ROS distros ship it); if `import yaml`
fails, a hand-written but valid plain-YAML emitter writes the same
content, so the script has no hard PyYAML dependency.

Exit codes:
  0  success (camera_matrix.yaml written)
  1  fewer than MIN_USABLE_IMAGES usable images
  2  bad arguments / missing files / detector not installed
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    from dt_apriltags import Detector
except ImportError:  # pragma: no cover — the Orin has it
    Detector = None

QUIET_ZONE_FRACTION = 0.8
MIN_USABLE_IMAGES = 5
CORNER_MATCH_MAX_PX = 20.0  # skip tags whose corners land far off


def load_grid(json_path):
    """Return {tag_id: (cx_in, cy_in, side_in)} from the grid JSON."""
    data = json.loads(Path(json_path).read_text())
    meta = data.get("meta", {})
    if not meta.get("size_in_includes_quiet_zone", False):
        raise ValueError(
            f"{json_path}: meta.size_in_includes_quiet_zone must be true")
    return {t["id"]: (t["center"]["x_in"], t["center"]["y_in"],
                      t["size_in"]) for t in data["tags"]}


def tag_object_corners(cx, cy, side):
    """The 4 black-border corners of a tag, order TL, TR, BR, BL (in)."""
    h = QUIET_ZONE_FRACTION * side / 2.0
    return [(cx - h, cy - h), (cx + h, cy - h),
            (cx + h, cy + h), (cx - h, cy + h)]


def fit_homography(src, dst):
    """Least-squares homography mapping src -> dst (N >= 4), DLT via SVD."""
    n = len(src)
    a = np.zeros((2 * n, 9))
    for i, ((x, y), (u, v)) in enumerate(zip(src, dst)):
        a[2 * i] = [x, y, 1, 0, 0, 0, -u * x, -u * y, -u]
        a[2 * i + 1] = [0, 0, 0, x, y, 1, -v * x, -v * y, -v]
    _, _, vt = np.linalg.svd(a)
    hh = vt[-1].reshape(3, 3)
    return hh / hh[2, 2]


def apply_homography(hh, pts):
    """Apply a homography to (N,2) points, returning (N,2)."""
    pts = np.atleast_2d(np.asarray(pts, dtype=float))
    t = hh @ np.column_stack([pts, np.ones(len(pts))]).T
    return (t[:2] / t[2]).T


def detection_fields(d):
    """(tag_id, corners (4,2), center (2,)) in a detector-agnostic shape."""
    tid = int(d.tag_id)
    corners = np.asarray(d.corners, dtype=float).reshape(-1, 2)
    center = np.asarray(d.center, dtype=float).ravel()[:2]
    return tid, corners, center


def match_detections(grid_tags, detections):
    """Match detection corners to grid object corners.

    Returns (obj_pts (M,3) in, img_pts (M,2) px, n_tags) with object
    corner k corresponding to image corner k for every matched tag, or
    (None, None, n) if the image is unusable (fewer than 4 tags).
    """
    det_by_id = {}
    for d in detections:
        tid, _, _ = detection_fields(d)
        if tid in grid_tags and tid not in det_by_id:
            det_by_id[tid] = d
    ids = sorted(det_by_id)
    if len(ids) < 4:
        return None, None, len(ids)
    src = np.array([grid_tags[i][:2] for i in ids], dtype=float)
    dst = np.array([detection_fields(det_by_id[i])[2] for i in ids],
                   dtype=float)
    hh = fit_homography(src, dst)

    obj_list, img_list = [], []
    used = 0
    for i in ids:
        cx, cy, side = grid_tags[i]
        corners_obj = np.asarray(tag_object_corners(cx, cy, side),
                                 dtype=float)
        _, corners_img, _ = detection_fields(det_by_id[i])
        proj = apply_homography(hh, corners_obj)
        order = [int(np.argmin(np.linalg.norm(corners_img - p, axis=1)))
                 for p in proj]
        if len(set(order)) != 4:
            continue  # ambiguous assignment; skip this tag
        if np.linalg.norm(corners_img[order] - proj, axis=1).max() \
                > CORNER_MATCH_MAX_PX:
            continue  # corners far from the board model; skip this tag
        obj_list.append(np.column_stack([corners_obj, np.zeros(4)]))
        img_list.append(corners_img[order])
        used += 1
    if used < 4:
        return None, None, used
    return np.vstack(obj_list), np.vstack(img_list), used


def calibrate_from_corners(obj_points_list, img_points_list, image_size):
    """Run cv2.calibrateCamera on the planar views.

    Returns (rms_px, K, dist, rvecs, tvecs, per_image_errors_px).

    Points are converted to float32: OpenCV >= 5.0 rejects float64
    calibration inputs, float32 is accepted by every 4.x/5.x release.
    """
    obj_arr = [np.asarray(op, dtype=np.float32).reshape(-1, 3)
               for op in obj_points_list]
    img_arr = [np.asarray(ip, dtype=np.float32).reshape(-1, 2)
               for ip in img_points_list]
    _, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_arr, img_arr, image_size, None, None)
    sq_errors = []
    per_image = []
    for op, ip, rv, tv in zip(obj_arr, img_arr, rvecs, tvecs):
        proj, _ = cv2.projectPoints(op, rv, tv, K, dist)
        d = np.linalg.norm(proj.reshape(-1, 2) - ip.reshape(-1, 2),
                           axis=1)
        per_image.append(float(d.mean()))
        sq_errors.append(d)
    rms = float(np.sqrt(np.mean(np.concatenate(sq_errors) ** 2)))
    return rms, K, dist, rvecs, tvecs, per_image


def calibration_data(rms, K, dist, image_size, per_image_errors):
    """Build the result dict written to camera_matrix.yaml."""
    return {
        "camera_matrix": np.asarray(K, dtype=float).reshape(3, 3).tolist(),
        "dist_coeffs": np.asarray(dist, dtype=float).ravel().tolist(),
        "image_size": [int(image_size[0]), int(image_size[1])],
        "rms": float(rms),
        "per_image_errors": [float(e) for e in per_image_errors],
    }


def plain_yaml(data):
    """Hand-written but valid YAML emitter (PyYAML-free fallback)."""
    lines = ["camera_matrix:"]
    for row in data["camera_matrix"]:
        lines.append("  - [" + ", ".join(repr(float(v)) for v in row) + "]")
    lines.append("dist_coeffs: [" +
                 ", ".join(repr(float(v)) for v in data["dist_coeffs"])
                 + "]")
    lines.append("image_size: [" + ", ".join(str(int(v))
                                             for v in data["image_size"])
                 + "]")
    lines.append("rms: " + repr(float(data["rms"])))
    lines.append("per_image_errors:")
    for e in data["per_image_errors"]:
        lines.append("  - " + repr(float(e)))
    return "\n".join(lines) + "\n"


def dump_yaml(out_path, data):
    """Write the calibration result as YAML.

    Prefers PyYAML (safe_dump); falls back to plain_yaml(). Returns the
    writer name used (for the log line).
    """
    try:
        import yaml  # noqa: F401
        text = yaml.safe_dump(data, default_flow_style=False,
                              sort_keys=False)
        writer = "PyYAML"
    except ImportError:
        text = plain_yaml(data)
        writer = "fallback text writer (PyYAML not installed)"
    Path(out_path).write_text(text)
    return writer


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Calibrate the Sentinel 1 C270 from apriltag-grid "
                    "photos (T003).")
    ap.add_argument("--images", required=True,
                    help="directory of img_*.jpg captures")
    ap.add_argument("--grid", default=None,
                    help="grid JSON (default: "
                         "grid_5x4_tag36h11_ids18-37.json next to the "
                         "images dir)")
    ap.add_argument("--out", default=None,
                    help="output yaml (default: camera_matrix.yaml next "
                         "to the images dir)")
    ap.add_argument("--min-tags", type=int, default=4,
                    help="reject an image with fewer matched tags "
                         "(default 4)")
    args = ap.parse_args(argv)

    images_dir = Path(args.images)
    if not images_dir.is_dir():
        ap.error(f"--images: not a directory: {images_dir}")
    if args.min_tags < 4:
        ap.error("--min-tags must be >= 4")
    grid_path = Path(args.grid) if args.grid else (
        images_dir.parent / "grid_5x4_tag36h11_ids18-37.json")
    if not grid_path.is_file():
        ap.error(f"--grid: not a file: {grid_path}")
    out_path = Path(args.out) if args.out else (
        images_dir.parent / "camera_matrix.yaml")

    if Detector is None:
        print("ERROR: dt-apriltags is not installed on the Orin "
              "(run: pip3 install -r requirements.txt)", file=sys.stderr)
        return 2
    try:
        grid_tags = load_grid(grid_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        ap.error(f"cannot load grid {grid_path}: {exc}")
    print(f"grid: {grid_path} ({len(grid_tags)} tags)")

    image_files = sorted(images_dir.glob("img_*.jpg"))
    if not image_files:
        print(f"ERROR: no img_*.jpg found in {images_dir}",
              file=sys.stderr)
        return 2

    detector = Detector(families="tag36h11", quad_decimate=1.0)

    obj_list, img_list, used_paths = [], [], []
    image_size = None
    for path in image_files:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"rejected {path.name}: unreadable image")
            continue
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        detections = detector.detect(gray, estimate_tag_pose=False)
        obj_pts, img_pts, n_tags = match_detections(grid_tags, detections)
        if obj_pts is None or n_tags < args.min_tags:
            print(f"rejected {path.name}: {n_tags} grid tags matched "
                  f"(need >= {args.min_tags})")
            continue
        obj_list.append(obj_pts)
        img_list.append(img_pts)
        used_paths.append(path)
        print(f"using {path.name}: {n_tags} tags, {len(obj_pts)} corners")

    if len(obj_list) < MIN_USABLE_IMAGES:
        print(f"ERROR: only {len(obj_list)} usable image(s) — need at "
              f"least {MIN_USABLE_IMAGES}; capture more photos with "
              "different board poses", file=sys.stderr)
        return 1

    rms, K, dist, _, _, errors = calibrate_from_corners(
        obj_list, img_list, image_size)
    for path, err in zip(used_paths, errors):
        print(f"reproj {path.name}: {err:.3f} px")
    print(f"images: {len(obj_list)} used, "
          f"{len(image_files) - len(obj_list)} rejected")
    print(f"RMS: {rms:.4f} px")
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    k1, k2, p1, p2, k3 = dist.ravel()
    print(f"fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}")
    print(f"k1={k1:.5f} k2={k2:.5f} p1={p1:.5f} p2={p2:.5f} k3={k3:.5f}")
    data = calibration_data(rms, K, dist, image_size, errors)
    writer = dump_yaml(out_path, data)
    print(f"wrote {out_path} ({writer})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
