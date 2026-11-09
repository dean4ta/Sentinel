#!/usr/bin/env python3
"""Check script for T002: validate 5x4 tag36h11 apriltag grid detection
from a webcam frame (Mac-side, pure python, no ROS).

Detector choice: pupil-apriltags (pip-bundled libapriltag bindings — the
AprilRobotics C library compiled into the wheel, numpy-only interface).
The `apriltag` wheel (swatbotics) was the documented fallback but was not
needed: pupil-apriltags imports cleanly and detects correctly on this Mac.

Dependencies (all pip, run via uv): pupil-apriltags, pillow. numpy ships
with pupil-apriltags. No cv2, no rclpy, no system installs.

Usage:
  # synthetic self-test (one rendered tag, no input image needed)
  uv run --with pupil-apriltags --with pillow python3 \\
      utils/apriltag_detect/check_detect.py --selftest

  # synthetic full-grid pipeline test (renders the 5x4 layout from the
  # JSON, optionally with a perspective warp) — validates the whole check
  # before the webcam image lands
  uv run --with pupil-apriltags --with pillow python3 \\
      utils/apriltag_detect/check_detect.py --syntest [--warp]

  # real check: detect the webcam frame, assert ID set and layout
  # agreement, write an annotated PNG next to the input
  uv run --with pupil-apriltags --with pillow python3 \\
      utils/apriltag_detect/check_detect.py /tmp/sentinel/shared/webcam_5x4.jpg

Layout model: the grid JSON gives each tag's center in inches, origin at
the top-left corner of tag 0, x right, y down, with size_in including the
1-cell white quiet zone. The webcam sees a perspective view of a ground
plane, so a homography maps detected centers (px) to JSON centers (in);
the worst reprojection error is the layout-agreement metric. Pass
tolerance: worst error < 2% of printed tag side in pixels, or < 0.1 in
equivalent.

Measured convention (verified empirically against this script's renderer):
pupil-apriltags detection corners lie on the outer edge of the tag's
black border, i.e. 8/10 of the printed side (printed side includes the
white quiet zone). Tag centers are unaffected, so the homography check
does not depend on this factor; it only matters for the sanity metric
that compares measured quad side to the expected black-border size.

Prints PASS/FAIL per assertion and a final RESULT: PASS|FAIL line.
Exit code 0 on RESULT: PASS, 1 otherwise.
"""

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

try:
    from pupil_apriltags import Detector
except ImportError:  # pragma: no cover - exercised only without the dep
    Detector = None

_WORKTREE_UTILS = Path(__file__).resolve().parents[1]
if str(_WORKTREE_UTILS) not in sys.path:
    sys.path.insert(0, str(_WORKTREE_UTILS))
from apriltag_grid_gen.tag36h11 import BIT_X, BIT_Y, TAG36H11_CODES  # noqa: E402

DEFAULT_JSON = (
    Path(__file__).resolve().parents[2]
    / "generated"
    / "grid_5x4_tag36h11_ids18-37.json"
)

# Detection quad corners exclude the 1-cell quiet zone (verified
# empirically): black border is 8 of the 10 printed cells.
QUIET_ZONE_FRACTION = 0.8


def render_tag(tag_id, cell_px):
    """Render one tag as an L-mode PIL image of (10*cell_px) square pixels.

    Mirrors utils/apriltag_grid_gen/gen_grids.py:render_tag: outer 1-cell
    white quiet zone, 1-cell black border, 6x6 data area. Data bit i of
    the codeword (bit 0 = MSB) maps to full-tag cell (BIT_X[i]+1,
    BIT_Y[i]+1); bit 1 = white, bit 0 = black.
    """
    code = TAG36H11_CODES[tag_id]
    size = 10 * cell_px
    im = Image.new("L", (size, size), 255)
    px = im.load()
    draw = ImageDraw.Draw(im)
    draw.rectangle([cell_px, cell_px, 9 * cell_px - 1, 2 * cell_px - 1], fill=0)
    draw.rectangle([cell_px, 8 * cell_px, 9 * cell_px - 1, 9 * cell_px - 1], fill=0)
    draw.rectangle([cell_px, cell_px, 2 * cell_px - 1, 9 * cell_px - 1], fill=0)
    draw.rectangle([8 * cell_px, cell_px, 9 * cell_px - 1, 9 * cell_px - 1], fill=0)
    for i in range(36):
        bit = (code >> (35 - i)) & 1
        v = 255 if bit else 0
        x0 = (BIT_X[i] + 1) * cell_px
        y0 = (BIT_Y[i] + 1) * cell_px
        for dy in range(cell_px):
            for dx in range(cell_px):
                px[x0 + dx, y0 + dy] = v
    return im


def load_grid(json_path):
    """Return (expected_id_set, {id: (x_in, y_in)}, tag_side_in)."""
    data = json.loads(Path(json_path).read_text())
    if not data["meta"].get("size_in_includes_quiet_zone", False):
        raise ValueError(
            f"{json_path}: meta.size_in_includes_quiet_zone must be true"
        )
    expected = {t["id"] for t in data["tags"]}
    centers = {t["id"]: (t["center"]["x_in"], t["center"]["y_in"])
               for t in data["tags"]}
    side = data["tags"][0]["size_in"]
    return expected, centers, side


def load_gray(image_path):
    """Load an image as a uint8 grayscale numpy array via PIL."""
    img = Image.open(image_path)
    img = ImageOps.exif_transpose(img)
    return np.asarray(img.convert("L"), dtype=np.uint8)


def detect(gray):
    """Run the pupil-apriltags detector on a uint8 grayscale array."""
    if Detector is None:
        raise RuntimeError(
            "pupil-apriltags is not installed; run with "
            "`uv run --with pupil-apriltags --with pillow`"
        )
    # quad_decimate=1.0 for maximum sensitivity on ~70 px tags at 1280x720;
    # cost is negligible at this resolution.
    detector = Detector(families="tag36h11", quad_decimate=1.0)
    return detector.detect(gray)


def fit_homography(src, dst):
    """Least-squares homography mapping src -> dst (N >= 4), DLT via SVD."""
    n = len(src)
    A = np.zeros((2 * n, 9))
    for i, ((x, y), (u, v)) in enumerate(zip(src, dst)):
        A[2 * i] = [x, y, 1, 0, 0, 0, -u * x, -u * y, -u]
        A[2 * i + 1] = [0, 0, 0, x, y, 1, -v * x, -v * y, -v]
    _, _, vt = np.linalg.svd(A)
    h = vt[-1].reshape(3, 3)
    return h / h[2, 2]


def apply(h, pts):
    """Apply homography to (N, 2) points, return (N, 2)."""
    pts = np.atleast_2d(np.asarray(pts, dtype=float))
    t = h @ np.column_stack([pts, np.ones(len(pts))]).T
    return (t[:2] / t[2]).T


def px_per_inch(h, at):
    """Pixels per inch implied by h near image point `at` (x and y axes)."""
    p0 = apply(h, np.atleast_2d(at))[0]
    p1 = apply(h, np.atleast_2d(np.asarray(at) + [1.0, 0.0]))[0]
    p2 = apply(h, np.atleast_2d(np.asarray(at) + [0.0, 1.0]))[0]
    return 1.0 / np.linalg.norm(p1 - p0), 1.0 / np.linalg.norm(p2 - p0)


def quad_sides(detection):
    """Side lengths of a detection quad (px)."""
    c = np.asarray(detection.corners)
    return [np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]


def annotate(rgb, detections, matched_ids, centers_in, h_inv, out_path):
    """Draw tag boxes, IDs, centers, and the expected grid projection."""
    draw = ImageDraw.Draw(rgb)
    try:
        font = ImageFont.load_default(size=18)
    except TypeError:  # older Pillow: no size argument
        font = ImageFont.load_default()
    for d in detections:
        c = np.round(d.corners).astype(int)
        color = (0, 180, 0) if d.tag_id in matched_ids else (255, 140, 0)
        draw.polygon([tuple(p) for p in c], outline=color, width=2)
        cx, cy = float(d.center[0]), float(d.center[1])
        draw.ellipse([cx - 3, cy - 3, cx + 3, cy + 3],
                     fill=(220, 0, 0), outline=(220, 0, 0))
        draw.text((cx + 8, cy - 22), str(d.tag_id), fill=color, font=font)
    if h_inv is not None:
        for tid, (x, y) in centers_in.items():
            px, py = apply(h_inv, [(x, y)])[0]
            draw.line([px - 6, py, px + 6, py], fill=(0, 0, 220), width=1)
            draw.line([px, py - 6, px, py + 6], fill=(0, 0, 220), width=1)
    rgb.save(out_path)


def run_check(gray, json_path, out_path, label=""):
    """Full check pipeline on a grayscale array. Returns True iff PASS."""
    expected, centers_in, tag_side_in = load_grid(json_path)
    print(f"grid json: {json_path} — {len(expected)} expected tags, "
          f"tag side {tag_side_in:g} in (incl. quiet zone)")
    if label:
        print(f"image: {label} ({gray.shape[1]}x{gray.shape[0]})")

    detections = detect(gray)
    det_by_id = {d.tag_id: d for d in detections}
    found = set(det_by_id)
    missing = sorted(expected - found)
    unexpected = sorted(found - expected)
    matched = sorted(expected & found)

    results = []

    ok = len(detections) == len(expected) and not missing and not unexpected
    results.append(("count", ok,
                    f"{len(detections)} detections vs {len(expected)} expected"))
    ok = not missing
    results.append(("missing", ok, f"missing {missing if missing else 'none'}"))
    ok = not unexpected
    results.append(("unexpected", ok,
                    f"false positives {unexpected if unexpected else 'none'}"))

    ok_h = len(matched) >= 4
    results.append(("homography fit", ok_h,
                    f"{len(matched)} matched tag centers"
                    if ok_h else "fewer than 4 matched tags"))

    h = None
    if ok_h:
        src = np.array([det_by_id[i].center for i in matched])
        dst = np.array([centers_in[i] for i in matched])
        h = fit_homography(src, dst)
        err_in = np.linalg.norm(apply(h, src) - dst, axis=1)
        px_per_in_x, px_per_in_y = px_per_inch(h, src.mean(axis=0))
        px_per_in = (px_per_in_x + px_per_in_y) / 2.0
        err_px = err_in * px_per_in
        worst_i = int(np.argmax(err_in))
        worst_in = err_in[worst_i]
        worst_px = err_px[worst_i]
        thr_in = 0.02 * tag_side_in
        thr_px = 0.02 * tag_side_in * px_per_in
        ok_layout = (worst_px < thr_px) or (worst_in < 0.1)
        crit = "2% tag side" if worst_px < thr_px else "0.1 in"
        results.append(("layout agreement", ok_layout,
                        f"worst reprojection {worst_in:.4f} in "
                        f"({worst_px:.2f} px) on id={matched[worst_i]}; "
                        f"tolerance {thr_px:.2f} px (< 2% of tag side) "
                        f"or < 0.1 in — met via {crit}"))
        sides = [quad_sides(det_by_id[i]) for i in matched]
        med_side = float(np.median(sides))
        exp_side = QUIET_ZONE_FRACTION * tag_side_in * px_per_in
        margins = [det_by_id[i].decision_margin for i in matched]
        print(f"reprojection: worst {worst_in:.4f} in ({worst_px:.2f} px), "
              f"mean {err_in.mean():.4f} in, median "
              f"{float(np.median(err_in)):.4f} in")
        print(f"scale: {px_per_in:.1f} px/in (x {px_per_in_x:.1f}, "
              f"y {px_per_in_y:.1f})")
        print(f"quad side: median {med_side:.1f} px vs expected black border "
              f"{exp_side:.1f} px (ratio {med_side / exp_side:.3f})")
        print(f"decision margin: min {min(margins):.1f} "
              f"(median {float(np.median(margins)):.1f})")
    else:
        results.append(("layout agreement", False,
                        "skipped: homography fit failed"))

    passed = True
    for name, ok, detail in results:
        print(f"CHECK {name}: {'PASS' if ok else 'FAIL'} — {detail}")
        passed = passed and ok

    rgb = Image.fromarray(gray).convert("RGB")
    h_inv = np.linalg.inv(h) if h is not None else None
    annotate(rgb, detections, set(matched), centers_in, h_inv, out_path)
    print(f"annotated image: {out_path}")
    print("RESULT: " + ("PASS" if passed else "FAIL"))
    return passed


def selftest():
    """Synthetic round trip: render one tag, detect it, assert the ID."""
    print("== synthetic self-test: render tag36h11 id=18, detect, assert ==")
    cell = 25  # tag = 250 px, black border = 200 px
    canvas = Image.new("L", (500, 500), 255)
    canvas.paste(render_tag(18, cell), (125, 125))
    gray = np.asarray(canvas, dtype=np.uint8)
    detections = detect(gray)
    ok_count = len(detections) == 1
    print(f"CHECK detect exactly one tag: "
          f"{'PASS' if ok_count else 'FAIL'} — {len(detections)} found")
    ok_id = ok_count and detections[0].tag_id == 18
    got = detections[0].tag_id if detections else None
    print(f"CHECK detected id == 18: {'PASS' if ok_id else 'FAIL'} — got {got}")
    ok_side = False
    if detections:
        med = float(np.median(quad_sides(detections[0])))
        # quad must equal the 8-cell black border (200 px), confirming that
        # detection corners exclude the 1-cell quiet zone
        ok_side = abs(med - 200.0) < 2.0
        print(f"CHECK quad side == black border (200 px): "
              f"{'PASS' if ok_side else 'FAIL'} — {med:.1f} px")
    else:
        print("CHECK quad side == black border (200 px): FAIL — no detection")
    passed = ok_count and ok_id and ok_side
    print("RESULT: " + ("PASS" if passed else "FAIL"))
    return passed


def synth_grid(json_path, warp):
    """Render the JSON grid as a small image (optionally warped), then run
    the full check pipeline against the same JSON."""
    expected, centers_in, tag_side_in = load_grid(json_path)
    cell = 12
    tag_px = 10 * cell
    px_per_in = tag_px / tag_side_in
    margin = 30
    max_x = max(x for x, _ in centers_in.values()) + tag_side_in / 2
    max_y = max(y for _, y in centers_in.values()) + tag_side_in / 2
    w = margin + int(max_x * px_per_in) + 1
    h = margin + int(max_y * px_per_in) + 1
    canvas = Image.new("L", (w, h), 255)
    for tid in sorted(expected):
        x, y = centers_in[tid]
        cx = margin + int(round(x * px_per_in))
        cy = margin + int(round(y * px_per_in))
        canvas.paste(render_tag(tid, cell), (cx - tag_px // 2, cy - tag_px // 2))
    if warp:
        # mild perspective: pull the corners inward unevenly
        src_quad = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=float)
        dst_quad = np.array([[28, 16], [w - 22, 12],
                             [w - 10, h - 20], [12, h - 8]], dtype=float)
        hmap = fit_homography(dst_quad, src_quad)  # dst px -> src px
        yy, xx = np.mgrid[0:h, 0:w]
        pts = np.column_stack([xx.ravel(), yy.ravel()])
        s = apply(hmap, pts)
        sx = np.clip(np.round(s[:, 0]).astype(int), 0, w - 1)
        sy = np.clip(np.round(s[:, 1]).astype(int), 0, h - 1)
        src_arr = np.asarray(canvas, dtype=np.uint8)
        gray = src_arr[sy, sx].reshape(h, w)
        label = f"synthetic 5x4 grid, perspective-warped"
        out = Path(tempfile.gettempdir()) / "syntest_warp_detected.png"
    else:
        gray = np.asarray(canvas, dtype=np.uint8)
        label = "synthetic 5x4 grid, flat"
        out = Path(tempfile.gettempdir()) / "syntest_flat_detected.png"
    print(f"== synthetic grid test: {label} ({w}x{h}) ==")
    return run_check(gray, json_path, out, label)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Validate tag36h11 grid detection (T002).")
    ap.add_argument("image", nargs="?", help="input image path")
    ap.add_argument("--json", default=str(DEFAULT_JSON),
                    help="grid JSON path (default: generated 5x4 grid)")
    ap.add_argument("--out", default=None,
                    help="annotated PNG output (default: <image>_detected.png)")
    ap.add_argument("--selftest", action="store_true",
                    help="run the synthetic single-tag round trip and exit")
    ap.add_argument("--syntest", action="store_true",
                    help="render a synthetic grid and run the full check")
    ap.add_argument("--warp", action="store_true",
                    help="with --syntest: apply a perspective warp")
    args = ap.parse_args(argv)

    if args.selftest:
        return 0 if selftest() else 1
    if args.syntest:
        return 0 if synth_grid(args.json, args.warp) else 1
    if not args.image:
        ap.error("an image path is required (or use --selftest / --syntest)")
    out = Path(args.out) if args.out else (
        Path(args.image).with_name(Path(args.image).stem + "_detected.png"))
    gray = load_gray(args.image)
    label = f"{args.image} ({gray.shape[1]}x{gray.shape[0]})"
    return 0 if run_check(gray, args.json, out, label) else 1


if __name__ == "__main__":
    sys.exit(main())
