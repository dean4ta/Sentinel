#!/usr/bin/env python3
"""T003 — collect calibration photos on the Sentinel 1 Orin Nano.

Two windows: "detections" (live view with tag36h11 overlays) and
"capture" (frame preview with the save counter). SPACE/ENTER saves the
current frame to --outdir as img_%04d.jpg; Q/ESC quits early; capture
stops automatically after --n photos.

Pure python3 + cv2 + numpy + dt-apriltags. No ROS.

C270 dark-start ramp (handoff T002): the first ~20 frames after open are
pure black while auto-exposure ramps (~0.7 s). This script reads up to
60 frames, discarding frames with mean < 5 until a bright frame arrives;
saving a dark frame is refused the same way.

Exit codes:
  0  --n photos captured, or the user quit early with Q/ESC
  2  camera or display problem (open failed, all-black warm-up,
     no DISPLAY / GUI unavailable)
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

try:
    from dt_apriltags import Detector
except ImportError:  # pragma: no cover — the Orin has it
    Detector = None

DEFAULT_OUTDIR = "/home/sentinel1/dean/camera_calibration/images"
DEFAULT_CAM = "/dev/sentinel_webcam"
DARK_THRESHOLD = 5.0


def warm_up(cap, max_reads=60, dark_thr=DARK_THRESHOLD):
    """Read frames until a bright one arrives; return it, or None."""
    for _ in range(max_reads):
        ok, frame = cap.read()
        if not ok:
            continue
        if float(frame.mean()) >= dark_thr:
            return frame
    return None


def draw_overlay(bgr, detections):
    """Return a copy of bgr with per-tag quad, ID text, and center dot."""
    view = bgr.copy()
    for d in detections:
        corners = np.asarray(d.corners, dtype=float).reshape(-1, 2)
        pts = np.round(corners).astype(int).reshape(-1, 1, 2)
        cv2.polylines(view, [pts], True, (0, 200, 0), 2)
        cx, cy = np.round(np.asarray(d.center, dtype=float).ravel()[:2])
        cx, cy = int(cx), int(cy)
        cv2.circle(view, (cx, cy), 3, (0, 0, 255), -1)
        cv2.putText(view, str(int(d.tag_id)), (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
    return view


def should_detect(frame_idx, detect_every):
    """True when the loop runs detection for this frame index.

    detect_every=1 fires on every frame; detect_every=3 fires on
    frame 3, 6, 9, ... (~1/3 of frames)."""
    return frame_idx % detect_every == 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Collect apriltag-grid calibration photos (T003).")
    ap.add_argument("--n", type=int, default=20,
                    help="photos to collect (default 20)")
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR,
                    help="output dir (default %(default)s)")
    ap.add_argument("--cam", default=DEFAULT_CAM,
                    help="camera device (default %(default)s)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args(argv)

    if args.n < 1:
        ap.error("--n must be >= 1")

    if not os.environ.get("DISPLAY"):
        print("ERROR: no DISPLAY — the two capture windows need a "
              "graphical session on the Orin (run at the robot's "
              "desktop, or ssh with X forwarding, not a plain ssh "
              "session).", file=sys.stderr)
        return 2

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera {args.cam}", file=sys.stderr)
        return 2
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("warming up camera (C270 auto-exposure ramp) ...")
    frame = warm_up(cap)
    if frame is None:
        print("ERROR: no bright frame after 60 reads — camera problem "
              f"({args.cam}); check the cable and the udev symlink.",
              file=sys.stderr)
        cap.release()
        return 2
    print("live")
    height, width = frame.shape[:2]
    print(f"capture {width}x{height}, {args.n} photos -> {outdir}")

    if Detector is None:
        print("ERROR: dt-apriltags is not installed on the Orin "
              "(run: pip3 install -r requirements.txt)", file=sys.stderr)
        cap.release()
        return 2
    detector = Detector(families="tag36h11", quad_decimate=1.0)

    try:
        cv2.namedWindow("detections", cv2.WINDOW_NORMAL)
        cv2.namedWindow("capture", cv2.WINDOW_NORMAL)
    except cv2.error as exc:
        print(f"ERROR: cannot create GUI windows ({exc}) — the two "
              "windows need a display session on the Orin.",
              file=sys.stderr)
        cap.release()
        return 2

    saved = 0
    detect_every = 1
    detections = []
    overlay = frame.copy()
    frame_idx = 0
    fps_window_start = time.monotonic()
    fps_frames = 0

    while saved < args.n:
        ok, frame = cap.read()
        if not ok:
            continue
        frame_idx += 1

        # Run detection at most once per `detect_every` frames; the
        # interval grows when fps drops below 8 and shrinks above 15.
        if should_detect(frame_idx, detect_every):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray, estimate_tag_pose=False)
        overlay = draw_overlay(frame, detections)

        control = frame.copy()
        text = f"{saved}/{args.n} saved (SPACE save, Q quit)"
        cv2.putText(control, text, (12, height - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2)

        cv2.imshow("detections", overlay)
        cv2.imshow("capture", control)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q"), 27):
            print(f"quit: saved {saved}/{args.n}")
            break
        if key in (32, 13):  # SPACE or ENTER
            if float(frame.mean()) < DARK_THRESHOLD:
                print("skip: frame too dark (auto-exposure glitch), "
                      "not saved")
            else:
                path = outdir / f"img_{saved + 1:04d}.jpg"
                cv2.imwrite(str(path), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved += 1
                print(f"saved {saved}/{args.n}: {path}")

        fps_frames += 1
        now = time.monotonic()
        if now - fps_window_start >= 3.0:
            fps = fps_frames / (now - fps_window_start)
            print(f"fps {fps:.1f} (detect every {detect_every} frame(s))")
            if fps < 8.0 and detect_every < 8:
                detect_every += 1
            elif fps > 15.0 and detect_every > 1:
                detect_every -= 1
            fps_window_start = now
            fps_frames = 0

    cap.release()
    cv2.destroyAllWindows()
    if saved == args.n:
        print(f"done: {saved}/{args.n} photos in {outdir}")
    else:
        print(f"quit early: {saved}/{args.n} photos in {outdir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
