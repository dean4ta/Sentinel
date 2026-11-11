#!/usr/bin/env python3
"""Validator-02 independent checks for T002 (check_detect.py).

Plain asserts, no framework. Covers the gaps in check_detect.py's own
self-tests: wrong-board loudness, CLI edge cases, and exit codes.

Run (Mac, repo root, any cwd):
  uv run --with pupil-apriltags --with pillow python3 \\
      utils/apriltag_detect/check_validator_t002.py

Writes only into /tmp/sentinel/validator/ (scratch). Does not modify
repo files.
"""

import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
CHECK = ROOT / "utils" / "apriltag_detect" / "check_detect.py"
SCRATCH = Path("/tmp/sentinel/validator")
SCRATCH.mkdir(parents=True, exist_ok=True)
WORK = Path(tempfile.mkdtemp(prefix="t002_check_", dir=SCRATCH))

sys.path.insert(0, str(CHECK.parent))
import check_detect  # noqa: E402  (imports pupil_apriltags if present)


def run_cli(*args):
    """Run check_detect.py as a subprocess; return (exit_code, stdout+stderr)."""
    proc = subprocess.run(
        [sys.executable, str(CHECK), *args],
        capture_output=True, text=True, timeout=300,
    )
    return proc.returncode, proc.stdout + proc.stderr


def main():
    failures = []

    def check(name, cond, detail):
        if cond:
            print(f"PASS {name}")
        else:
            failures.append(name)
            print(f"FAIL {name} — {detail}")

    # 1. Wrong board: a single tag36h11 id=5 (inside the 0-17 range the
    # task card flags as wrong for the 5x4 JSON whose expected set is
    # 18-37) must FAIL loudly and exit nonzero.
    canvas = Image.new("L", (500, 500), 255)
    canvas.paste(check_detect.render_tag(5, 25), (125, 125))
    wrong_png = WORK / "wrongboard_id5.png"
    canvas.save(wrong_png)
    code, out = run_cli(str(wrong_png))
    check("wrong-board exit nonzero", code != 0, f"exit={code} out={out!r}")
    check("wrong-board RESULT: FAIL", "RESULT: FAIL" in out, f"out={out!r}")
    check("wrong-board loud id report",
          "CHECK unexpected: FAIL" in out and "false positives [5]" in out,
          f"out={out!r}")
    # Pixel-level assert on the annotated PNG (not just existence): the
    # wrong-board tag is not in the expected set, so its box is drawn
    # orange (255,140,0) and its center dot red (220,0,0). Count exact
    # color pixels to prove boxes and centers were actually drawn.
    ann_png = WORK / "wrongboard_id5_detected.png"
    exists = ann_png.exists()
    orange = red = -1
    if exists:
        ann_arr = np.asarray(Image.open(ann_png).convert("RGB"),
                             dtype=np.uint8)
        orange = int(np.sum(np.all(ann_arr == [255, 140, 0], axis=2)))
        red = int(np.sum(np.all(ann_arr == [220, 0, 0], axis=2)))
    check("wrong-board annotated PNG written", exists, f"out={out!r}")
    check("wrong-board annotated box pixels (orange)",
          orange > 20, f"orange px={orange} out={out!r}")
    check("wrong-board annotated center pixels (red)",
          red > 0, f"red px={red} out={out!r}")

    # 2. Missing image path argument: argparse usage error, exit code 2.
    code, out = run_cli()
    check("no-args exit 2", code == 2, f"exit={code} out={out!r}")
    check("no-args clear message", "an image path is required" in out,
          f"out={out!r}")

    # 3. Nonexistent input file: nonzero exit, message names the path.
    missing = WORK / "does_not_exist.jpg"
    code, out = run_cli(str(missing))
    check("missing-file exit nonzero", code != 0, f"exit={code} out={out!r}")
    check("missing-file message names path", str(missing) in out,
          f"out={out!r}")

    # 4. Corrupt input file: nonzero exit, no silent zero-detection PASS.
    corrupt = WORK / "corrupt.png"
    corrupt.write_bytes(b"this is not an image file")
    code, out = run_cli(str(corrupt))
    check("corrupt-file exit nonzero", code != 0, f"exit={code} out={out!r}")
    check("corrupt-file identifies image error",
          "cannot identify image" in out, f"out={out!r}")

    # 5. Blank black image: RESULT FAIL, exit 1, zero detections, zero
    # false positives, no crash (no traceback).
    black_png = WORK / "black.png"
    Image.new("L", (1280, 720), 0).save(black_png)
    code, out = run_cli(str(black_png))
    check("black-image exit 1", code == 1, f"exit={code} out={out!r}")
    check("black-image RESULT: FAIL", "RESULT: FAIL" in out, f"out={out!r}")
    check("black-image 0 detections",
          "0 detections vs 20 expected" in out, f"out={out!r}")
    check("black-image no false positives",
          "false positives none" in out, f"out={out!r}")
    check("black-image no traceback", "Traceback" not in out, f"out={out!r}")

    # 6. Sanity: the script's own selftest round trip still exits 0.
    code, out = run_cli("--selftest")
    check("selftest exit 0", code == 0, f"exit={code} out={out!r}")
    check("selftest RESULT: PASS", "RESULT: PASS" in out, f"out={out!r}")

    if failures:
        print(f"\nRESULT: FAIL — {len(failures)} failed: {failures}")
        return 1
    print("\nRESULT: PASS — all validator checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
