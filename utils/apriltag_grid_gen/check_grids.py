#!/usr/bin/env python3
"""Runnable check for the T001 tag36h11 calibration grid PDFs.

Run from the repo root (Mac, no ROS needed):

    uv run --with pillow python3 utils/apriltag_grid_gen/check_grids.py

Verifies:
  1. Each generated PDF exists, is valid PDF, single US Letter page
     (MediaBox 612 x 792 pt = 8.5 x 11 in).
  2. Tag count per grid (3x2 -> 6, 4x3 -> 12, 5x4 -> 20), IDs valid
     (0..586) and unique per grid and across grids.
  3. Regeneration is byte-reproducible (regenerates into a temp dir and
     compares against the committed PDFs).
  4. Tag bitmap structure: 10x10 cells, white quiet ring, black border
     ring, and the 4 rotations of each checked tag's data pattern are
     distinct (family orientation property).
  5. Fit math: tag side length in px at 2 ft and 2.5 ft for a 1280-px
     (720p) webcam, assuming 60 deg horizontal FOV; asserts every grid's
     tag side >= 20 px at 2.5 ft.

Exits non-zero on any failure.
"""

import math
import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_grids
from tag36h11 import BIT_X, BIT_Y, NCODES, TAG36H11_CODES

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GENERATED = os.path.join(REPO_ROOT, "generated")

# Fit math assumptions (documented in the handoff and docs)
CAM_W_PX = 1280          # 720p webcam horizontal resolution
HFOV_DEG = 60.0          # assumed horizontal field of view
MIN_TAG_PX = 20          # minimum tag side in px at the far distance
DIST_FT = (2.0, 2.5)     # robot webcam working distances

failures = []


def check(cond, msg):
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {msg}")
    if not cond:
        failures.append(msg)


def rot90(grid):
    n = len(grid)
    return tuple(tuple(grid[n - 1 - c][r] for c in range(n)) for r in range(n))


def data_pattern(code):
    """6x6 data pattern of a codeword (bit 1 -> 1, bit 0 -> 0)."""
    g = [[0] * 6 for _ in range(6)]
    for i in range(36):
        g[BIT_Y[i] - 1][BIT_X[i] - 1] = (code >> (35 - i)) & 1
    return tuple(tuple(row) for row in g)


print("== T001 apriltag grid check ==")

# --- 1. PDF page geometry ------------------------------------------------
for grid in gen_grids.GRIDS:
    cols, rows = grid["cols"], grid["rows"]
    ids = list(range(grid["id_start"], grid["id_start"] + cols * rows))
    fname = f"{grid['name']}_tag36h11_ids{ids[0]}-{ids[-1]}.pdf"
    path = os.path.join(GENERATED, fname)
    ok = os.path.isfile(path)
    check(ok, f"{fname} exists in generated/")
    if not ok:
        continue
    data = open(path, "rb").read()
    check(data.startswith(b"%PDF"), f"{fname} has PDF magic")
    mbox = re.search(rb"/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)",
                     data)
    check(mbox is not None, f"{fname} has MediaBox")
    if mbox:
        w, h = float(mbox.group(3)), float(mbox.group(4))
        check(abs(w - 612) < 0.5 and abs(h - 792) < 0.5,
              f"{fname} page is US Letter 612x792 pt (got {w:.1f}x{h:.1f})")
    pages = len(re.findall(rb"/Type\s*/Page[^s]", data))
    check(pages == 1, f"{fname} has exactly 1 page (got {pages})")

# --- 2. Tag count / ID validity / uniqueness -----------------------------
all_ids = []
for grid in gen_grids.GRIDS:
    cols, rows = grid["cols"], grid["rows"]
    ids = list(range(grid["id_start"], grid["id_start"] + cols * rows))
    all_ids.extend(ids)
    check(len(ids) == cols * rows, f"{grid['name']}: {cols}x{rows} = {len(ids)} tags")
    check(all(0 <= i < NCODES for i in ids),
          f"{grid['name']}: all IDs in valid tag36h11 range 0..{NCODES - 1}")
    check(len(set(ids)) == len(ids), f"{grid['name']}: no repeated IDs within grid")
    # all ink inside the 0.5 in printer margin
    page = gen_grids.build_page(grid)
    outside = 0
    for y in range(page.height):
        for x in range(page.width):
            if (x < gen_grids.MARGIN_PX or x >= page.width - gen_grids.MARGIN_PX
                    or y < gen_grids.MARGIN_PX or y >= page.height - gen_grids.MARGIN_PX):
                if page.getpixel((x, y)) != 255:
                    outside += 1
    check(outside == 0,
          f"{grid['name']}: all content inside 0.5 in margin ({outside} px outside)")
check(len(set(all_ids)) == len(all_ids),
      f"no repeated IDs across all grids ({len(all_ids)} tags total)")

# --- 3. Reproducibility --------------------------------------------------
with tempfile.TemporaryDirectory() as tmp:
    gen_grids.main(["--out", tmp])
    same = True
    for grid in gen_grids.GRIDS:
        cols, rows = grid["cols"], grid["rows"]
        ids = list(range(grid["id_start"], grid["id_start"] + cols * rows))
        fname = f"{grid['name']}_tag36h11_ids{ids[0]}-{ids[-1]}.pdf"
        a = open(os.path.join(GENERATED, fname), "rb").read()
        b = open(os.path.join(tmp, fname), "rb").read()
        if a != b:
            same = False
            check(False, f"{fname} regenerates byte-identically")
        else:
            check(True, f"{fname} regenerates byte-identically")

# --- 4. Tag bitmap structure ---------------------------------------------
for tag_id in (0, 1, 18, 37):
    im = gen_grids.render_tag(tag_id, 4)  # 40x40 px
    check(im.size == (40, 40), f"tag {tag_id}: bitmap is 10x10 cells")
    px = im.load()
    ring_white = all(px[x, y] == 255 for x in range(40) for y in range(40)
                     if x < 4 or x >= 36 or y < 4 or y >= 36)
    ring_black = all(px[x, y] == 0 for x in range(4, 36) for y in range(4, 36)
                     if (4 <= x < 8 or 32 <= x < 36 or 4 <= y < 8 or 32 <= y < 36))
    check(ring_white, f"tag {tag_id}: outer white quiet zone present")
    check(ring_black, f"tag {tag_id}: black border ring present")
    pat = data_pattern(TAG36H11_CODES[tag_id])
    rots = {pat}
    cur = pat
    for _ in range(3):
        cur = rot90(cur)
        rots.add(cur)
    check(len(rots) == 4, f"tag {tag_id}: 4 rotations of data pattern distinct")

# --- 5. Fit math ---------------------------------------------------------
px_per_in = {
    d: CAM_W_PX / (2 * d * 12 * math.tan(math.radians(HFOV_DEG / 2)))
    for d in DIST_FT
}
print(f"\n  Fit math (720p webcam, {CAM_W_PX} px wide, assumed HFOV {HFOV_DEG} deg):")
print(f"  px/in: 2 ft -> {px_per_in[2.0]:.1f}, 2.5 ft -> {px_per_in[2.5]:.1f}")
print(f"  {'grid':<9} {'tag in':>7} {'px @2ft':>8} {'px @2.5ft':>9}")
for grid in gen_grids.GRIDS:
    side_in = grid["tag_in"]
    p2 = side_in * px_per_in[2.0]
    p25 = side_in * px_per_in[2.5]
    print(f"  {grid['name']:<9} {side_in:>7.2f} {p2:>8.1f} {p25:>9.1f}")
    check(p25 >= MIN_TAG_PX,
          f"{grid['name']}: tag side {p25:.1f} px at 2.5 ft >= {MIN_TAG_PX} px")

print()
if failures:
    print(f"RESULT: FAIL ({len(failures)} failure(s))")
    sys.exit(1)
print("RESULT: PASS (all checks)")
