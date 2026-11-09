#!/usr/bin/env python3
"""Runnable check for the T001 tag36h11 calibration grid PDFs and JSON sidecars.

Run from the repo root (Mac, no ROS needed):

    uv run --with pillow python3 utils/apriltag_grid_gen/check_grids.py

Verifies:
  1. Each generated PDF exists, is valid PDF, single US Letter page
     (MediaBox 612 x 792 pt = 8.5 x 11 in).
  2. Portrait orientation: rows (larger count) along the 11 in dimension,
     cols along 8.5 in. Tag count per grid (3x2 -> 6, 4x3 -> 12,
     5x4 -> 20), IDs valid (0..586) and unique per grid and across grids.
  3. Corner squares: count (2 / 6 / 12), each centered on an interior
     grid intersection (midpoint of 4 adjacent tag centers, recomputed
     from generation parameters), fully inside the 0.5 in margin, no
     shared black pixels with any tag ink, and corner-touching: each
     square's 4 corners sit within 3 px (0.01 in at 300 dpi) of the
     black-border corners of the 4 surrounding tags (one distinct
     diagonal tag per square corner, no shared pixels).
  4. No text labels: the only ink on each page is tags and corner
     squares (every black pixel lies inside a tag or square rect).
  5. JSON sidecars: exist next to each PDF, parse, and agree with the
     actual layout — meta fields, tag ids/sizes/centers and square
     centers/sizes recomputed from generation parameters match the JSON,
     and JSON inch coordinates match pixel placements within 0.5 px.
  6. Regeneration is byte-reproducible (PDFs and JSONs regenerated into
     a temp dir match the committed files).
  7. Tag bitmap structure: 10x10 cells, white quiet ring, black border
     ring, and the 4 rotations of each checked tag's data pattern are
     distinct (family orientation property).
  8. Fit math: tag side length in px at 2 ft and 2.5 ft for a 1280-px
     (720p) webcam, assuming 60 deg horizontal FOV; asserts every grid's
     tag side >= 20 px at 2.5 ft.

Exits non-zero on any failure.
"""

import json
import math
import os
import re
import sys
import tempfile

from PIL import Image, ImageChops, ImageDraw

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


def grid_fnames(grid):
    rows, cols = grid["rows"], grid["cols"]
    ids = list(range(grid["id_start"], grid["id_start"] + rows * cols))
    base = f"{grid['name']}_tag36h11_ids{ids[0]}-{ids[-1]}"
    return base + ".pdf", base + ".json"


print("== T001 apriltag grid check ==")

for grid in gen_grids.GRIDS:
    rows, cols = grid["rows"], grid["cols"]
    ids = list(range(grid["id_start"], grid["id_start"] + rows * cols))
    fname, jname = grid_fnames(grid)
    pdf_path = os.path.join(GENERATED, fname)
    json_path = os.path.join(GENERATED, jname)

    # --- 1. PDF page geometry --------------------------------------------
    ok = os.path.isfile(pdf_path)
    check(ok, f"{fname} exists in generated/")
    if ok:
        data = open(pdf_path, "rb").read()
        check(data.startswith(b"%PDF"), f"{fname} has PDF magic")
        mbox = re.search(
            rb"/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)", data)
        check(mbox is not None, f"{fname} has MediaBox")
        if mbox:
            w, h = float(mbox.group(3)), float(mbox.group(4))
            check(abs(w - 612) < 0.5 and abs(h - 792) < 0.5,
                  f"{fname} page is US Letter 612x792 pt (got {w:.1f}x{h:.1f})")
        pages = len(re.findall(rb"/Type\s*/Page[^s]", data))
        check(pages == 1, f"{fname} has exactly 1 page (got {pages})")

    # --- 2. Portrait orientation, tag count, IDs -------------------------
    check(rows > cols,
          f"{grid['name']}: portrait - rows {rows} (along 11 in) > cols {cols} "
          "(along 8.5 in)")
    check(len(ids) == rows * cols, f"{grid['name']}: {rows}x{cols} = {len(ids)} tags")
    check(all(0 <= i < NCODES for i in ids),
          f"{grid['name']}: all IDs in valid tag36h11 range 0..{NCODES - 1}")
    check(len(set(ids)) == len(ids), f"{grid['name']}: no repeated IDs within grid")

    # --- 3. Corner squares -----------------------------------------------
    pl = gen_grids.placements(grid)  # recomputed from generation parameters
    expected_squares = (rows - 1) * (cols - 1)
    check(len(pl["squares"]) == expected_squares,
          f"{grid['name']}: {expected_squares} corner squares "
          f"(got {len(pl['squares'])})")
    # recompute interior intersections independently (square-grid layout)
    usable_w = gen_grids.PAGE_W_PX - 2 * gen_grids.MARGIN_PX
    usable_h = gen_grids.PAGE_H_PX - 2 * gen_grids.MARGIN_PX
    s = min(usable_w // cols, usable_h // rows)
    s = (s // 10) * 10
    cell = s // 10
    tag_px = 10 * cell
    q = s - 8 * cell  # square side = gap between adjacent tags' black borders
    block_w = (cols - 1) * s + tag_px
    block_h = (rows - 1) * s + tag_px
    x0 = gen_grids.MARGIN_PX + (usable_w - block_w) // 2
    y0 = gen_grids.MARGIN_PX + (usable_h - block_h) // 2
    x_centers = [x0 + tag_px // 2 + i * s for i in range(cols)]
    y_centers = [y0 + tag_px // 2 + j * s for j in range(rows)]
    tags_match = all(
        pl["tags"][j * cols + i]["cx"] == x_centers[i]
        and pl["tags"][j * cols + i]["cy"] == y_centers[j]
        and pl["tags"][j * cols + i]["px"] == tag_px
        for j in range(rows) for i in range(cols))
    check(tags_match,
          f"{grid['name']}: tag centers/sizes match recomputed square grid "
          f"(spacing {s} px, tag side {tag_px} px)")
    want_sq = {((x_centers[i] + x_centers[i + 1]) // 2,
                (y_centers[j] + y_centers[j + 1]) // 2)
               for j in range(rows - 1) for i in range(cols - 1)}
    got_sq = {(s["cx"], s["cy"]) for s in pl["squares"]}
    check(got_sq == want_sq,
          f"{grid['name']}: squares centered on interior grid intersections")
    check(all(sq["px"] == q for sq in pl["squares"]),
          f"{grid['name']}: square side = black-border gap ({q} px "
          f"= {q / gen_grids.DPI:.4f} in)")
    sq_in_margin = all(
        s["cx"] - s["px"] // 2 >= gen_grids.MARGIN_PX
        and s["cx"] - s["px"] // 2 + s["px"] - 1
        < gen_grids.PAGE_W_PX - gen_grids.MARGIN_PX
        and s["cy"] - s["px"] // 2 >= gen_grids.MARGIN_PX
        and s["cy"] - s["px"] // 2 + s["px"] - 1
        < gen_grids.PAGE_H_PX - gen_grids.MARGIN_PX
        for s in pl["squares"])
    check(sq_in_margin, f"{grid['name']}: all squares inside 0.5 in margin")

    # --- corner touch: every square corner sits within 3 px (0.01 in at
    # 300 dpi) of a black-border corner of one of the 4 diagonal tags,
    # with no shared pixels (touching, not overlapping)
    def tag_black_corners(t):
        c = t["px"] // 10
        tx = t["cx"] - t["px"] // 2
        ty = t["cy"] - t["px"] // 2
        return [(tx + c, ty + c), (tx + 9 * c - 1, ty + c),
                (tx + c, ty + 9 * c - 1), (tx + 9 * c - 1, ty + 9 * c - 1)]

    corners = [(t["id"], b) for t in pl["tags"] for b in tag_black_corners(t)]
    touch_ok = True
    for sq in pl["squares"]:
        h = sq["px"] // 2
        sq_corners = [(sq["cx"] - h, sq["cy"] - h),
                      (sq["cx"] + h - 1, sq["cy"] - h),
                      (sq["cx"] - h, sq["cy"] + h - 1),
                      (sq["cx"] + h - 1, sq["cy"] + h - 1)]
        nearest = []
        for (x, y) in sq_corners:
            tid, (bx, by), d = min(
                ((tid, (bx, by), math.hypot(x - bx, y - by))
                 for tid, (bx, by) in corners),
                key=lambda p: p[2])
            nearest.append(tid)
            if not 1 <= d <= 3:
                touch_ok = False
        if len(set(nearest)) != 4:  # one distinct diagonal tag per corner
            touch_ok = False
    check(touch_ok,
          f"{grid['name']}: each square corner touches a diagonal tag "
          "black-border corner within 3 px (0.01 in)")

    # --- 4. No text labels ----------------------------------------------
    page = gen_grids.build_page(grid)
    ink = page.point(lambda v: 0 if v == 255 else 255)
    covered = Image.new("L", page.size, 0)
    draw = ImageDraw.Draw(covered)
    for t in pl["tags"]:
        draw.rectangle([t["cx"] - t["px"] // 2, t["cy"] - t["px"] // 2,
                        t["cx"] - t["px"] // 2 + t["px"] - 1,
                        t["cy"] - t["px"] // 2 + t["px"] - 1], fill=255)
    for s in pl["squares"]:
        draw.rectangle([s["cx"] - s["px"] // 2, s["cy"] - s["px"] // 2,
                        s["cx"] - s["px"] // 2 + s["px"] - 1,
                        s["cy"] - s["px"] // 2 + s["px"] - 1], fill=255)
    outside = ImageChops.multiply(ink, ImageChops.invert(covered))
    check(outside.getextrema()[1] == 0,
          f"{grid['name']}: no ink outside tag and corner-square rects "
          "(no text labels)")
    # no shared black pixels between square ink and tag ink: a square
    # corner that touches a tag's black-border corner diagonally extends
    # into the tag's white quiet zone (unavoidable for the touch), but
    # must never cover any black tag pixel
    tag_ink = Image.new("L", page.size, 255)
    for t in pl["tags"]:
        tag_ink.paste(gen_grids.render_tag(t["id"], t["px"] // 10),
                      (t["cx"] - t["px"] // 2, t["cy"] - t["px"] // 2))
    tag_ink = tag_ink.point(lambda v: 0 if v == 255 else 255)
    sq_ink = Image.new("L", page.size, 0)
    sq_draw = ImageDraw.Draw(sq_ink)
    for s in pl["squares"]:
        sq_draw.rectangle([s["cx"] - s["px"] // 2, s["cy"] - s["px"] // 2,
                           s["cx"] - s["px"] // 2 + s["px"] - 1,
                           s["cy"] - s["px"] // 2 + s["px"] - 1], fill=255)
    check(ImageChops.multiply(tag_ink, sq_ink).getextrema()[1] == 0,
          f"{grid['name']}: no shared black pixels between square and tag ink")

    # --- 5. JSON sidecar agrees with layout ------------------------------
    ok = os.path.isfile(json_path)
    check(ok, f"{jname} exists in generated/")
    if ok:
        with open(json_path) as f:
            try:
                jd = json.load(f)
                parsed = True
            except ValueError:
                parsed = False
        check(parsed, f"{jname} parses as JSON")
        if parsed:
            meta = jd.get("meta", {})
            check(meta.get("family") == "tag36h11", f"{jname}: meta.family")
            check(meta.get("rows") == rows and meta.get("cols") == cols,
                  f"{jname}: meta rows/cols = {rows}/{cols}")
            check(meta.get("page_size_in") == [8.5, 11.0],
                  f"{jname}: meta.page_size_in = [8.5, 11.0]")
            check(meta.get("margin_in") == 0.5, f"{jname}: meta.margin_in = 0.5")
            check(meta.get("units") == "inches", f"{jname}: meta.units")
            check(meta.get("origin") == "top-left corner of tag 0",
                  f"{jname}: meta.origin")
            check(meta.get("x_axis") == "right" and meta.get("y_axis") == "down",
                  f"{jname}: meta axes")
            tags = jd.get("tags", [])
            check(len(tags) == rows * cols, f"{jname}: tags list has {rows*cols} entries")
            check([t["id"] for t in tags] == ids,
                  f"{jname}: tag ids match {ids[0]}-{ids[-1]}")
            check(all(t["size_in"] == pl["tag_in"] for t in tags),
                  f"{jname}: tag size_in = {pl['tag_in']:.4f}")
            centers_match = all(
                t["center"] == ref["center"]
                for t, ref in zip(tags, pl["tags_in"]))
            check(centers_match,
                  f"{jname}: tag centers match recomputed layout")
            squares = jd.get("squares", [])
            check(len(squares) == expected_squares,
                  f"{jname}: squares list has {expected_squares} entries")
            sq_match = all(
                s == ref for s, ref in zip(squares, pl["squares_in"]))
            check(sq_match, f"{jname}: square centers/sizes match recomputed layout")
            # JSON inch coords consistent with actual pixel placements
            ox, oy = pl["origin_px"]
            px_ok = all(
                abs(t["center"]["x_in"] * gen_grids.DPI
                    - (pl["tags"][i]["cx"] - ox)) <= 0.5
                and abs(t["center"]["y_in"] * gen_grids.DPI
                    - (pl["tags"][i]["cy"] - oy)) <= 0.5
                for i, t in enumerate(tags))
            px_ok = px_ok and all(
                abs(s["center"]["x_in"] * gen_grids.DPI
                    - (pl["squares"][i]["cx"] - ox)) <= 0.5
                and abs(s["center"]["y_in"] * gen_grids.DPI
                    - (pl["squares"][i]["cy"] - oy)) <= 0.5
                for i, s in enumerate(squares))
            check(px_ok, f"{jname}: inch coords match pixel placements (<= 0.5 px)")

# --- 6. Reproducibility --------------------------------------------------
with tempfile.TemporaryDirectory() as tmp:
    gen_grids.main(["--out", tmp])
    for grid in gen_grids.GRIDS:
        fname, jname = grid_fnames(grid)
        for name in (fname, jname):
            a = open(os.path.join(GENERATED, name), "rb").read()
            b = open(os.path.join(tmp, name), "rb").read()
            check(a == b, f"{name} regenerates byte-identically")

# --- 7. Tag bitmap structure ---------------------------------------------
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

# --- 8. Fit math ---------------------------------------------------------
px_per_in = {
    d: CAM_W_PX / (2 * d * 12 * math.tan(math.radians(HFOV_DEG / 2)))
    for d in DIST_FT
}
print(f"\n  Fit math (720p webcam, {CAM_W_PX} px wide, assumed HFOV {HFOV_DEG} deg):")
print(f"  px/in: 2 ft -> {px_per_in[2.0]:.1f}, 2.5 ft -> {px_per_in[2.5]:.1f}")
print(f"  {'grid':<9} {'tag in':>7} {'sq in':>6} {'px @2ft':>8} {'px @2.5ft':>9}")
for grid in gen_grids.GRIDS:
    pl = gen_grids.placements(grid)
    side_in = pl["tag_in"]
    sq_in = pl["square_px"] / gen_grids.DPI
    p2 = side_in * px_per_in[2.0]
    p25 = side_in * px_per_in[2.5]
    print(f"  {grid['name']:<9} {side_in:>7.4f} {sq_in:>6.4f} {p2:>8.1f} {p25:>9.1f}")
    check(p25 >= MIN_TAG_PX,
          f"{grid['name']}: tag side {p25:.1f} px at 2.5 ft >= {MIN_TAG_PX} px")

# cross-grid ID uniqueness
all_ids = [i for grid in gen_grids.GRIDS
           for i in range(grid["id_start"],
                          grid["id_start"] + grid["rows"] * grid["cols"])]
check(len(set(all_ids)) == len(all_ids),
      f"no repeated IDs across all grids ({len(all_ids)} tags total)")

print()
if failures:
    print(f"RESULT: FAIL ({len(failures)} failure(s))")
    sys.exit(1)
print("RESULT: PASS (all checks)")
