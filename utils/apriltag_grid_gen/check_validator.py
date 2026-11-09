#!/usr/bin/env python3
"""Independent validator check for T001 tag36h11 calibration grid PDFs (validator-01).

Validates the REVISED portrait + corner-square + JSON-sidecar + title-label
spec. Does NOT import or reuse utils/apriltag_grid_gen/check_grids.py or
gen_grids.py internals; the committed tag36h11.py is imported as DATA ONLY
(codeword table + bit layout).

Run (Mac, pure python; pymupdf for rasterizing, Pillow for PNG/text):

    uv run --with pillow --with pymupdf python3 utils/apriltag_grid_gen/check_validator.py [--gt DIR]

Checks, from the committed PDF bytes and independently rasterized pixels:

 1. PDF structure (raw parse): magic, MediaBox [0 0 612 792] US Letter,
    exactly 1 page, no CropBox.
 2. Raster: pymupdf render at 300 dpi == 2550x3300 px.
 3. Margins: all ink (value < 248) inside the 0.5 in printer margin.
 4. Geometry from pixels (connected components, union-find over row runs):
    tag annuli (6/12/20), black corner squares (2/6/12), no other ink.
 5. Portrait orientation: distinct tag-center rows (3/4/5) run along the
    11 in axis, cols (2/3/4) along 8.5 in; square grid (x spacing == y
    spacing); vertical extent > horizontal extent.
 6. Squares: side == gap between adjacent tag black borders (2 cells);
    centered at the midpoint of the 4 surrounding tags; every square corner
    touches the diagonal tag's black-border corner (corner-to-corner
    ~1 px, within 0.01 in) with NO shared/4-adjacent black pixels.
 7. Title: the only text on the page. Ink bbox inside the 0.5 in top
    margin, horizontally centered, no overlap with tags/squares; rendered
    content matches the PDF filename (independent Pillow render, tolerant
    pixel compare).
 8. ID decode: sample each tag's 10x10 cells from pixels, verify quiet
    ring + black border ring, decode the 6x6 data region against the
    committed tag36h11 table; counts, ID validity (0..586), uniqueness
    within and across grids, documented per-grid ID sets.
 9. JSON sidecars: meta fields exact; recomputed tag/square centers
    (inches, origin = top-left corner of tag 0, x right, y down) and sizes
    match the JSON within 0.01 in; row-major ID order.
10. Ground truth (--gt): all 38 PDF tags plus ~40 randomly sampled other
    IDs rendered independently from the codeword table compared against the
    official AprilRobotics/apriltag-imgs tag36_11_*.png images.
11. Reproducibility: regenerate PDFs + JSONs via subprocess into a temp
    dir; byte-compare with the committed files.
12. Fit math: from the MEASURED tag side (pixels -> inches), recompute
    px at 2 ft / 2.5 ft for a 1280 px 720p webcam, HFOV 60 deg (stated
    assumption); require >= 20 px at 2.5 ft.
13. Hygiene: no third-party files committed.

Prints [PASS]/[FAIL] per check and a final RESULT line; exits 1 on failure.
"""

import argparse
import json
import math
import os
import random
import re
import subprocess
import sys
import tempfile

try:
    import pymupdf as fitz
except ImportError:  # older pymupdf API
    import fitz

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag36h11 import BIT_X, BIT_Y, NCODES, TAG36H11_CODES  # data only

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
GENERATED = os.path.join(REPO_ROOT, "generated")

# --- task spec (current card), hardcoded here for independence ---
GRIDS = [
    {"name": "grid_3x2", "rows": 3, "cols": 2, "tag_in": 3.3333, "id0": 0, "id1": 5},
    {"name": "grid_4x3", "rows": 4, "cols": 3, "tag_in": 2.5000, "id0": 6, "id1": 17},
    {"name": "grid_5x4", "rows": 5, "cols": 4, "tag_in": 1.8667, "id0": 18, "id1": 37},
]
# tag total side (incl. quiet zone) in px at 300 dpi, and derived cells
for g in GRIDS:
    g["s_px"] = round(g["tag_in"] * 300 / 10) * 10  # 1000 / 750 / 560
    g["cell_px"] = g["s_px"] // 10
    g["ann_px"] = 8 * g["cell_px"]   # black border = 8 cells
    g["sq_px"] = 2 * g["cell_px"]    # square side = gap between borders

PAGE_W_IN, PAGE_H_IN = 8.5, 11.0
MARGIN_IN = 0.5
PAGE_W_PX, PAGE_H_PX = 2550, 3300
MARGIN_PX = 150
INK_THRESH = 248     # decoded raster: values < 248 are real ink
BLACK_THRESH = 128   # black/white cell sampling
TITLE_FONT_PX = 64

CAM_W_PX = 1280
HFOV_DEG = 60.0
MIN_TAG_PX = 20
DIST_FT = (2.0, 2.5)
GT_SAMPLE_SEED = 42
GT_SAMPLE_N = 40

failures = []


def check(cond, msg):
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {msg}")
    if not cond:
        failures.append(msg)


# ---------------------------------------------------------------- raw parse
def parse_pdf(data):
    if not data.startswith(b"%PDF"):
        return None
    mbs = re.findall(rb'/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*\]', data)
    cbs = re.findall(rb'/CropBox\s*\[[^\]]*\]', data)
    pages = re.findall(rb'/Type\s*/Page(?![sA-Za-z])', data)
    counts = [int(c) for c in re.findall(rb'/Count\s+(\d+)', data)]
    return {"mediaboxes": mbs, "cropboxes": cbs, "n_pages": len(pages), "counts": counts}


# ------------------------------------------------------------- raster + CC
def rasterize(pdf):
    page = fitz.open(pdf)[0]
    pix = page.get_pixmap(matrix=fitz.Matrix(300 / 72, 300 / 72),
                          colorspace=fitz.csGRAY)
    return Image.frombytes("L", (pix.width, pix.height), pix.samples)


def row_runs(g, thresh):
    W, H = g.size
    data = g.tobytes()
    table = bytes([0 if i < thresh else 255 for i in range(256)])
    rows = {}
    for y in range(H):
        row = data[y * W:(y + 1) * W].translate(table)
        x = 0
        while True:
            a = row.find(b"\x00", x)
            if a < 0:
                break
            b = row.find(b"\xff", a)
            if b < 0:
                rows.setdefault(y, []).append((a, W - 1))
                break
            rows.setdefault(y, []).append((a, b - 1))
            x = b + 1
    return rows


def components(runs_by_row):
    parent = {}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    prev = {}
    prev_y = None
    for y in sorted(runs_by_row):
        if prev_y is not None and y > prev_y + 1:
            prev = {}
        prev_y = y
        cur = runs_by_row[y]
        for (x0, x1) in cur:
            lab = (y, x0, x1)
            parent[lab] = lab
            for pl, (px0, px1) in prev.items():
                if x0 <= px1 and px0 <= x1:
                    union(lab, pl)
        prev = {(y, x0, x1): (x0, x1) for (x0, x1) in cur}
    boxes = {}
    for lab in parent:
        r = find(lab)
        y, x0, x1 = lab
        if r not in boxes:
            boxes[r] = [x0, x1, y, y]
        else:
            b = boxes[r]
            if x0 < b[0]:
                b[0] = x0
            if x1 > b[1]:
                b[1] = x1
            if y < b[2]:
                b[2] = y
            if y > b[3]:
                b[3] = y
    return [tuple(b) for b in boxes.values()]


# ------------------------------------------------------------ tag sampling
def sample_tag_cells(g, bbox):
    """10x10 cell matrix sampled at cell centers of a tag annulus bbox.

    Returns (matrix, points) where points[r][c] is the sampled pixel coord.
    """
    x0, x1, y0, y1 = bbox
    w, h = x1 - x0 + 1, y1 - y0 + 1
    if abs(w - h) > max(2, 0.05 * w):
        return None, None
    cell = w / 8.0
    cx0, cy0 = x0 - cell, y0 - cell
    px = g.load()
    m = []
    pts = []
    for r in range(10):
        row = []
        prow = []
        for c in range(10):
            x = int(cx0 + (c + 0.5) * cell)
            y = int(cy0 + (r + 0.5) * cell)
            prow.append((x, y))
            v = px[x, y]
            row.append(1 if v >= BLACK_THRESH else 0)
        m.append(row)
        pts.append(prow)
    return m, pts


def covered_corners(g, pts):
    """Quiet-zone corner cells whose sample point falls on a corner square.

    By design the corner squares (side = 2 cells) extend into the tags'
    white quiet-zone corners, so those 4 sample points may be black.
    """
    covered = set()
    for (r, c) in ((0, 0), (0, 9), (9, 0), (9, 9)):
        x, y = pts[r][c]
        if any(s[0] <= x <= s[1] and s[2] <= y <= s[3] for s in g["squares"]):
            covered.add((r, c))
    return covered


def expected_matrix(code):
    e = [[0] * 6 for _ in range(6)]
    for i in range(36):
        e[BIT_Y[i] - 1][BIT_X[i] - 1] = (code >> (35 - i)) & 1
    return e


def decode_matrix(m):
    for tid in range(NCODES):
        if expected_matrix(TAG36H11_CODES[tid]) == m:
            return tid
    return None


def render10(code):
    g = [[1] * 10 for _ in range(10)]
    for r in range(1, 9):
        for c in range(1, 9):
            if r in (1, 8) or c in (1, 8):
                g[r][c] = 0
    for i in range(36):
        g[BIT_Y[i] + 1][BIT_X[i] + 1] = (code >> (35 - i)) & 1
    return g


def official_matrix(png_path):
    im = Image.open(png_path).convert("RGBA")
    if im.size != (10, 10):
        return None
    px = im.load()
    m = []
    for r in range(10):
        row = []
        for c in range(10):
            r_, g_, b_, a_ = px[c, r]
            v = r_ if a_ >= 128 else 255
            row.append(1 if v >= BLACK_THRESH else 0)
        m.append(row)
    return m


def render_title_mask(text):
    """Render text with Pillow's sized default font; return (mask, l, t, w, h).

    The mask canvas has the text ink starting at row 0 (drawn at -textbbox t).
    """
    font = ImageFont.load_default(size=TITLE_FONT_PX)
    tmp = Image.new("L", (1600, 160), 255)
    d = ImageDraw.Draw(tmp)
    l, t, r, b = d.textbbox((0, 0), text, font=font)
    d.text((-l, -t), text, font=font, fill=0)
    return tmp, l, t, r - l, b - t


# ------------------------------------------------------------------ checks
def check_pdf_structure():
    print("== 1. PDF structure (raw parse) ==")
    for grid in GRIDS:
        fname = f"{grid['name']}_tag36h11_ids{grid['id0']}-{grid['id1']}.pdf"
        path = os.path.join(GENERATED, fname)
        ok = os.path.isfile(path)
        check(ok, f"{fname} exists")
        if not ok:
            continue
        info = parse_pdf(open(path, "rb").read())
        check(info is not None, f"{fname} has PDF magic")
        if info is None:
            continue
        good = (len(info["mediaboxes"]) >= 1 and
                all(abs(float(a)) < 0.5 and abs(float(b)) < 0.5 and
                    abs(float(c) - 612) < 0.5 and abs(float(d) - 792) < 0.5
                    for a, b, c, d in info["mediaboxes"]))
        check(good, f"{fname} MediaBox is US Letter 612x792 pt "
                    f"(found {len(info['mediaboxes'])})")
        if info["cropboxes"]:
            check(False, f"{fname} has unexpected CropBox")
        else:
            check(True, f"{fname} no CropBox")
        check(info["n_pages"] == 1, f"{fname} exactly 1 page (got {info['n_pages']})")
        check(info["counts"] and all(c == 1 for c in info["counts"]),
              f"{fname} /Pages /Count == 1 (got {info['counts']})")


def check_margins(rasters):
    print("== 2-3. Raster + margins ==")
    table = bytes([0 if i < INK_THRESH else 255 for i in range(256)])
    for grid in GRIDS:
        im = rasters[grid["name"]]
        W, H = im.size
        check((W, H) == (PAGE_W_PX, PAGE_H_PX),
              f"{grid['name']}: raster is 2550x3300 px (got {W}x{H})")
        data = im.tobytes()
        rows_t = [data[y * W:(y + 1) * W].translate(table) for y in range(H)]
        # title bbox: all ink in the top-margin band is the title
        tminx = tmaxx = tminy = tmaxy = None
        for y in range(MARGIN_PX):
            row = rows_t[y]
            a = row.find(b"\x00")
            if a >= 0:
                if tminy is None:
                    tminy = y
                tmaxy = y
                b = row.rfind(b"\x00")
                tminx = a if tminx is None else min(tminx, a)
                tmaxx = b if tmaxx is None else max(tmaxx, b)
        # body ink bbox (below the top-margin band)
        minx = miny = 10 ** 9
        maxx = maxy = -1
        for y in range(MARGIN_PX, H):
            row = rows_t[y]
            a = row.find(b"\x00")
            if a >= 0:
                if y < miny:
                    miny = y
                maxy = y
                if a < minx:
                    minx = a
                b = row.rfind(b"\x00")
                if b > maxx:
                    maxx = b
        # ink outside the margin box, title rect excepted (title belongs
        # inside the top margin band by design)
        outside = 0
        for y in range(H):
            row = rows_t[y]
            if y < MARGIN_PX:
                if tminy is not None and tminy <= y <= tmaxy:
                    outside += row[:MARGIN_PX].count(b"\x00")
                    outside += row[W - MARGIN_PX:].count(b"\x00")
                    outside += row[MARGIN_PX:tminx].count(b"\x00")
                    outside += row[tmaxx + 1:W - MARGIN_PX].count(b"\x00")
                else:
                    outside += row.count(b"\x00")
            elif y >= H - MARGIN_PX:
                outside += row.count(b"\x00")
            else:
                outside += row[:MARGIN_PX].count(b"\x00")
                outside += row[W - MARGIN_PX:].count(b"\x00")
        body_ok = (minx >= MARGIN_PX and maxx <= W - 1 - MARGIN_PX
                   and miny >= MARGIN_PX and maxy <= H - 1 - MARGIN_PX)
        title_ok = (tminx is not None and tminx >= MARGIN_PX
                    and tmaxx <= W - 1 - MARGIN_PX and tmaxy < MARGIN_PX)
        check(body_ok and title_ok and outside == 0,
              f"{grid['name']}: body ink inside 0.5 in margin box, title "
              f"inside top margin band, no stray ink (body bbox "
              f"{minx/300:.3f}-{maxx/300:.3f} in x, "
              f"{miny/300:.3f}-{maxy/300:.3f} in y; {outside} px outside)")


def geometry(grid, rasters):
    """Detect tags/squares/title; return dict of measurements."""
    im = rasters[grid["name"]]
    px = im.load()
    comps = components(row_runs(im, BLACK_THRESH))
    ann, sq, cell = grid["ann_px"], grid["sq_px"], grid["cell_px"]
    tags = [b for b in comps
            if 0.85 * ann <= (b[1] - b[0] + 1) <= 1.15 * ann
            and 0.85 * ann <= (b[3] - b[2] + 1) <= 1.15 * ann]
    inset = [b for b in comps if b not in tags and
             any(b[0] >= t[0] and b[1] <= t[1] and b[2] >= t[2] and b[3] <= t[3]
                 for t in tags)]
    squares = [b for b in comps
               if b not in tags and b not in inset
               and 0.6 * sq <= (b[1] - b[0] + 1) <= 1.5 * sq
               and 0.6 * sq <= (b[3] - b[2] + 1) <= 1.5 * sq]
    title = [b for b in comps if b not in tags and b not in inset
             and b not in squares and b[3] < MARGIN_PX]
    other = [b for b in comps if b not in tags and b not in squares
             and b not in title and b not in inset]
    return {"tags": tags, "squares": squares, "title": title,
            "inset": inset, "other": other, "px": px, "im": im}


def check_geometry(rasters):
    print("== 4. Geometry: tags, squares, title (from raster) ==")
    meas = {}
    for grid in GRIDS:
        g = geometry(grid, rasters)
        meas[grid["name"]] = g
        n_tags = grid["rows"] * grid["cols"]
        n_sq = (grid["rows"] - 1) * (grid["cols"] - 1)
        check(len(g["tags"]) == n_tags,
              f"{grid['name']}: found {len(g['tags'])} tag annuli (expected {n_tags})")
        check(len(g["squares"]) == n_sq,
              f"{grid['name']}: found {len(g['squares'])} corner squares (expected {n_sq})")
        check(len(g["title"]) > 0, f"{grid['name']}: title text present")
        check(len(g["other"]) == 0,
              f"{grid['name']}: no ink outside tags/squares/title "
              f"({len(g['other'])} stray components)")
        # tag annulus side == 8 cells
        sides = [(b[1] - b[0] + 1) for b in g["tags"]]
        ok_side = all(abs(s - grid["ann_px"]) <= max(4, 0.05 * grid["ann_px"])
                      for s in sides)
        check(ok_side, f"{grid['name']}: tag black borders are 8 cells "
                       f"(sides {min(sides)}..{max(sides)} px, expected "
                       f"{grid['ann_px']})")
        # square side == 2 cells (gap between black borders)
        ssides = [(b[1] - b[0] + 1, b[3] - b[2] + 1) for b in g["squares"]]
        ok_sq = all(abs(s - grid["sq_px"]) <= max(3, 0.05 * grid["sq_px"])
                    and abs(t - grid["sq_px"]) <= max(3, 0.05 * grid["sq_px"])
                    for s, t in ssides)
        check(ok_sq, f"{grid['name']}: square side == black-border gap (2 cells) "
                     f"({sorted(set(s for s, _ in ssides))} px, expected "
                     f"{grid['sq_px']})")
    return meas


def check_portrait(meas):
    print("== 5. Portrait orientation (larger count along 11 in) ==")
    for grid in GRIDS:
        g = meas[grid["name"]]
        xs = sorted({(b[0] + b[1]) / 2 for b in g["tags"]})
        ys = sorted({(b[2] + b[3]) / 2 for b in g["tags"]})
        check(len(ys) == grid["rows"] and len(xs) == grid["cols"],
              f"{grid['name']}: {len(ys)} tag rows along 11 in, {len(xs)} cols "
              f"along 8.5 in (expected {grid['rows']}x{grid['cols']})")
        if len(xs) >= 2 and len(ys) >= 2:
            dx = xs[1] - xs[0]
            dy = ys[1] - ys[0]
            check(abs(dx - dy) <= 2, f"{grid['name']}: square grid (x spacing "
                                     f"{dx:.1f} px == y spacing {dy:.1f} px)")
            check((len(ys) - 1) * dy > (len(xs) - 1) * dx,
                  f"{grid['name']}: vertical extent > horizontal extent "
                  f"({(len(ys)-1)*dy} px vs {(len(xs)-1)*dx} px)")
        check(len(ys) > len(xs), f"{grid['name']}: rows > cols "
                                 f"({len(ys)} > {len(xs)})")


def check_squares(meas):
    print("== 6. Corner squares: intersection-centered, corner-touch ==")
    for grid in GRIDS:
        g = meas[grid["name"]]
        px = g["px"]
        tags = g["tags"]
        all_ok = True
        for (x0, x1, y0, y1) in g["squares"]:
            scx, scy = (x0 + x1) / 2, (y0 + y1) / 2
            near = [t for t in tags
                    if abs((t[0] + t[1]) / 2 - scx) < 0.6 * grid["s_px"]
                    and abs((t[2] + t[3]) / 2 - scy) < 0.6 * grid["s_px"]]
            if len(near) != 4:
                check(False, f"{grid['name']}: square at ({x0},{y0}) has "
                             f"{len(near)} surrounding tags (expected 4)")
                all_ok = False
                continue
            mc = (sum((t[0] + t[1]) / 2 for t in near) / 4,
                  sum((t[2] + t[3]) / 2 for t in near) / 4)
            if abs(mc[0] - scx) > 3 or abs(mc[1] - scy) > 3:
                check(False, f"{grid['name']}: square at ({x0},{y0}) not "
                             f"centered at intersection (off "
                             f"{mc[0]-scx:.1f},{mc[1]-scy:.1f} px)")
                all_ok = False
            # corner-touch: each square corner diagonally touches a distinct
            # surrounding tag's black-border corner; no shared pixels
            corners = [((x0 - 1, y0 - 1), lambda t: (t[1], t[3])),
                       ((x1 + 1, y0 - 1), lambda t: (t[0], t[3])),
                       ((x0 - 1, y1 + 1), lambda t: (t[1], t[2])),
                       ((x1 + 1, y1 + 1), lambda t: (t[0], t[2]))]
            seen = []
            for (cx_, cy_), key in corners:
                hit = [t for t in near
                       if abs(key(t)[0] - cx_) <= 2 and abs(key(t)[1] - cy_) <= 2]
                if len(hit) == 1:
                    seen.append(tags.index(hit[0]))
                else:
                    check(False, f"{grid['name']}: square corner ({cx_},{cy_}) "
                                 f"not touching a tag border corner "
                                 f"({len(hit)} matches)")
                    all_ok = False
            if len(set(seen)) != 4:
                check(False, f"{grid['name']}: square at ({x0},{y0}) corners "
                             f"touch only {len(set(seen))} distinct tags")
                all_ok = False
            bridges = [(x0, y0 - 1), (x0 - 1, y0), (x1, y0 - 1), (x1 + 1, y0),
                       (x0, y1 + 1), (x0 - 1, y1), (x1, y1 + 1), (x1 + 1, y1)]
            shared = [b for b in bridges if px[b[0], b[1]] < BLACK_THRESH]
            if shared:
                check(False, f"{grid['name']}: square at ({x0},{y0}) shares "
                             f"black pixels with tags at {shared}")
                all_ok = False
        check(all_ok, f"{grid['name']}: all {len(g['squares'])} squares "
                      f"intersection-centered and corner-touching "
                      f"(no shared ink)")


def check_title(meas):
    print("== 7. Title: filename, top-centered, margin, only text ==")
    for grid in GRIDS:
        g = meas[grid["name"]]
        fname = f"{grid['name']}_tag36h11_ids{grid['id0']}-{grid['id1']}.pdf"
        tcomps = g["title"]
        if not tcomps:
            check(False, f"{fname}: no title ink found")
            continue
        x0 = min(b[0] for b in tcomps)
        x1 = max(b[1] for b in tcomps)
        y0 = min(b[2] for b in tcomps)
        y1 = max(b[3] for b in tcomps)
        check(y1 < MARGIN_PX,
              f"{fname}: title ink y {y0}-{y1} fully inside 0.5 in top margin")
        check(abs((x0 + x1) / 2 - PAGE_W_PX / 2) <= 25,
              f"{fname}: title centered (center x {(x0+x1)/2:.1f} vs "
              f"{PAGE_W_PX/2})")
        tag_top = min(b[2] for b in g["tags"])
        sq_top = min(b[2] for b in g["squares"])
        check(y1 < min(tag_top, sq_top),
              f"{fname}: title does not overlap tags/squares "
              f"(title bottom {y1} < content top {min(tag_top, sq_top)})")
        # content match against an independent render of the expected filename
        mask, l, t, w, h = render_title_mask(fname)
        check(abs(w - (x1 - x0 + 1)) <= 4 and abs(h - (y1 - y0 + 1)) <= 2,
              f"{fname}: title ink bbox {x1-x0+1}x{y1-y0+1} matches filename "
              f"render {w}x{h} at font {TITLE_FONT_PX}")
        mx, my = x0, y0
        px = g["px"]
        maskpx = mask.load()
        mask_ink = [(xx, yy) for yy in range(h) for xx in range(w)
                    if maskpx[xx, yy] < BLACK_THRESH]
        page_ink = [(xx, yy) for xx in range(w) for yy in range(h)
                    if px[mx + xx, my + yy] < BLACK_THRESH]
        m_ok = sum(1 for (xx, yy) in mask_ink
                   if any(px[mx + xx + dx, my + yy + dy] < BLACK_THRESH
                          for dx in range(-3, 4) for dy in range(-3, 4)
                          if 0 <= mx + xx + dx < PAGE_W_PX
                          and 0 <= my + yy + dy < PAGE_H_PX))
        p_ok = sum(1 for (xx, yy) in page_ink
                   if any(maskpx[xx + dx, yy + dy] < BLACK_THRESH
                          for dx in range(-3, 4) for dy in range(-3, 4)
                          if 0 <= xx + dx < w and 0 <= yy + dy < h))
        mcov = 100.0 * m_ok / max(1, len(mask_ink))
        pcov = 100.0 * p_ok / max(1, len(page_ink))
        check(mcov >= 92 and pcov >= 92,
              f"{fname}: title pixels match filename render "
              f"(mask coverage {mcov:.1f}%, page coverage {pcov:.1f}%)")


def check_ids(meas):
    print("== 8. ID decode from pixels vs tag36h11 table ==")
    decoded = {}
    all_ids = []
    for grid in GRIDS:
        g = meas[grid["name"]]
        im = g["im"]
        grid_ids = []
        for bbox in sorted(g["tags"], key=lambda b: (b[2], b[0])):
            m10, pts = sample_tag_cells(im, bbox)
            if m10 is None:
                check(False, f"{grid['name']}: tag bbox {bbox} not square")
                continue
            covered = covered_corners(g, pts)
            outer = all(m10[r][c] == 1 for r in range(10) for c in range(10)
                        if (r in (0, 9) or c in (0, 9)) and (r, c) not in covered)
            ring = all(m10[r][c] == 0 for r in range(1, 9) for c in range(1, 9)
                       if r in (1, 8) or c in (1, 8))
            check(outer and ring, f"{grid['name']}: tag at x={bbox[0]} has "
                                  f"quiet ring + black border (square-covered "
                                  f"corners {sorted(covered)} excepted)")
            tid = decode_matrix([row[2:8] for row in m10[2:8]])
            if tid is None:
                check(False, f"{grid['name']}: tag at x={bbox[0]} undecodable")
                continue
            grid_ids.append(tid)
        expected = set(range(grid["id0"], grid["id1"] + 1))
        check(set(grid_ids) == expected,
              f"{grid['name']}: decoded IDs {sorted(set(grid_ids))} == expected "
              f"{sorted(expected)}")
        check(len(grid_ids) == len(expected),
              f"{grid['name']}: {len(grid_ids)} unique tags decoded")
        check(all(0 <= i < NCODES for i in grid_ids),
              f"{grid['name']}: IDs in valid range 0..{NCODES-1}")
        decoded[grid["name"]] = grid_ids
        all_ids.extend(grid_ids)
    check(len(set(all_ids)) == len(all_ids) and len(all_ids) == 38,
          f"no repeated IDs across grids ({len(all_ids)} tags total)")
    return decoded


def check_json(meas, decoded):
    print("== 9. JSON sidecars vs measured pixels ==")
    for grid in GRIDS:
        fname = f"{grid['name']}_tag36h11_ids{grid['id0']}-{grid['id1']}"
        jpath = os.path.join(GENERATED, fname + ".json")
        ok = os.path.isfile(jpath)
        check(ok, f"{fname}.json exists")
        if not ok:
            continue
        data = json.load(open(jpath))
        meta = data.get("meta", {})
        m_ok = (meta.get("family") == "tag36h11"
                and meta.get("rows") == grid["rows"]
                and meta.get("cols") == grid["cols"]
                and meta.get("page_size_in") == [PAGE_W_IN, PAGE_H_IN]
                and meta.get("margin_in") == MARGIN_IN
                and meta.get("units") == "inches"
                and meta.get("origin") == "top-left corner of tag 0"
                and meta.get("x_axis") == "right"
                and meta.get("y_axis") == "down")
        check(m_ok, f"{fname}.json meta fields correct (got {meta})")
        g = meas[grid["name"]]
        tags = sorted(g["tags"], key=lambda b: (b[2], b[0]))
        # origin = top-left corner of tag 0 (measured from pixels)
        t0 = tags[0]
        cell = (t0[1] - t0[0] + 1) / 8.0
        total = 10 * cell
        ox = (t0[0] + t0[1]) / 2 - total / 2
        oy = (t0[2] + t0[3]) / 2 - total / 2
        jtags = data.get("tags", [])
        check(len(jtags) == len(tags), f"{fname}.json has {len(jtags)} tags")
        ids_ok = all(j["id"] == decoded[grid["name"]][i]
                      for i, j in enumerate(jtags[:len(tags)]))
        check(ids_ok, f"{fname}.json tag IDs match decoded IDs in row-major order")
        terr = 0
        for i, j in enumerate(jtags):
            b = tags[i]
            cx = ((b[0] + b[1]) / 2 - ox) / 300
            cy = ((b[2] + b[3]) / 2 - oy) / 300
            err = max(abs(cx - j["center"]["x_in"]), abs(cy - j["center"]["y_in"]),
                      abs(total / 300 - j["size_in"]))
            terr = max(terr, err)
        check(terr <= 0.01,
              f"{fname}.json tag centers/sizes match pixels within 0.01 in "
              f"(max err {terr:.4f})")
        jsquares = data.get("squares", [])
        check(len(jsquares) == len(g["squares"]),
              f"{fname}.json has {len(jsquares)} squares")
        serr = 0
        for j, b in zip(jsquares, sorted(g["squares"],
                                          key=lambda b: (b[2], b[0]))):
            cx = ((b[0] + b[1]) / 2 - ox) / 300
            cy = ((b[2] + b[3]) / 2 - oy) / 300
            serr = max(serr, abs(cx - j["center"]["x_in"]),
                       abs(cy - j["center"]["y_in"]),
                       abs((b[1] - b[0] + 1) / 300 - j["size_in"]))
        check(serr <= 0.01,
              f"{fname}.json square centers/sizes match pixels within 0.01 in "
              f"(max err {serr:.4f})")


def check_ground_truth(meas, gt_dir):
    print("== 10. Ground truth vs official apriltag-imgs ==")
    if gt_dir is None:
        print("  [SKIP] no --gt dir (clone AprilRobotics/apriltag-imgs and "
              "pass --gt <dir>)")
        return
    tagdir = os.path.join(gt_dir, "tag36h11")
    if not os.path.isdir(tagdir):
        check(False, f"ground-truth dir missing: {tagdir}")
        return
    mism = []
    n = 0
    for grid in GRIDS:
        g = meas[grid["name"]]
        im = g["im"]
        for bbox in g["tags"]:
            m10, pts = sample_tag_cells(im, bbox)
            tid = decode_matrix([row[2:8] for row in m10[2:8]])
            off = official_matrix(os.path.join(tagdir, f"tag36_11_{tid:05d}.png"))
            covered = covered_corners(g, pts)
            same = all((r, c) in covered or off[r][c] == m10[r][c]
                       for r in range(10) for c in range(10))
            if off is None or not same:
                mism.append(f"PDF tag {tid} in {grid['name']}")
            n += 1
    rng = random.Random(GT_SAMPLE_SEED)
    for tid in rng.sample(range(38, NCODES), GT_SAMPLE_N):
        mine = render10(TAG36H11_CODES[tid])
        off = official_matrix(os.path.join(tagdir, f"tag36_11_{tid:05d}.png"))
        if off is None or off != mine:
            mism.append(f"rendered ID {tid}")
        n += 1
    check(n == 38 + GT_SAMPLE_N, f"compared {n} tags vs official PNGs")
    check(not mism, f"all compared tags pixel-match official PNGs "
                    f"(mismatches: {mism if mism else 'none'})")


def check_reproducibility():
    print("== 11. Reproducibility (byte-compare PDFs + JSONs) ==")
    gen = os.path.join(SCRIPT_DIR, "gen_grids.py")
    with tempfile.TemporaryDirectory() as tmp:
        proc = subprocess.run([sys.executable, gen, "--out", tmp],
                              capture_output=True, text=True, cwd=REPO_ROOT)
        check(proc.returncode == 0,
              f"gen_grids.py --out <tmp> exit 0 (got {proc.returncode}; "
              f"stderr: {proc.stderr.strip()[:200]})")
        for grid in GRIDS:
            base = f"{grid['name']}_tag36h11_ids{grid['id0']}-{grid['id1']}"
            for ext in (".pdf", ".json"):
                a = open(os.path.join(GENERATED, base + ext), "rb").read()
                p = os.path.join(tmp, base + ext)
                ok = os.path.isfile(p) and open(p, "rb").read() == a
                check(ok, f"{base}{ext} regenerates byte-identically")


def check_fit_math(meas):
    print("== 12. Fit math from measured tag sides (720p, 1280 px, HFOV %.0f deg) =="
          % HFOV_DEG)
    px_per_in = {d: CAM_W_PX / (2 * d * 12 * math.tan(math.radians(HFOV_DEG / 2)))
                 for d in DIST_FT}
    print(f"  px/in: 2 ft -> {px_per_in[2.0]:.1f}, 2.5 ft -> {px_per_in[2.5]:.1f}")
    print(f"  {'grid':<9} {'meas in':>8} {'px @2ft':>8} {'px @2.5ft':>9}")
    for grid in GRIDS:
        g = meas[grid["name"]]
        cells = [(b[1] - b[0] + 1) / 8.0 for b in g["tags"]]
        total_in = (10 * sum(cells) / len(cells)) / 300
        p2 = total_in * px_per_in[2.0]
        p25 = total_in * px_per_in[2.5]
        print(f"  {grid['name']:<9} {total_in:>8.4f} {p2:>8.1f} {p25:>9.1f}")
        check(p25 >= MIN_TAG_PX,
              f"{grid['name']}: measured tag side {p25:.1f} px at 2.5 ft "
              f">= {MIN_TAG_PX} px")
        check(abs(total_in - grid["tag_in"]) <= 0.05,
              f"{grid['name']}: measured tag side {total_in:.4f} in == nominal "
              f"{grid['tag_in']} in")


def check_hygiene():
    print("== 13. Repo hygiene ==")
    proc = subprocess.run(["git", "-C", REPO_ROOT, "ls-files"],
                          capture_output=True, text=True)
    check(proc.returncode == 0, "git ls-files runs")
    files = proc.stdout.splitlines()
    third = [f for f in files if "tag36_11" in f or "apriltag-imgs" in f.lower()]
    check(not third, f"no apriltag-imgs PNGs or third-party files tracked "
                     f"(found: {third if third else 'none'})")
    pkg = [f for f in files if f.startswith("utils/apriltag_grid_gen/")]
    check(all(f.endswith(".py") for f in pkg),
          f"utils/apriltag_grid_gen/ contains only .py files ({pkg})")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gt", default=None,
                    help="dir of AprilRobotics/apriltag-imgs clone "
                         "(auto-detected: /tmp/sentinel/validator-01/apriltag-imgs)")
    args = ap.parse_args()
    gt_dir = args.gt
    if gt_dir is None:
        auto = "/tmp/sentinel/validator-01/apriltag-imgs"
        if os.path.isdir(os.path.join(auto, "tag36h11")):
            gt_dir = auto

    print("== T001 independent validator check (validator-01, revised spec) ==")
    check_pdf_structure()
    rasters = {}
    for grid in GRIDS:
        fname = f"{grid['name']}_tag36h11_ids{grid['id0']}-{grid['id1']}.pdf"
        path = os.path.join(GENERATED, fname)
        rasters[grid["name"]] = rasterize(path) if os.path.isfile(path) else None
    check_margins(rasters)
    meas = check_geometry(rasters)
    check_portrait(meas)
    check_squares(meas)
    check_title(meas)
    decoded = check_ids(meas)
    check_json(meas, decoded)
    check_ground_truth(meas, gt_dir)
    check_reproducibility()
    check_fit_math(meas)
    check_hygiene()

    print()
    if failures:
        print(f"RESULT: FAIL ({len(failures)} failure(s))")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("RESULT: PASS (all checks)")


if __name__ == "__main__":
    main()
