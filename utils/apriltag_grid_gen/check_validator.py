#!/usr/bin/env python3
"""Independent validator check for T001 tag36h11 calibration grid PDFs (validator-01).

Does NOT import or reuse utils/apriltag_grid_gen/check_grids.py or gen_grids.py
internals. The committed tag36h11.py is imported as DATA ONLY (codeword table +
bit layout), as required by the T001 validation plan.

Run (Mac, pure python, Pillow only):

    uv run --with pillow python3 utils/apriltag_grid_gen/check_validator.py [--gt DIR]

Checks, from the committed PDF bytes themselves (raw parse + JPEG stream
decode, no sips/ghostscript/poppler):

 1. PDF structure: magic, MediaBox [0 0 612 792] (US Letter), exactly 1 page
    (raw parse: /Type /Page object count and /Pages /Count cross-check).
 2. Embedded page image: one image XObject, 2550x3300 px (300 dpi),
    DeviceGray 8bpc, decodable.
 3. Margins: ink bounding box of the decoded committed raster (ink = value
    < 248; JPEG ringing tolerance, true ink is < 248) fully inside the 0.5 in
    printer margin.
 4. Tag detection: connected-component analysis of black pixels in the
    committed raster (union-find over row runs; no layout assumptions); finds
    exactly 6/12/20 black border annuli per grid; measured tag side matches
    the nominal per-grid side within one cell.
 5. Tag structure + ID decode: for every detected tag, sample the 10x10 cell
    grid at cell centers; verify outer white quiet zone and black border
    ring; decode the 6x6 data region to a 36-bit pattern and match it against
    all 587 codewords in the committed tag36h11 table; check counts, ID
    validity (0..586), uniqueness per grid and across grids, and that each
    grid's ID set equals the documented one (0-5 / 6-17 / 18-37).
 6. Ground truth (--gt, default auto-detected clone dir): compare the decoded
    10x10 bitmaps of all 38 PDF tags, plus ~40 randomly sampled other IDs
    rendered independently from the codeword table, against the official
    AprilRobotics/apriltag-imgs tag36_11_*.png images. SKIP if dir absent.
 7. Reproducibility: regenerate via subprocess
    `python3 utils/apriltag_grid_gen/gen_grids.py --out <tmp>` and
    byte-compare all three PDFs against the committed ones.
 8. Fit math: recompute tag side in px at 2 ft / 2.5 ft for a 1280 px (720p)
    webcam with assumed 60 deg HFOV FROM THE MEASURED rendered tag side (not
    the docs); require >= 20 px at 2.5 ft for every grid.
 9. Hygiene: no third-party files committed (no tag36_11 pngs tracked;
    utils/apriltag_grid_gen/ contains only .py files).

Prints [PASS]/[FAIL] per check and a final `RESULT: PASS (all checks)` or
`RESULT: FAIL (n failure(s))`; exits non-zero on any failure.
"""

import argparse
import io
import os
import random
import re
import subprocess
import sys
import tempfile
import zlib

from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tag36h11 import BIT_X, BIT_Y, NCODES, TAG36H11_CODES  # data only

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
GENERATED = os.path.join(REPO_ROOT, "generated")

# --- task spec (done criteria / handoff), hardcoded here for independence ---
GRIDS = [
    {"name": "grid_3x2", "cols": 3, "rows": 2, "tag_in": 2.20, "id0": 0, "id1": 5},
    {"name": "grid_4x3", "cols": 4, "rows": 3, "tag_in": 1.70, "id0": 6, "id1": 17},
    {"name": "grid_5x4", "cols": 5, "rows": 4, "tag_in": 1.40, "id0": 18, "id1": 37},
]
PAGE_W_IN, PAGE_H_IN = 8.5, 11.0
MARGIN_IN = 0.5
DPI = 300
PAGE_W_PX, PAGE_H_PX = 2550, 3300
MARGIN_PX = 150
INK_THRESH = 248   # decoded JPEG: values < 248 are real ink; 249-254 = ringing
BLACK_THRESH = 128

CAM_W_PX = 1280     # 720p webcam horizontal resolution
HFOV_DEG = 60.0     # stated assumption
MIN_TAG_PX = 20     # minimum tag side in px at 2.5 ft
DIST_FT = (2.0, 2.5)

GT_SAMPLE_SEED = 42
GT_SAMPLE_N = 40    # random IDs beyond the 38 used in the PDFs

failures = []


def check(cond, msg):
    status = "PASS" if cond else "FAIL"
    print(f"  [{status}] {msg}")
    if not cond:
        failures.append(msg)


# ---------------------------------------------------------------- PDF parse
def pdf_objects(data):
    """Return dict {objnum: bytes} of indirect objects (raw parse)."""
    objs = {}
    for m in re.finditer(rb'(\d+)\s+(\d+)\s+obj\b(.*?)endobj', data, re.S):
        objs[int(m.group(1))] = m.group(3)
    return objs


def parse_pdf(data):
    """Raw structural facts about a PDF. Returns dict or None on magic fail."""
    if not data.startswith(b"%PDF"):
        return None
    mediaboxes = [mb for mb in re.findall(rb'/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*\]', data)]
    cropboxes = re.findall(rb'/CropBox\s*\[[^\]]*\]', data)
    pages = re.findall(rb'/Type\s*/Page(?![sA-Za-z])', data)
    counts = [int(c) for c in re.findall(rb'/Count\s+(\d+)', data)]
    return {"mediaboxes": mediaboxes, "cropboxes": cropboxes,
            "n_pages": len(pages), "counts": counts}


def extract_page_raster(data):
    """Decode the page image XObject stream. Returns (Image L, w, h) or None."""
    m = re.search(rb'/Subtype\s*/Image(.*?)stream', data, re.S)
    if not m:
        return None
    d = m.group(1)
    w = int(re.search(rb'/Width\s+(\d+)', d).group(1))
    h = int(re.search(rb'/Height\s+(\d+)', d).group(1))
    filt = re.search(rb'/Filter\s*/(\w+)', d)
    fname = filt.group(1).decode() if filt else ""
    cs = re.search(rb'/ColorSpace\s*/(\w+)', d)
    csname = cs.group(1).decode() if cs else ""
    bpc = re.search(rb'/BitsPerComponent\s+(\d+)', d)
    bpcname = int(bpc.group(1)) if bpc else 0
    lm = re.search(rb'/Length\s+(\d+)', d)
    length = int(lm.group(1)) if lm else None
    if length is None:
        return None
    s = data.find(b"stream", m.end() - 6)
    body = data[s:]
    nl = body.find(b"\n") + 1
    raw = body[nl:nl + length]
    if fname == "DCTDecode":
        im = Image.open(io.BytesIO(raw))
    elif fname == "FlateDecode":
        im = Image.open(io.BytesIO(zlib.decompress(raw)))
    else:
        return None
    return im.convert("L"), w, h, csname, bpcname


# ------------------------------------------------- tag detection + decoding
def row_runs(g, thresh):
    """Black-pixel row runs for an L image (list of (y, x0, x1))."""
    W, H = g.size
    data = g.tobytes()
    out = []
    for y in range(H):
        row = data[y * W:(y + 1) * W]
        x = 0
        while x < W:
            if row[x] < thresh:
                x0 = x
                while x < W and row[x] < thresh:
                    x += 1
                out.append((y, x0, x - 1))
            else:
                x += 1
    return out


def components(runs):
    """Union-find over row runs connected by vertical overlap -> bbox per comp."""
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
    for (y, x0, x1) in runs:
        if prev_y is not None and y > prev_y + 1:
            prev = {}  # row gap: nothing connects across it
        prev_y = y
        lab = (y, x0, x1)
        parent[lab] = lab
        for pl, (px0, px1) in prev.items():
            if x0 <= px1 and px0 <= x1:
                union(lab, pl)
        prev[lab] = (x0, x1)
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


def sample_tag_cells(g, bbox):
    """Sample a 10x10 cell matrix at cell centers of a tag's annulus bbox.

    The black border annulus spans 8 cells (1-cell quiet zone around it in
    the full 10x10 tag). Returns None if the bbox is not square enough.
    """
    x0, x1, y0, y1 = bbox
    w, h = x1 - x0 + 1, y1 - y0 + 1
    if abs(w - h) > max(2, 0.05 * w):
        return None
    cell = w / 8.0
    cx0, cy0 = x0 - cell, y0 - cell
    px = g.load()
    m = []
    for r in range(10):
        row = []
        for c in range(10):
            v = px[int(cx0 + (c + 0.5) * cell), int(cy0 + (r + 0.5) * cell)]
            row.append(1 if v >= BLACK_THRESH else 0)
        m.append(row)
    return m


def expected_matrix(code):
    """6x6 data matrix for a codeword, from the committed BIT layout."""
    e = [[0] * 6 for _ in range(6)]
    for i in range(36):
        e[BIT_Y[i] - 1][BIT_X[i] - 1] = (code >> (35 - i)) & 1
    return e


def decode_matrix(m):
    """Decode a 6x6 matrix to its tag ID via the committed codeword table."""
    for tid in range(NCODES):
        if expected_matrix(TAG36H11_CODES[tid]) == m:
            return tid
    return None


def render10(code):
    """Independently render a 10x10 binary tag from the codeword table.

    Layout per the official convention: cell 0/9 white quiet ring, cells 1/8
    black border, data bit i (bit 0 = MSB of the 36-bit code) at full-tag
    cell (BIT_Y[i]+1, BIT_X[i]+1); bit 1 = white.
    """
    g = [[1] * 10 for _ in range(10)]
    for r in range(1, 9):
        for c in range(1, 9):
            if r in (1, 8) or c in (1, 8):
                g[r][c] = 0
    for i in range(36):
        g[BIT_Y[i] + 1][BIT_X[i] + 1] = (code >> (35 - i)) & 1
    return g


def official_matrix(png_path):
    """Load an official apriltag-imgs PNG as a 10x10 binary matrix."""
    im = Image.open(png_path).convert("RGBA")
    px = im.load()
    if im.size != (10, 10):
        return None
    m = []
    for r in range(10):
        row = []
        for c in range(10):
            r_, g_, b_, a_ = px[c, r]
            v = r_ if a_ >= 128 else 255  # transparent -> white background
            row.append(1 if v >= BLACK_THRESH else 0)
        m.append(row)
    return m


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
        data = open(path, "rb").read()
        info = parse_pdf(data)
        check(info is not None, f"{fname} has PDF magic")
        if info is None:
            continue
        mbs = info["mediaboxes"]
        good_mb = all(
            abs(float(a)) < 0.5 and abs(float(b)) < 0.5 and
            abs(float(c) - 612) < 0.5 and abs(float(d) - 792) < 0.5
            for a, b, c, d in mbs) and len(mbs) >= 1
        check(good_mb, f"{fname} MediaBox is US Letter 612x792 pt "
                        f"(found {len(mbs)}, values {mbs[:1]})")
        if info["cropboxes"]:
            check(False, f"{fname} has unexpected CropBox {info['cropboxes'][0]}")
        else:
            check(True, f"{fname} no CropBox")
        check(info["n_pages"] == 1, f"{fname} has exactly 1 /Type /Page object "
                                    f"(got {info['n_pages']})")
        check(info["counts"] and all(c == 1 for c in info["counts"]),
              f"{fname} /Pages /Count == 1 (got {info['counts']})")


def check_raster():
    print("== 2. Embedded page raster ==")
    rasters = {}
    for grid in GRIDS:
        fname = f"{grid['name']}_tag36h11_ids{grid['id0']}-{grid['id1']}.pdf"
        data = open(os.path.join(GENERATED, fname), "rb").read()
        got = extract_page_raster(data)
        ok = got is not None
        check(ok, f"{fname} has a decodable image stream")
        if ok:
            im, w, h, cs, bpc = got
            rasters[grid["name"]] = im
            check((w, h) == (PAGE_W_PX, PAGE_H_PX),
                  f"{fname} raster is {PAGE_W_PX}x{PAGE_H_PX} px (300 dpi) (got {w}x{h})")
            check(cs == "DeviceGray", f"{fname} grayscale (got {cs})")
            check(bpc == 8, f"{fname} 8 bits/component (got {bpc})")
    return rasters


def check_margins(rasters):
    print("== 3. Margins: ink inside 0.5 in (ink threshold %d) ==" % INK_THRESH)
    for grid in GRIDS:
        im = rasters[grid["name"]]
        W, H = im.size
        data = im.tobytes()
        outside = 0
        minx = miny = 10 ** 9
        maxx = maxy = -1
        for y in range(H):
            row = data[y * W:(y + 1) * W]
            for x in range(W):
                if row[x] < INK_THRESH:
                    if (x < MARGIN_PX or x >= W - MARGIN_PX
                            or y < MARGIN_PX or y >= H - MARGIN_PX):
                        outside += 1
                    if x < minx:
                        minx = x
                    if x > maxx:
                        maxx = x
                    if y < miny:
                        miny = y
                    if y > maxy:
                        maxy = y
        inside = (outside == 0 and minx >= MARGIN_PX and miny >= MARGIN_PX
                  and maxx <= W - MARGIN_PX and maxy <= H - MARGIN_PX)
        check(inside, f"{grid['name']}: all ink inside 0.5 in margin "
                      f"(bbox {minx/DPI:.3f}-{maxx/DPI:.3f} in x, "
                      f"{miny/DPI:.3f}-{maxy/DPI:.3f} in y; {outside} px outside)")


def check_tags_and_ids(rasters):
    print("== 4-5. Tag detection, structure, ID decode (from committed raster) ==")
    all_ids = []
    for grid in GRIDS:
        name = grid["name"]
        im = rasters[name]
        runs = row_runs(im, BLACK_THRESH)
        comps = components(runs)
        cell_exp = round(grid["tag_in"] * DPI / 10)
        ann_side_exp = 8 * cell_exp
        lo, hi = 0.8 * ann_side_exp, 1.2 * ann_side_exp
        tags = []
        for (x0, x1, y0, y1) in comps:
            w, h = x1 - x0 + 1, y1 - y0 + 1
            if lo <= w <= hi and lo <= h <= hi and abs(w - h) <= 0.05 * max(w, h):
                tags.append((x0, x1, y0, y1))
        check(len(tags) == grid["cols"] * grid["rows"],
              f"{name}: found {len(tags)} tag annuli (expected "
              f"{grid['cols'] * grid['rows']})")
        grid_ids = []
        sides_px = []
        for bbox in tags:
            m10 = sample_tag_cells(im, bbox)
            if m10 is None:
                check(False, f"{name}: tag bbox {bbox} not square")
                continue
            outer_white = all(m10[r][c] == 1 for r in range(10) for c in range(10)
                              if r in (0, 9) or c in (0, 9))
            border_black = all(m10[r][c] == 0 for r in range(1, 9) for c in range(1, 9)
                               if r in (1, 8) or c in (1, 8))
            check(outer_white, f"{name}: tag at x={bbox[0]} has white quiet ring")
            check(border_black, f"{name}: tag at x={bbox[0]} has black border ring")
            data6 = [row[2:8] for row in m10[2:8]]
            tid = decode_matrix(data6)
            if tid is None:
                check(False, f"{name}: tag at x={bbox[0]} does not decode to any "
                             f"tag36h11 codeword")
                continue
            grid_ids.append(tid)
            w = bbox[1] - bbox[0] + 1
            sides_px.append(w / 8.0 * 10.0)
        expected = set(range(grid["id0"], grid["id1"] + 1))
        check(set(grid_ids) == expected,
              f"{name}: decoded ID set {sorted(set(grid_ids))} == expected "
              f"{sorted(expected)}")
        check(len(grid_ids) == len(expected),
              f"{name}: {len(grid_ids)} tags decoded (no duplicates)")
        check(all(0 <= i < NCODES for i in grid_ids),
              f"{name}: all decoded IDs in valid range 0..{NCODES - 1}")
        if sides_px:
            side_in = (sum(sides_px) / len(sides_px)) / DPI
            check(abs(side_in - grid["tag_in"]) <= 0.05,
                  f"{name}: measured tag side {side_in:.3f} in == nominal "
                  f"{grid['tag_in']:.2f} in (within 0.05)")
            grid["measured_in"] = side_in
        else:
            grid["measured_in"] = None
        all_ids.extend(grid_ids)
    check(len(set(all_ids)) == len(all_ids) and len(all_ids) == 38,
          f"no repeated IDs across grids ({len(all_ids)} tags total)")


def check_ground_truth(rasters, gt_dir):
    print("== 6. Ground truth vs official apriltag-imgs ==")
    if gt_dir is None:
        print("  [SKIP] no --gt dir; clone AprilRobotics/apriltag-imgs and rerun "
              "with --gt <dir>")
        return
    tagdir = os.path.join(gt_dir, "tag36h11")
    if not os.path.isdir(tagdir):
        check(False, f"ground-truth dir missing: {tagdir}")
        return
    mism = []
    n_compared = 0
    # (a) the 38 tags actually in the PDFs, decoded from committed raster
    for grid in GRIDS:
        im = rasters[grid["name"]]
        runs = row_runs(im, BLACK_THRESH)
        comps = components(runs)
        cell_exp = round(grid["tag_in"] * DPI / 10)
        lo, hi = 0.8 * 8 * cell_exp, 1.2 * 8 * cell_exp
        for (x0, x1, y0, y1) in comps:
            w, h = x1 - x0 + 1, y1 - y0 + 1
            if not (lo <= w <= hi and abs(w - h) <= 0.05 * max(w, h)):
                continue
            m10 = sample_tag_cells(im, (x0, x1, y0, y1))
            tid = decode_matrix([row[2:8] for row in m10[2:8]])
            off = official_matrix(os.path.join(tagdir, f"tag36_11_{tid:05d}.png"))
            if off is None or off != m10:
                mism.append(f"PDF tag ID {tid} in {grid['name']}")
            n_compared += 1
    # (b) ~40 random other IDs rendered independently from the committed table
    rng = random.Random(GT_SAMPLE_SEED)
    sample = rng.sample(range(38, NCODES), GT_SAMPLE_N)
    for tid in sample:
        mine = render10(TAG36H11_CODES[tid])
        off = official_matrix(os.path.join(tagdir, f"tag36_11_{tid:05d}.png"))
        if off is None or off != mine:
            mism.append(f"rendered ID {tid}")
        n_compared += 1
    check(n_compared == 38 + GT_SAMPLE_N,
          f"compared {n_compared} tags vs official PNGs (38 PDF + {GT_SAMPLE_N} sample)")
    check(not mism, f"all compared tags pixel-match official PNGs "
                    f"(mismatches: {mism if mism else 'none'})")


def check_reproducibility():
    print("== 7. Reproducibility (byte-compare vs committed) ==")
    gen = os.path.join(SCRIPT_DIR, "gen_grids.py")
    with tempfile.TemporaryDirectory() as tmp:
        proc = subprocess.run(
            [sys.executable, gen, "--out", tmp],
            capture_output=True, text=True, cwd=REPO_ROOT)
        check(proc.returncode == 0,
              f"gen_grids.py --out <tmp> exit 0 (got {proc.returncode}; "
              f"stderr: {proc.stderr.strip()[:200]})")
        for grid in GRIDS:
            fname = f"{grid['name']}_tag36h11_ids{grid['id0']}-{grid['id1']}.pdf"
            a = open(os.path.join(GENERATED, fname), "rb").read()
            p = os.path.join(tmp, fname)
            ok = os.path.isfile(p) and open(p, "rb").read() == a
            check(ok, f"{fname} regenerates byte-identically")


def check_fit_math():
    print("== 8. Fit math from measured tag sides (720p, 1280 px, HFOV %.0f deg) =="
          % HFOV_DEG)
    import math
    px_per_in = {d: CAM_W_PX / (2 * d * 12 * math.tan(math.radians(HFOV_DEG / 2)))
                 for d in DIST_FT}
    print(f"  px/in: 2 ft -> {px_per_in[2.0]:.1f}, 2.5 ft -> {px_per_in[2.5]:.1f}")
    print(f"  {'grid':<9} {'meas in':>8} {'px @2ft':>8} {'px @2.5ft':>9}")
    for grid in GRIDS:
        side_in = grid.get("measured_in")
        if side_in is None:
            check(False, f"{grid['name']}: no measured tag side available")
            continue
        p2 = side_in * px_per_in[2.0]
        p25 = side_in * px_per_in[2.5]
        print(f"  {grid['name']:<9} {side_in:>8.3f} {p2:>8.1f} {p25:>9.1f}")
        check(p25 >= MIN_TAG_PX,
              f"{grid['name']}: measured tag side {p25:.1f} px at 2.5 ft "
              f">= {MIN_TAG_PX} px")


def check_hygiene():
    print("== 9. Repo hygiene (no third-party files committed) ==")
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

    print("== T001 independent validator check (validator-01) ==")
    check_pdf_structure()
    rasters = check_raster()
    check_margins(rasters)
    check_tags_and_ids(rasters)
    check_ground_truth(rasters, gt_dir)
    check_reproducibility()
    check_fit_math()
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
