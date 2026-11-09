#!/usr/bin/env python3
"""Generate print-ready US Letter PDFs of tag36h11 AprilTag calibration grids.

Pure python (Pillow only), no ROS. Regenerates all PDFs and their JSON
sidecars reproducibly:

    uv run --with pillow python3 utils/apriltag_grid_gen/gen_grids.py [--out DIR]

Output files (default DIR = <repo root>/generated):
    grid_3x2_tag36h11_ids0-5.pdf   (+ .json)  3 rows x 2 cols,  6 tags, ids 0-5,   tag side 2.70 in
    grid_4x3_tag36h11_ids6-17.pdf  (+ .json)  4 rows x 3 cols, 12 tags, ids 6-17,  tag side 1.90 in
    grid_5x4_tag36h11_ids18-37.pdf (+ .json)  5 rows x 4 cols, 20 tags, ids 18-37, tag side 1.40 in

Layout (portrait): rows run along the 11 in page dimension, cols along
8.5 in. Pages contain ONLY the tags and one black corner square at each
interior grid intersection (where 4 adjacent tags meet); no text is
drawn, so nothing but tags and squares is printed. Tag IDs and positions
are recorded in the JSON sidecar, with tag centers given in inches
relative to the top-left corner of tag 0 (x right, y down).

Each page is US Letter (8.5 x 11 in) rendered at 300 dpi (2550 x 3300 px)
and exported as PDF at 300 dpi resolution, so a 100%-scale print
reproduces the exact inch dimensions. All content sits inside a 0.5 in
margin so every ordinary printer can print it.

Tag rendering convention matches AprilRobotics/apriltag-imgs (verified
pixel-identical for all 587 tags): 10x10 cells = 1-cell white quiet zone,
1-cell black border, 6x6 data area; data bit 1 = white, 0 = black.
"""

import argparse
import json
import os
import sys

from PIL import Image, ImageDraw

from tag36h11 import BIT_X, BIT_Y, NCODES, TAG36H11_CODES

PAGE_W_IN = 8.5
PAGE_H_IN = 11.0
MARGIN_IN = 0.5
DPI = 300

PAGE_W_PX = int(PAGE_W_IN * DPI)  # 2550
PAGE_H_PX = int(PAGE_H_IN * DPI)  # 3300
MARGIN_PX = int(MARGIN_IN * DPI)  # 150

SQUARE_IN = 0.25  # black corner squares at interior intersections

# Portrait grids: rows = larger count along 11 in, cols along 8.5 in.
# name reads rows x cols. Tag side in inches; ids consecutive, row-major
# starting at the top-left tag (tag 0).
GRIDS = [
    {"name": "grid_3x2", "rows": 3, "cols": 2, "tag_in": 2.70, "id_start": 0},
    {"name": "grid_4x3", "rows": 4, "cols": 3, "tag_in": 1.90, "id_start": 6},
    {"name": "grid_5x4", "rows": 5, "cols": 4, "tag_in": 1.40, "id_start": 18},
]


def render_tag(tag_id, cell_px):
    """Render one tag as an L-mode image of (10*cell_px) square pixels.

    Layout: outer 1-cell white quiet zone, 1-cell black border, 6x6 data.
    Data bit i of the codeword (bit 0 = MSB) maps to full-tag cell
    (BIT_X[i] + 1, BIT_Y[i] + 1); bit 1 = white, bit 0 = black.
    """
    code = TAG36H11_CODES[tag_id]
    size = 10 * cell_px
    im = Image.new("L", (size, size), 255)
    px = im.load()
    # black border ring: cells 1 and 8 on each axis, full cell thickness
    draw = ImageDraw.Draw(im)
    draw.rectangle([cell_px, cell_px, 9 * cell_px - 1, 2 * cell_px - 1], fill=0)
    draw.rectangle([cell_px, 8 * cell_px, 9 * cell_px - 1, 9 * cell_px - 1], fill=0)
    draw.rectangle([cell_px, cell_px, 2 * cell_px - 1, 9 * cell_px - 1], fill=0)
    draw.rectangle([8 * cell_px, cell_px, 9 * cell_px - 1, 9 * cell_px - 1], fill=0)
    # data cells
    for i in range(36):
        bit = (code >> (35 - i)) & 1
        v = 255 if bit else 0
        x0 = (BIT_X[i] + 1) * cell_px
        y0 = (BIT_Y[i] + 1) * cell_px
        for dy in range(cell_px):
            for dx in range(cell_px):
                px[x0 + dx, y0 + dy] = v
    return im


def placements(grid):
    """Compute all tag and corner-square placements for a grid.

    Returns a dict with pixel placements ("tags", "squares") and the same
    placements in inches relative to the top-left corner of tag 0
    ("tags_in", "squares_in", rounded to 4 decimals). Tags are row-major,
    tag 0 = top-left. Corner squares sit centered on the interior grid
    intersections (midpoints between the centers of 4 adjacent tags).
    """
    cols, rows = grid["cols"], grid["rows"]
    tag_in = grid["tag_in"]
    cell_px = round(tag_in * DPI / 10)
    tag_px = 10 * cell_px
    sq_px = round(SQUARE_IN * DPI)

    usable_w = PAGE_W_PX - 2 * MARGIN_PX
    usable_h = PAGE_H_PX - 2 * MARGIN_PX
    x_centers = [MARGIN_PX + round((i + 0.5) * usable_w / cols)
                 for i in range(cols)]
    y_centers = [MARGIN_PX + round((j + 0.5) * usable_h / rows)
                 for j in range(rows)]

    tags = []
    for j in range(rows):
        for i in range(cols):
            tags.append({
                "id": grid["id_start"] + j * cols + i,
                "cx": x_centers[i], "cy": y_centers[j], "px": tag_px,
            })
    squares = []
    for j in range(rows - 1):
        for i in range(cols - 1):
            squares.append({
                "cx": round((x_centers[i] + x_centers[i + 1]) / 2),
                "cy": round((y_centers[j] + y_centers[j + 1]) / 2),
                "px": sq_px,
            })

    origin_x = tags[0]["cx"] - tags[0]["px"] // 2
    origin_y = tags[0]["cy"] - tags[0]["px"] // 2
    tags_in = [
        {"id": t["id"], "size_in": tag_in,
         "center": {"x_in": round((t["cx"] - origin_x) / DPI, 4),
                    "y_in": round((t["cy"] - origin_y) / DPI, 4)}}
        for t in tags
    ]
    squares_in = [
        {"center": {"x_in": round((s["cx"] - origin_x) / DPI, 4),
                    "y_in": round((s["cy"] - origin_y) / DPI, 4)},
         "size_in": SQUARE_IN}
        for s in squares
    ]
    return {"cols": cols, "rows": rows, "tag_in": tag_in,
            "tags": tags, "squares": squares,
            "tags_in": tags_in, "squares_in": squares_in,
            "origin_px": (origin_x, origin_y)}


def build_page(grid):
    """Build one full letter page (L mode): tags + corner squares, no text."""
    pl = placements(grid)
    page = Image.new("L", (PAGE_W_PX, PAGE_H_PX), 255)
    draw = ImageDraw.Draw(page)
    for t in pl["tags"]:
        page.paste(render_tag(t["id"], t["px"] // 10),
                   (t["cx"] - t["px"] // 2, t["cy"] - t["px"] // 2))
    for s in pl["squares"]:
        draw.rectangle([s["cx"] - s["px"] // 2, s["cy"] - s["px"] // 2,
                        s["cx"] - s["px"] // 2 + s["px"] - 1,
                        s["cy"] - s["px"] // 2 + s["px"] - 1], fill=0)
    return page


def grid_json(grid, pl):
    """JSON sidecar describing the layout (schema: meta / tags / squares)."""
    return {
        "meta": {
            "family": "tag36h11",
            "rows": pl["rows"],
            "cols": pl["cols"],
            "page_size_in": [PAGE_W_IN, PAGE_H_IN],
            "margin_in": MARGIN_IN,
            "units": "inches",
            "origin": "top-left corner of tag 0",
            "x_axis": "right",
            "y_axis": "down",
        },
        "tags": pl["tags_in"],
        "squares": pl["squares_in"],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None,
                        help="output directory (default: <repo root>/generated)")
    args = parser.parse_args(argv)

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    out_dir = args.out if args.out else os.path.join(repo_root, "generated")
    os.makedirs(out_dir, exist_ok=True)

    for grid in GRIDS:
        rows, cols = grid["rows"], grid["cols"]
        ids = list(range(grid["id_start"], grid["id_start"] + rows * cols))
        assert all(0 <= i < NCODES for i in ids)
        pl = placements(grid)
        page = build_page(grid)
        base = f"{grid['name']}_tag36h11_ids{ids[0]}-{ids[-1]}"
        pdf_path = os.path.join(out_dir, base + ".pdf")
        # fixed PDF dates so regeneration is byte-reproducible
        page.save(pdf_path, "PDF", resolution=float(DPI),
                  creationDate="D:20260101000000Z", modDate="D:20260101000000Z")
        json_path = os.path.join(out_dir, base + ".json")
        with open(json_path, "w") as f:
            json.dump(grid_json(grid, pl), f, indent=2)
            f.write("\n")
        print(f"wrote {pdf_path} ({rows} rows x {cols} cols, {len(ids)} tags, "
              f"tag side {grid['tag_in']:.2f} in, "
              f"{len(pl['squares'])} corner squares)")
        print(f"wrote {json_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
