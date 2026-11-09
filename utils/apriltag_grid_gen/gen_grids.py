#!/usr/bin/env python3
"""Generate print-ready US Letter PDFs of tag36h11 AprilTag calibration grids.

Pure python (Pillow only), no ROS. Regenerates all three PDFs reproducibly:

    uv run --with pillow python3 utils/apriltag_grid_gen/gen_grids.py [--out DIR]

Output files (default DIR = <repo root>/generated):
    grid_3x2_tag36h11_ids0-5.pdf   6 tags, ids 0-5,   tag side 2.20 in
    grid_4x3_tag36h11_ids6-17.pdf  12 tags, ids 6-17, tag side 1.70 in
    grid_5x4_tag36h11_ids18-37.pdf 20 tags, ids 18-37, tag side 1.40 in

Each page is US Letter (8.5 x 11 in) rendered at 300 dpi (2550 x 3300 px) and
exported as PDF at 300 dpi resolution, so a 100%-scale print reproduces the
exact inch dimensions. All content (tags, labels, title) sits inside a 0.5 in
margin so every ordinary printer can print it.

Tag rendering convention matches AprilRobotics/apriltag-imgs (verified
pixel-identical for all 587 tags): 10x10 cells = 1-cell white quiet zone,
1-cell black border, 6x6 data area; data bit 1 = white, 0 = black.
"""

import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

from tag36h11 import BIT_X, BIT_Y, NCODES, TAG36H11_CODES

PAGE_W_IN = 8.5
PAGE_H_IN = 11.0
MARGIN_IN = 0.5
DPI = 300

PAGE_W_PX = int(PAGE_W_IN * DPI)  # 2550
PAGE_H_PX = int(PAGE_H_IN * DPI)  # 3300
MARGIN_PX = int(MARGIN_IN * DPI)  # 150

LABEL_H_IN = 0.35
LABEL_GAP_IN = 0.10
ROW_GAP_IN = 0.60

# cols x rows, tag side length in inches, first tag id (ids are consecutive)
GRIDS = [
    {"name": "grid_3x2", "cols": 3, "rows": 2, "tag_in": 2.20, "id_start": 0},
    {"name": "grid_4x3", "cols": 4, "rows": 3, "tag_in": 1.70, "id_start": 6},
    {"name": "grid_5x4", "cols": 5, "rows": 4, "tag_in": 1.40, "id_start": 18},
]


def _font(size_px):
    """Default font at a requested size (Pillow >= 10.1)."""
    return ImageFont.load_default(size=size_px)


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


def build_page(grid):
    """Build one full letter page (L mode) for a grid spec."""
    page = Image.new("L", (PAGE_W_PX, PAGE_H_PX), 255)
    draw = ImageDraw.Draw(page)

    cols, rows = grid["cols"], grid["rows"]
    tag_in = grid["tag_in"]
    id_start = grid["id_start"]
    ids = list(range(id_start, id_start + cols * rows))

    cell_px = round(tag_in * DPI / 10)
    tag_px = 10 * cell_px
    label_h_px = round(LABEL_H_IN * DPI)
    label_gap_px = round(LABEL_GAP_IN * DPI)
    row_gap_px = round(ROW_GAP_IN * DPI)

    title_font = _font(56)
    note_font = _font(30)
    label_font = _font(44)

    # Title block, top of page (inside the 0.5 in margin)
    draw.text((MARGIN_PX, MARGIN_PX),
              f"AprilTag calibration grid - {cols}x{rows} tag36h11",
              font=title_font, fill=0)
    draw.text((MARGIN_PX, MARGIN_PX + 72),
              f"IDs {ids[0]}-{ids[-1]}  |  tag side {tag_in:.2f} in  |  "
              "print at 100% scale (no fit-to-page), US Letter",
              font=note_font, fill=0)

    # Layout: centers evenly spread across the usable width
    usable_x0 = MARGIN_PX
    usable_x1 = PAGE_W_PX - MARGIN_PX
    usable_w = usable_x1 - usable_x0
    x_centers = [usable_x0 + round((i + 0.5) * usable_w / cols) for i in range(cols)]

    # Rows centered in the space below the title block
    content_y0 = MARGIN_PX + 72 + 56 + 30
    content_y1 = PAGE_H_PX - MARGIN_PX
    block_h = tag_px + label_gap_px + label_h_px
    total_h = rows * block_h + (rows - 1) * row_gap_px
    y_start = content_y0 + (content_y1 - content_y0 - total_h) // 2

    for r in range(rows):
        row_top = y_start + r * (block_h + row_gap_px)
        for c in range(cols):
            tag_id = ids[r * cols + c]
            tag_im = render_tag(tag_id, cell_px)
            left = x_centers[c] - tag_px // 2
            top = row_top
            page.paste(tag_im, (left, top))
            label = f"36h11 #{tag_id}"
            bbox = draw.textbbox((0, 0), label, font=label_font)
            tw = bbox[2] - bbox[0]
            draw.text((x_centers[c] - tw // 2, row_top + tag_px + label_gap_px),
                      label, font=label_font, fill=0)

    return page


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None,
                        help="output directory (default: <repo root>/generated)")
    args = parser.parse_args(argv)

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    out_dir = args.out if args.out else os.path.join(repo_root, "generated")
    os.makedirs(out_dir, exist_ok=True)

    for grid in GRIDS:
        cols, rows = grid["cols"], grid["rows"]
        ids = list(range(grid["id_start"], grid["id_start"] + cols * rows))
        assert all(0 <= i < NCODES for i in ids)
        page = build_page(grid)
        fname = f"{grid['name']}_tag36h11_ids{ids[0]}-{ids[-1]}.pdf"
        path = os.path.join(out_dir, fname)
        # fixed PDF dates so regeneration is byte-reproducible
        page.save(path, "PDF", resolution=float(DPI),
                  creationDate="D:20260101000000Z", modDate="D:20260101000000Z")
        print(f"wrote {path} ({cols}x{rows}, {len(ids)} tags, "
              f"tag side {grid['tag_in']:.2f} in)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
