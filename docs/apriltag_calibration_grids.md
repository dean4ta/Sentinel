# AprilTag calibration grids (T001)

Print-ready US Letter PDFs of `tag36h11` AprilTag grids for calibrating the
720p webcam (`/dev/sentinel_webcam` on the Orin Nano, aimed at the ground
2-2.5 ft in front of the robot). Later tasks use these grids for camera
intrinsic calibration and webcam/arm hand-eye calibration.

## Files

| File (in `generated/`) | Grid (rows x cols, portrait) | Tags | Tag IDs | Tag side | Corner square side |
|---|---|---|---|---|---|
| `grid_3x2_tag36h11_ids0-5.pdf` | 3 x 2 | 6 | 0-5 | 3.3333 in | 0.6667 in |
| `grid_4x3_tag36h11_ids6-17.pdf` | 4 x 3 | 12 | 6-17 | 2.5000 in | 0.5000 in |
| `grid_5x4_tag36h11_ids18-37.pdf` | 5 x 4 | 20 | 18-37 | 1.8667 in | 0.3733 in |

Portrait orientation: rows run along the 11 in page dimension, columns along
8.5 in. The larger grid count always aligns with the 11 in side. Tag centers
form a square grid: the same center-to-center spacing in x and y (the largest
that fits the 0.5 in margin box), so tags are as large as feasible.

Each page contains ONLY the tags and one black corner square centered on
every interior grid intersection (where 4 adjacent tags meet): 2 squares on
the 3x2 grid, 6 on the 4x3, 12 on the 5x4. Each square's side equals the gap
between the black borders of two adjacent tags along an axis (spacing - 8
tag cells), so each of the square's 4 corners touches the black-border
corner of the surrounding tag diagonally — corner-to-corner distance about
1 px (0.003 in) at 300 dpi, never overlapping any black tag pixel. The
square corner extends into the tag's white quiet-zone corner, which is
unavoidable for the diagonal touch and harmless for detection. There is no
text on the pages — no ID labels. Tag identities and positions live in the
JSON sidecar next to each PDF (`grid_*_ids*.json`).

All IDs are unique within and across grids. Every tag is rendered
pixel-identically to the official AprilRobotics/apriltag-imgs `tag36h11`
images (verified for all 587 family tags during development), so any
apriltag3-based detector (e.g. `apriltag` ROS wrapper, `apriltag3` python
bindings) recognizes them.

## JSON sidecar schema

`generated/grid_3x2_tag36h11_ids0-5.json` (etc.):

- `meta`: `{family: "tag36h11", rows, cols, page_size_in: [8.5, 11.0],
  margin_in: 0.5, units: "inches", origin: "top-left corner of tag 0",
  x_axis: "right", y_axis: "down"}`
- `tags`: per tag (row-major, tag 0 = top-left):
  `{id, size_in, center: {x_in, y_in}}` — tag center in inches relative to
  the origin (tag 0's top-left corner), x right, y down. Tag 0's center is
  therefore `(size_in/2, size_in/2)`.
- `squares`: per corner square:
  `{center: {x_in, y_in}, size_in}` — same coordinate frame.

## Print instructions

1. Open each PDF and print on **US Letter (8.5 x 11 in)** paper.
2. **Scale: 100% / "Actual size".** Do NOT use "fit to page" — the tag
   dimensions below assume exact 1:1 printing.
3. All content sits inside a 0.5 in margin, so ordinary printers are fine.
4. The pages carry no labels: identify tags via the JSON sidecar (tag 0 is
   the top-left tag on each page, IDs increase left-to-right, top-to-bottom).
5. Optional: verify with a ruler that a tag side on the 5x4 grid measures
   1.8667 in (the 3x2 grid: 3.3333 in).

## Fit math

Assumptions: 720p webcam, 1280 px horizontal, **HFOV 60 deg** (typical for
Logitech C270-class webcams). Pixels per inch at distance d:

    px/in = 1280 / (2 * d * tan(30 deg))

| Distance | px/in | 3x2 (3.3333 in) | 4x3 (2.5000 in) | 5x4 (1.8667 in) |
|---|---|---|---|---|
| 2.0 ft | 46.2 | 154.0 px | 115.5 px | 86.2 px |
| 2.5 ft | 37.0 | 123.2 px | 92.4 px | 69.0 px |

Minimum requirement is >= 20 px tag side at 2.5 ft; the smallest grid
(5x4) gives 69.0 px, a ~3.5x margin. Even at a conservative 50 deg HFOV
(px/in 45.7 at 2.5 ft) the 5x4 tags are 85 px. Note the camera looks down
at the ground at an angle, so effective pixels per inch drop with
foreshortening; the larger grids (3x2, 4x3) are the ones to use at the
2.5 ft range if detection is marginal.

## Regenerating

Pure python (Pillow only), no ROS, no robot. From the repo root:

```bash
uv run --with pillow python3 utils/apriltag_grid_gen/gen_grids.py
```

Writes the three PDFs and three JSON sidecars into `generated/`. Output is
byte-reproducible (fixed PDF dates, deterministic JSON). Requires Pillow
>= 10.1. The tag36h11 codeword table in
`utils/apriltag_grid_gen/tag36h11.py` is data extracted from
AprilRobotics/apriltag (BSD 2-Clause, U. Michigan / AprilTag team); see the
attribution header in that file.

## Check

```bash
uv run --with pillow python3 utils/apriltag_grid_gen/check_grids.py
```

Verifies page size (US Letter MediaBox), portrait orientation, tag counts,
ID validity and uniqueness, corner-square count/placement, corner-touching
(every square corner within 0.01 in of the diagonal tag's black-border
corner, no shared black pixels between square and tag ink), absence of text
(no ink outside tag and square rects), JSON agreement with the layout,
content inside the 0.5 in margin, byte-identical regeneration (PDFs and
JSONs), tag bitmap structure (quiet zone, border ring, 4 distinct
rotations), and the fit-math minimums. Prints `RESULT: PASS (all checks)`.

## Out of scope (later tasks)

- Camera intrinsic calibration from these grids.
- Webcam/arm hand-eye (extrinsic) calibration.
- Robot-side apriltag detection test on the Orin Nano.
