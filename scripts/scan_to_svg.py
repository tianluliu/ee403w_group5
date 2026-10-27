#!/usr/bin/env python3

import csv
import math
import sys
import yaml
from pathlib import Path

if len(sys.argv) != 2:
    print("Usage: scan_to_svg.py <scan.yaml>")
    sys.exit(1)

path = Path(sys.argv[1])

with path.open() as f:
    scan = next(
        doc for doc in yaml.safe_load_all(f)
        if doc is not None
    )   

angle = float(scan["angle_min"])
increment = float(scan["angle_increment"])
range_min = float(scan["range_min"])
range_max = float(scan["range_max"])

points = []

for value in scan["ranges"]:
    r = float(value)

    if math.isfinite(r) and range_min <= r <= range_max:
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        points.append((angle, r, x, y))

    angle += increment

csv_path = path.with_suffix(".csv")
svg_path = path.with_suffix(".svg")

with csv_path.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["angle_rad", "distance_m", "x_m", "y_m"])
    writer.writerows(points)

xs = [p[2] for p in points]
ys = [p[3] for p in points]

margin = 0.25
min_x = min(xs) - margin
max_x = max(xs) + margin
min_y = min(ys) - margin
max_y = max(ys) + margin

width = max_x - min_x
height = max_y - min_y

dots = "\n".join(
    f'<circle cx="{x}" cy="{-y}" r="0.012" fill="black"/>'
    for _, _, x, y in points
)

svg = f"""<svg xmlns="http://www.w3.org/2000/svg"
viewBox="{min_x} {-max_y} {width} {height}">
<rect x="{min_x}" y="{-max_y}" width="{width}" height="{height}" fill="white"/>
{dots}
<circle cx="0" cy="0" r="0.05" fill="red"/>
</svg>
"""

svg_path.write_text(svg)

print(f"Valid points: {len(points)}")
print(f"CSV: {csv_path}")
print(f"SVG: {svg_path}")
