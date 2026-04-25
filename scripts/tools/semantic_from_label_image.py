#!/usr/bin/env python3
"""Convert a color-labeled semantic image into semantic_zones.yaml.

This script is designed for a fixed-map workflow:
1) Paint one semantic label image with fixed RGB colors.
2) Read map metadata from a ROS map yaml (resolution/origin/yaw).
3) Export polygons in map frame to semantic_zones.yaml.

Default extraction mode is axis-aligned bounding box per connected component,
which is robust and simple for rectangular zones such as speed bumps.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import yaml
from PIL import Image


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]

# 用户在这里修改输入输出路径，不再需要命令行传参。
LABEL_IMAGE_PATH = WORKSPACE_ROOT / "rmuc_2025.png"
MAP_YAML_PATH = WORKSPACE_ROOT / "src/gxu2026_sentry_nav/gxu2026_nav_bringup/map/simulation/rmuc_2025.yaml"
OUTPUT_YAML_PATH = WORKSPACE_ROOT / "src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/semantic_zones.yaml"
PALETTE_YAML_PATH: Path | None = None
MIN_PIXELS = 20


RGB = Tuple[int, int, int]
RC = Tuple[int, int]


@dataclass(frozen=True)
class LabelSpec:
    name: str
    rgb: RGB
    zone_type: str


DEFAULT_LABELS: Sequence[LabelSpec] = (
    LabelSpec(name="speed_bump", rgb=(0, 0, 255), zone_type="speed_bump"),
)


def _load_palette(path: Path | None) -> Sequence[LabelSpec]:
    if path is None:
        return DEFAULT_LABELS

    raw = yaml.safe_load(path.read_text()) or {}
    labels = raw.get("labels", [])
    parsed: List[LabelSpec] = []
    for item in labels:
        name = str(item["name"])
        rgb = tuple(int(v) for v in item["rgb"])
        if len(rgb) != 3:
            raise ValueError(f"Invalid rgb length for label '{name}': {rgb}")
        zone_type = str(item["zone_type"])
        parsed.append(LabelSpec(name=name, rgb=(rgb[0], rgb[1], rgb[2]), zone_type=zone_type))
    if not parsed:
        raise ValueError("Palette file has no labels")
    return parsed


def _load_map_metadata(path: Path) -> Tuple[float, Tuple[float, float], float]:
    raw = yaml.safe_load(path.read_text()) or {}
    resolution = float(raw["resolution"])
    origin = raw["origin"]
    if not isinstance(origin, list) or len(origin) < 3:
        raise ValueError("Map yaml 'origin' must be [x, y, yaw]")
    origin_xy = (float(origin[0]), float(origin[1]))
    yaw = float(origin[2])
    return resolution, origin_xy, yaw


def _neighbors(r: int, c: int, h: int, w: int) -> Iterable[RC]:
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        rr = r + dr
        cc = c + dc
        if 0 <= rr < h and 0 <= cc < w:
            yield rr, cc


def _connected_components(mask: List[List[bool]]) -> List[List[RC]]:
    h = len(mask)
    w = len(mask[0]) if h else 0
    visited = [[False for _ in range(w)] for _ in range(h)]
    comps: List[List[RC]] = []

    for r in range(h):
        for c in range(w):
            if not mask[r][c] or visited[r][c]:
                continue

            q: deque[RC] = deque([(r, c)])
            visited[r][c] = True
            comp: List[RC] = []
            while q:
                rr, cc = q.popleft()
                comp.append((rr, cc))
                for nr, nc in _neighbors(rr, cc, h, w):
                    if mask[nr][nc] and not visited[nr][nc]:
                        visited[nr][nc] = True
                        q.append((nr, nc))

            comps.append(comp)
    return comps


def _pixel_corner_to_map(
    col: float,
    row: float,
    image_h: int,
    resolution: float,
    origin_xy: Tuple[float, float],
    yaw: float,
) -> Tuple[float, float]:
    # local map coordinates when yaw=0 and image origin is top-left.
    lx = col * resolution
    ly = (image_h - row) * resolution

    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    wx = origin_xy[0] + cos_y * lx - sin_y * ly
    wy = origin_xy[1] + sin_y * lx + cos_y * ly
    return wx, wy


def _bbox_polygon_map(
    component: List[RC],
    image_h: int,
    resolution: float,
    origin_xy: Tuple[float, float],
    yaw: float,
) -> List[Tuple[float, float]]:
    rows = [rc[0] for rc in component]
    cols = [rc[1] for rc in component]
    min_r = min(rows)
    max_r = max(rows)
    min_c = min(cols)
    max_c = max(cols)

    # Rectangle corners in pixel-corner coordinates (clockwise in image frame).
    corners_px = [
        (min_c, min_r),
        (max_c + 1, min_r),
        (max_c + 1, max_r + 1),
        (min_c, max_r + 1),
    ]

    poly: List[Tuple[float, float]] = []
    for c, r in corners_px:
        x, y = _pixel_corner_to_map(
            col=float(c),
            row=float(r),
            image_h=image_h,
            resolution=resolution,
            origin_xy=origin_xy,
            yaw=yaw,
        )
        poly.append((x, y))
    return poly


def _image_to_rgb_array(path: Path) -> List[List[RGB]]:
    img = Image.open(path).convert("RGB")
    w, h = img.size
    px = img.load()
    data: List[List[RGB]] = []
    for r in range(h):
        row: List[RGB] = []
        for c in range(w):
            rgb = px[c, r]
            row.append((int(rgb[0]), int(rgb[1]), int(rgb[2])))
        data.append(row)
    return data


def _build_mask(rgb_data: List[List[RGB]], target: RGB) -> List[List[bool]]:
    h = len(rgb_data)
    w = len(rgb_data[0]) if h else 0
    mask: List[List[bool]] = [[False for _ in range(w)] for _ in range(h)]
    for r in range(h):
        for c in range(w):
            mask[r][c] = rgb_data[r][c] == target
    return mask


def _round_poly(poly: List[Tuple[float, float]], digits: int = 3) -> List[List[float]]:
    return [[round(x, digits), round(y, digits)] for x, y in poly]


def convert(
    label_image: Path,
    map_yaml: Path,
    output_yaml: Path,
    palette_path: Path | None,
    min_pixels: int,
) -> None:
    labels = _load_palette(palette_path)
    resolution, origin_xy, yaw = _load_map_metadata(map_yaml)
    rgb_data = _image_to_rgb_array(label_image)
    image_h = len(rgb_data)

    zones: List[Dict[str, object]] = []
    counts: Dict[str, int] = {}

    for label in labels:
        mask = _build_mask(rgb_data, label.rgb)
        components = _connected_components(mask)
        kept_components = [comp for comp in components if len(comp) >= min_pixels]
        if not kept_components:
            continue

        use_suffix = len(kept_components) > 1
        idx = 0
        for comp in kept_components:
            idx += 1
            counts[label.name] = counts.get(label.name, 0) + 1

            poly = _bbox_polygon_map(
                component=comp,
                image_h=image_h,
                resolution=resolution,
                origin_xy=origin_xy,
                yaw=yaw,
            )

            zones.append(
                {
                    "name": f"{label.name}_{idx}" if use_suffix else label.name,
                    "type": label.zone_type,
                    "vertices": _round_poly(poly),
                }
            )

    out = {
        "zones": zones,
    }
    output_yaml.write_text(yaml.safe_dump(out, sort_keys=False, allow_unicode=False))

    total = sum(counts.values())
    print(f"Generated {total} zones -> {output_yaml}")
    for k in sorted(counts.keys()):
        print(f"  {k}: {counts[k]}")


def main() -> int:
    if not LABEL_IMAGE_PATH.exists():
        raise FileNotFoundError(f"Label image not found: {LABEL_IMAGE_PATH}")
    if not MAP_YAML_PATH.exists():
        raise FileNotFoundError(f"Map yaml not found: {MAP_YAML_PATH}")

    OUTPUT_YAML_PATH.parent.mkdir(parents=True, exist_ok=True)
    convert(
        label_image=LABEL_IMAGE_PATH,
        map_yaml=MAP_YAML_PATH,
        output_yaml=OUTPUT_YAML_PATH,
        palette_path=PALETTE_YAML_PATH,
        min_pixels=MIN_PIXELS,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
