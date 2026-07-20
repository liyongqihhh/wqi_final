#!/usr/bin/env python3
"""
Generate the campus_delivery Gazebo world and Nav2 maps.

The generator is intentionally data-driven: edit campus_layout.yaml and rerun
this script to regenerate the SDF world, occupancy map, and keepout mask.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import List, Sequence, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover - dependency check path
    raise SystemExit("PyYAML is required to read campus_layout.yaml") from exc

try:
    from PIL import Image, ImageDraw
except ImportError as exc:  # pragma: no cover - dependency check path
    raise SystemExit("Pillow is required to generate PGM maps") from exc

Point = Tuple[float, float]

FLOOR_HEIGHT = 3.2
# Roads are visual markings on the shared flat ground collision surface. Raised,
# overlapping road collision boxes can catch these small wheels at junctions.
ROAD_Z = 0.002
ROAD_THICKNESS = 0.004
CLEARANCE = 0.5


def safe_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", name.strip().lower()).strip("_")


def pairwise(points: Sequence[Point]):
    return zip(points[:-1], points[1:])


def distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def interpolate(a: Point, b: Point, ratio: float) -> Point:
    return (
        a[0] + (b[0] - a[0]) * ratio,
        a[1] + (b[1] - a[1]) * ratio,
    )


def append_line(out: List[Point], start: Point, end: Point, step: float) -> None:
    samples = max(1, int(math.ceil(distance(start, end) / step)))
    for index in range(1, samples + 1):
        out.append(interpolate(start, end, index / samples))


def append_rounded_corner(
    out: List[Point],
    entry: Point,
    corner: Point,
    exit_point: Point,
    step: float,
) -> None:
    curve_length = distance(entry, corner) + distance(corner, exit_point)
    samples = max(2, int(math.ceil(curve_length / step)))
    for index in range(1, samples + 1):
        t = index / samples
        one_minus_t = 1.0 - t
        out.append(
            (
                one_minus_t * one_minus_t * entry[0]
                + 2.0 * one_minus_t * t * corner[0]
                + t * t * exit_point[0],
                one_minus_t * one_minus_t * entry[1]
                + 2.0 * one_minus_t * t * corner[1]
                + t * t * exit_point[1],
            )
        )


def corner_points(
    previous: Point,
    corner: Point,
    following: Point,
    radius: float,
) -> Tuple[Point, Point]:
    incoming_length = distance(previous, corner)
    outgoing_length = distance(corner, following)
    cut = min(radius, incoming_length / 3.0, outgoing_length / 3.0)
    entry = interpolate(corner, previous, cut / incoming_length)
    exit_point = interpolate(corner, following, cut / outgoing_length)
    return entry, exit_point


def smooth_path(
    points: Sequence[Sequence[float]],
    step: float = 0.9,
    corner_radius: float = 2.0,
) -> List[Point]:
    """Round polyline corners without opposite-direction spline overshoot."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    pts = [
        point
        for index, point in enumerate(pts)
        if index == 0 or point != pts[index - 1]
    ]
    if len(pts) < 3:
        return pts

    closed = pts[0] == pts[-1]
    vertices = pts[:-1] if closed else pts
    if len(vertices) < 3:
        return pts

    corners = [
        corner_points(
            vertices[index - 1],
            vertices[index],
            vertices[(index + 1) % len(vertices)],
            corner_radius,
        )
        for index in range(len(vertices))
    ]

    if closed:
        start = corners[0][1]
        out = [start]
        for index in range(1, len(vertices)):
            entry, exit_point = corners[index]
            append_line(out, out[-1], entry, step)
            append_rounded_corner(
                out, entry, vertices[index], exit_point, step
            )
        first_entry, first_exit = corners[0]
        append_line(out, out[-1], first_entry, step)
        append_rounded_corner(
            out, first_entry, vertices[0], first_exit, step
        )
        out[-1] = start
        return out

    out = [vertices[0]]
    for index in range(1, len(vertices) - 1):
        entry, exit_point = corners[index]
        append_line(out, out[-1], entry, step)
        append_rounded_corner(
            out, entry, vertices[index], exit_point, step
        )
    append_line(out, out[-1], vertices[-1], step)
    return out


def point_in_expanded_rect(
    point: Point,
    center: Sequence[float],
    size: Sequence[float],
    margin: float,
) -> bool:
    x, y = point
    cx, cy = float(center[0]), float(center[1])
    sx, sy = float(size[0]), float(size[1])
    return (cx - sx / 2 - margin) <= x <= (cx + sx / 2 + margin) and (
        cy - sy / 2 - margin
    ) <= y <= (cy + sy / 2 + margin)


def dist_point_to_segment(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denom = vx * vx + vy * vy
    if denom == 0:
        return distance(p, a)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
    proj = (ax + t * vx, ay + t * vy)
    return distance(p, proj)


def dist_point_to_polyline(p: Point, points: Sequence[Point]) -> float:
    return min(dist_point_to_segment(p, a, b) for a, b in pairwise(points))


def validate_layout(layout: dict, road_paths: dict[str, List[Point]]) -> None:
    errors: List[str] = []
    bounds = layout["world_bounds"]
    wall = layout["wall"]
    inner_x_min = wall["west_x"] + wall["thickness"] / 2
    inner_x_max = wall["east_x"] - wall["thickness"] / 2
    inner_y_min = wall["south_y"] + wall["thickness"] / 2
    inner_y_max = wall["north_y"] - wall["thickness"] / 2

    floors = [int(b["floors"]) for b in layout["buildings"]]
    if len(floors) != len(set(floors)):
        errors.append("building floors must be unique")
    for b in layout["buildings"]:
        if int(b["floors"]) < 3:
            errors.append(f"{b['name']} has fewer than 3 floors")
        cx, cy = b["center"]
        sx, sy = b["size"]
        inside_walls = (
            inner_x_min < cx - sx / 2
            and cx + sx / 2 < inner_x_max
            and inner_y_min < cy - sy / 2
            and cy + sy / 2 < inner_y_max
        )
        if not inside_walls:
            errors.append(f"{b['name']} is outside campus walls")

    for road in layout["roads"]:
        name = road["name"]
        width = float(road["width"])
        margin = 0.05
        for p in road_paths[name]:
            x, y = p
            inside_bounds = (
                bounds["x_min"] <= x <= bounds["x_max"]
                and bounds["y_min"] <= y <= bounds["y_max"]
            )
            if not inside_bounds:
                errors.append(f"road {name} point {p} is outside map bounds")
            inside_road_walls = (
                inner_x_min + width / 2 <= x <= inner_x_max - width / 2
                and inner_y_min + width / 2 <= y <= inner_y_max - width / 2
            )
            if not inside_road_walls:
                errors.append(
                    f"road {name} edge is too close to or outside the wall at {p}"
                )
            for b in layout["buildings"]:
                if point_in_expanded_rect(p, b["center"], b["size"], margin):
                    errors.append(f"road {name} conflicts with {b['name']} near {p}")
                    break

    # The loading platform may touch the logistics building. Validate the
    # spawn against both the road network and paved areas.
    spawn = layout["spawn_pose"]
    spawn_pt = (float(spawn["x"]), float(spawn["y"]))
    on_road = any(
        dist_point_to_polyline(spawn_pt, road_paths[road["name"]])
        <= float(road["width"]) / 2 + 0.5
        for road in layout["roads"]
    )
    if not on_road:
        in_platform = any(
            point_in_expanded_rect(
                spawn_pt,
                area["center"],
                area["size"],
                0.0,
            )
            for area in layout.get("paved_areas", [])
        )
        if not in_platform:
            errors.append("spawn_pose is not on a drivable road or platform")

    for tree in layout.get("park", {}).get("trees", []):
        p = (float(tree[0]), float(tree[1]))
        for road in layout["roads"]:
            road_clearance = float(road["width"]) / 2 + 0.8
            if dist_point_to_polyline(
                p, road_paths[road["name"]]
            ) <= road_clearance:
                errors.append(f"park tree at {p} is too close to road {road['name']}")
                break

    if errors:
        joined = "\n - ".join(errors[:40])
        more = "" if len(errors) <= 40 else f"\n... {len(errors) - 40} more errors"
        raise SystemExit(f"Campus layout validation failed:\n - {joined}{more}")


def rgba(values: Sequence[float]) -> str:
    return " ".join(f"{float(v):.3f}" for v in values)


def material(values: Sequence[float]) -> str:
    color = rgba(values)
    return f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"


def box_geom(size: Sequence[float]) -> str:
    return (
        "<geometry><box><size>"
        f"{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}"
        "</size></box></geometry>"
    )


def cylinder_geom(radius: float, length: float) -> str:
    return (
        "<geometry><cylinder>"
        f"<radius>{radius:.3f}</radius>"
        f"<length>{length:.3f}</length>"
        "</cylinder></geometry>"
    )


def sphere_geom(radius: float) -> str:
    return f"<geometry><sphere><radius>{radius:.3f}</radius></sphere></geometry>"


def visual_box(name: str, pose: str, size: Sequence[float], color: Sequence[float]) -> str:
    return f"<visual name='{name}'><pose>{pose}</pose>{box_geom(size)}{material(color)}</visual>"


def collision_box(name: str, pose: str, size: Sequence[float]) -> str:
    return f"<collision name='{name}'><pose>{pose}</pose>{box_geom(size)}</collision>"


def make_ground(layout: dict) -> str:
    b = layout["world_bounds"]
    sx = b["x_max"] - b["x_min"]
    sy = b["y_max"] - b["y_min"]
    cx = (b["x_min"] + b["x_max"]) / 2
    cy = (b["y_min"] + b["y_max"]) / 2
    return f"""
    <model name='campus_ground'>
      <static>true</static>
      <link name='ground'>
        <pose>{cx:.3f} {cy:.3f} -0.015 0 0 0</pose>
        {collision_box('collision', '0 0 0 0 0 0', [sx, sy, 0.03])}
        {visual_box('visual', '0 0 0 0 0 0', [sx, sy, 0.03], [0.36, 0.58, 0.28, 1])}
      </link>
    </model>
"""


def wall_link(name: str, cx: float, cy: float, sx: float, sy: float, h: float) -> str:
    return f"""
      <link name='{name}'>
        <pose>{cx:.3f} {cy:.3f} {h/2:.3f} 0 0 0</pose>
        {collision_box('collision', '0 0 0 0 0 0', [sx, sy, h])}
        {visual_box('visual', '0 0 0 0 0 0', [sx, sy, h], [0.45, 0.45, 0.42, 1])}
      </link>
"""


def make_walls(layout: dict) -> str:
    w = layout["wall"]
    t, h = w["thickness"], w["height"]
    south_len_w = w["south_gate_x_min"] - w["west_x"]
    south_len_e = w["east_x"] - w["south_gate_x_max"]
    parts = [
        wall_link("west_wall", w["west_x"], 0, t, w["north_y"] - w["south_y"], h),
        wall_link("east_wall", w["east_x"], 0, t, w["north_y"] - w["south_y"], h),
        wall_link("north_wall", 0, w["north_y"], w["east_x"] - w["west_x"], t, h),
        wall_link(
            "south_wall_west",
            w["west_x"] + south_len_w / 2,
            w["south_y"],
            south_len_w,
            t,
            h,
        ),
        wall_link(
            "south_wall_east",
            w["south_gate_x_max"] + south_len_e / 2,
            w["south_y"],
            south_len_e,
            t,
            h,
        ),
    ]
    return "<model name='campus_wall'><static>true</static>" + "".join(parts) + "</model>"


def make_roads(layout: dict, road_paths: dict[str, List[Point]]) -> str:
    links: List[str] = []
    road_color = [0.16, 0.16, 0.16, 1]
    park_color = [0.50, 0.50, 0.46, 1]
    line_color = [0.95, 0.78, 0.08, 1]
    for road in layout["roads"]:
        path = road_paths[road["name"]]
        width = float(road["width"])
        color = park_color if road.get("category") == "park" else road_color
        for i, (a, b) in enumerate(pairwise(path)):
            seg_len = distance(a, b)
            if seg_len < 0.05:
                continue
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            cx, cy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            lname = f"{safe_name(road['name'])}_{i:04d}"
            size = [seg_len + 0.35, width, ROAD_THICKNESS]
            visuals = visual_box("visual", "0 0 0 0 0 0", size, color)
            if road["category"] == "main":
                visuals += visual_box(
                    "center_line",
                    "0 0 0.008 0 0 0",
                    [max(0.1, seg_len * 0.55), 0.12, 0.012],
                    line_color,
                )
            links.append(
                f"<link name='{lname}'><pose>{cx:.3f} {cy:.3f} {ROAD_Z:.3f} 0 0 {yaw:.6f}</pose>"
                f"{visuals}</link>"
            )
    for area in layout.get("paved_areas", []):
        cx, cy = area["center"]
        sx, sy = area["size"]
        lname = safe_name(area["name"])
        color = area.get("color", [0.22, 0.22, 0.22, 1])
        size = [float(sx), float(sy), ROAD_THICKNESS]
        links.append(
            f"<link name='{lname}'><pose>{cx:.3f} {cy:.3f} {ROAD_Z:.3f} 0 0 0</pose>"
            f"{visual_box('visual', '0 0 0 0 0 0', size, color)}</link>"
        )
    return "<model name='campus_roads'><static>true</static>" + "".join(links) + "</model>"


def door_pose(direction: str, sx: float, sy: float) -> Tuple[str, List[float]]:
    direction = direction.lower()
    if direction in {"north", "northeast", "northwest"}:
        return f"0 {sy/2 + 0.012:.3f} 1.2 0 0 0", [3.0, 0.05, 2.4]
    if direction in {"south", "southeast", "southwest"}:
        return f"0 {-sy/2 - 0.012:.3f} 1.2 0 0 0", [3.0, 0.05, 2.4]
    if direction == "east":
        return f"{sx/2 + 0.012:.3f} 0 1.2 0 0 0", [0.05, 3.0, 2.4]
    return f"{-sx/2 - 0.012:.3f} 0 1.2 0 0 0", [0.05, 3.0, 2.4]


def make_building(b: dict) -> str:
    sx, sy = map(float, b["size"])
    cx, cy = map(float, b["center"])
    floors = int(b["floors"])
    h = floors * FLOOR_HEIGHT
    body = visual_box("body", f"0 0 {h/2:.3f} 0 0 0", [sx, sy, h], b["color"])
    body += collision_box("body_collision", f"0 0 {h/2:.3f} 0 0 0", [sx, sy, h])
    body += visual_box(
        "roof",
        f"0 0 {h + 0.10:.3f} 0 0 0",
        [sx + 0.6, sy + 0.6, 0.20],
        b["roof_color"],
    )
    sep_color = [0.08, 0.08, 0.08, 1]
    for floor in range(1, floors):
        z = floor * FLOOR_HEIGHT
        body += visual_box(
            f"floor_sep_n_{floor}",
            f"0 {sy/2 + 0.018:.3f} {z:.3f} 0 0 0",
            [sx + 0.1, 0.035, 0.06],
            sep_color,
        )
        body += visual_box(
            f"floor_sep_s_{floor}",
            f"0 {-sy/2 - 0.018:.3f} {z:.3f} 0 0 0",
            [sx + 0.1, 0.035, 0.06],
            sep_color,
        )
        body += visual_box(
            f"floor_sep_e_{floor}",
            f"{sx/2 + 0.018:.3f} 0 {z:.3f} 0 0 0",
            [0.035, sy + 0.1, 0.06],
            sep_color,
        )
        body += visual_box(
            f"floor_sep_w_{floor}",
            f"{-sx/2 - 0.018:.3f} 0 {z:.3f} 0 0 0",
            [0.035, sy + 0.1, 0.06],
            sep_color,
        )
    dpose, dsize = door_pose(b["door_direction"], sx, sy)
    body += visual_box("front_door", dpose, dsize, [0.08, 0.06, 0.04, 1])
    window_color = [0.62, 0.82, 0.95, 1]
    max_rows = min(floors, 6)
    for row in range(1, max_rows + 1):
        z = row * FLOOR_HEIGHT - 1.5
        for x in (-sx * 0.30, 0, sx * 0.30):
            body += visual_box(
                f"win_n_{row}_{x:.1f}",
                f"{x:.3f} {sy/2 + 0.02:.3f} {z:.3f} 0 0 0",
                [1.1, 0.035, 0.75],
                window_color,
            )
            body += visual_box(
                f"win_s_{row}_{x:.1f}",
                f"{x:.3f} {-sy/2 - 0.02:.3f} {z:.3f} 0 0 0",
                [1.1, 0.035, 0.75],
                window_color,
            )
    return f"""
    <model name='{b['model_name']}'>
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>
      <link name='body'>{body}</link>
    </model>
"""


def make_delivery_points(layout: dict) -> str:
    pieces: List[str] = []
    for point in layout.get("delivery_points", []):
        x, y = map(float, point["center"])
        name = safe_name(point["name"])
        base = visual_box("base", "0 0 0 0 0 0", [3.0, 3.0, 0.03], [0.08, 0.08, 0.08, 1])
        marker_x = visual_box("marker_x", "0 0 0.025 0 0 0", [2.1, 0.18, 0.02], [1, 1, 1, 1])
        marker_y = visual_box("marker_y", "0 0 0.030 0 0 0", [0.18, 2.1, 0.02], [1, 1, 1, 1])
        pieces.append(
            f"<link name='{name}'>"
            f"<pose>{x:.3f} {y:.3f} 0.085 0 0 0</pose>"
            f"{base}{marker_x}{marker_y}</link>"
        )
    return "<model name='delivery_points'><static>true</static>" + "".join(pieces) + "</model>"


def make_world(layout: dict, road_paths: dict[str, List[Point]]) -> str:
    buildings = "".join(make_building(b) for b in layout["buildings"])
    return f"""<sdf version='1.7'>
  <world name='campus_delivery'>
    <physics name='default_physics' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <light name='sun' type='directional'>
      <pose>0 0 80 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1</direction>
    </light>
    {make_ground(layout)}
    {make_walls(layout)}
    {make_roads(layout, road_paths)}
    {buildings}
    {make_delivery_points(layout)}
  </world>
</sdf>
"""


def world_to_pixel(
    point: Point,
    origin: Sequence[float],
    resolution: float,
    height_px: int,
) -> Tuple[int, int]:
    x, y = point
    col = int(round((x - origin[0]) / resolution))
    row = int(round(height_px - 1 - (y - origin[1]) / resolution))
    return col, row


def rect_polygon(
    center: Sequence[float],
    size: Sequence[float],
    origin: Sequence[float],
    res: float,
    hpx: int,
):
    cx, cy = map(float, center)
    sx, sy = map(float, size)
    corners = [
        (cx - sx / 2, cy - sy / 2),
        (cx + sx / 2, cy - sy / 2),
        (cx + sx / 2, cy + sy / 2),
        (cx - sx / 2, cy + sy / 2),
    ]
    return [world_to_pixel(p, origin, res, hpx) for p in corners]


def draw_line(draw: ImageDraw.ImageDraw, pts: List[Point], origin, res, hpx, width_m, color):
    pix = [world_to_pixel(p, origin, res, hpx) for p in pts]
    width_px = max(1, int(round(width_m / res)))
    try:
        draw.line(pix, fill=color, width=width_px, joint="curve")
    except TypeError:
        draw.line(pix, fill=color, width=width_px)
    r = width_px // 2
    for p in pix:
        draw.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=color)


def write_pgm(image: Image.Image, path: Path) -> None:
    path.write_bytes(b"P5\n%d %d\n255\n" % image.size + image.tobytes())


def make_maps(layout: dict, road_paths: dict[str, List[Point]], nav_maps: Path) -> None:
    b = layout["world_bounds"]
    res = float(layout["map_resolution"])
    navigation = layout.get("navigation", {})
    edge_margin = float(navigation.get("keepout_edge_margin", 0.0))
    guidance_band_width = float(navigation.get("guidance_band_width", 0.0))
    minimum_drivable_width = float(navigation.get("minimum_drivable_width", 1.0))
    guidance_band_cost = float(navigation.get("guidance_band_cost", 12.0))
    soft_keepout_cost = float(navigation.get("soft_keepout_cost", 35.0))
    if edge_margin < 0.0 or guidance_band_width < 0.0 or minimum_drivable_width <= 0.0:
        raise SystemExit(
            "navigation margins must be non-negative and minimum width must be positive"
        )
    if not 0.0 < guidance_band_cost < soft_keepout_cost < 100.0:
        raise SystemExit(
            "navigation costs must satisfy 0 < guidance band < soft keepout < 100"
        )

    free_thresh = 0.196
    occupied_thresh = 0.65

    def cost_to_pixel(cost: float) -> int:
        occupancy = free_thresh + cost / 100.0 * (occupied_thresh - free_thresh)
        return int(round(255.0 * (1.0 - occupancy)))

    guidance_band_pixel = cost_to_pixel(guidance_band_cost)
    soft_keepout_pixel = cost_to_pixel(soft_keepout_cost)
    width_px = int(round((b["x_max"] - b["x_min"]) / res))
    height_px = int(round((b["y_max"] - b["y_min"]) / res))
    origin = [float(b["x_min"]), float(b["y_min"]), 0.0]
    localization_map = Image.new("L", (width_px, height_px), 205)
    localization_draw = ImageDraw.Draw(localization_map)
    keepout_mask = Image.new("L", (width_px, height_px), 0)
    keepout_draw = ImageDraw.Draw(keepout_mask)

    # Draw the complete paved network first. In the keepout mask this is the
    # recoverable shoulder between the free center lane and lethal off-road area.
    for road in layout["roads"]:
        road_width = float(road["width"])
        draw_line(
            localization_draw,
            road_paths[road["name"]],
            origin,
            res,
            height_px,
            road_width,
            254,
        )
        draw_line(
            keepout_draw,
            road_paths[road["name"]],
            origin,
            res,
            height_px,
            road_width,
            soft_keepout_pixel,
        )
    for area in layout.get("paved_areas", []):
        area_size = [float(value) for value in area["size"]]
        localization_draw.polygon(
            rect_polygon(area["center"], area_size, origin, res, height_px),
            fill=254,
        )
        keepout_draw.polygon(
            rect_polygon(area["center"], area_size, origin, res, height_px),
            fill=soft_keepout_pixel,
        )

    # Apply a low-cost guidance band before the free center. With the passes
    # ordered by decreasing cost, intersecting roads retain a usable center.
    for road in layout["roads"]:
        road_width = float(road["width"])
        guidance_width = min(
            road_width,
            max(minimum_drivable_width, road_width - 2.0 * edge_margin),
        )
        draw_line(
            keepout_draw,
            road_paths[road["name"]],
            origin,
            res,
            height_px,
            guidance_width,
            guidance_band_pixel,
        )
    for area in layout.get("paved_areas", []):
        area_size = [float(value) for value in area["size"]]
        guidance_size = [
            min(size, max(minimum_drivable_width, size - 2.0 * edge_margin))
            for size in area_size
        ]
        keepout_draw.polygon(
            rect_polygon(area["center"], guidance_size, origin, res, height_px),
            fill=guidance_band_pixel,
        )

    # The zero-cost lane gives both the global planner and each replan a stable
    # center target instead of treating the full paved width as equivalent.
    center_margin = edge_margin + guidance_band_width
    for road in layout["roads"]:
        road_width = float(road["width"])
        free_width = min(
            road_width,
            max(minimum_drivable_width, road_width - 2.0 * center_margin),
        )
        draw_line(
            keepout_draw,
            road_paths[road["name"]],
            origin,
            res,
            height_px,
            free_width,
            254,
        )
    for area in layout.get("paved_areas", []):
        area_size = [float(value) for value in area["size"]]
        free_size = [
            min(size, max(minimum_drivable_width, size - 2.0 * center_margin))
            for size in area_size
        ]
        keepout_draw.polygon(
            rect_polygon(area["center"], free_size, origin, res, height_px),
            fill=254,
        )
    for building in layout["buildings"]:
        polygon = rect_polygon(building["center"], building["size"], origin, res, height_px)
        localization_draw.polygon(polygon, fill=0)
        keepout_draw.polygon(polygon, fill=0)

    wall = layout["wall"]
    wall_center_x = (float(wall["west_x"]) + float(wall["east_x"])) / 2
    wall_center_y = (float(wall["south_y"]) + float(wall["north_y"])) / 2
    wall_width = float(wall["east_x"]) - float(wall["west_x"])
    wall_height = float(wall["north_y"]) - float(wall["south_y"])
    wall_thickness = float(wall["thickness"])
    wall_rectangles = [
        ([wall["west_x"], wall_center_y], [wall_thickness, wall_height]),
        ([wall["east_x"], wall_center_y], [wall_thickness, wall_height]),
        ([wall_center_x, wall["north_y"]], [wall_width, wall_thickness]),
    ]
    west_south_length = float(wall["south_gate_x_min"]) - float(wall["west_x"])
    east_south_length = float(wall["east_x"]) - float(wall["south_gate_x_max"])
    wall_rectangles.extend([
        (
            [float(wall["west_x"]) + west_south_length / 2, wall["south_y"]],
            [west_south_length, wall_thickness],
        ),
        (
            [float(wall["south_gate_x_max"]) + east_south_length / 2, wall["south_y"]],
            [east_south_length, wall_thickness],
        ),
    ])
    for center, size in wall_rectangles:
        localization_draw.polygon(
            rect_polygon(center, size, origin, res, height_px),
            fill=0,
        )

    tree_radius_px = max(1, int(round(0.18 / res)))
    for tree in layout.get("park", {}).get("trees", []):
        px, py = world_to_pixel(tree, origin, res, height_px)
        localization_draw.ellipse(
            [
                px - tree_radius_px,
                py - tree_radius_px,
                px + tree_radius_px,
                py + tree_radius_px,
            ],
            fill=0,
        )

    spawn = (float(layout["spawn_pose"]["x"]), float(layout["spawn_pose"]["y"]))
    spx = world_to_pixel(spawn, origin, res, height_px)
    if keepout_mask.getpixel(spx) < 250:
        raise SystemExit(f"spawn point {spawn} is not on free road/platform in generated map")

    nav_maps.mkdir(parents=True, exist_ok=True)
    map_path = nav_maps / "campus_delivery_map.pgm"
    mask_path = nav_maps / "campus_keepout_mask.pgm"
    write_pgm(localization_map, map_path)
    write_pgm(keepout_mask, mask_path)

    def yaml_text(image: str, mode: str) -> str:
        return (
            f"image: {image}\n"
            f"mode: {mode}\n"
            f"resolution: {res}\n"
            f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
            "negate: 0\n"
            f"occupied_thresh: {occupied_thresh}\n"
            f"free_thresh: {free_thresh}\n"
        )

    (nav_maps / "campus_delivery_map.yaml").write_text(
        yaml_text("campus_delivery_map.pgm", "trinary"),
        encoding="utf-8",
    )
    (nav_maps / "campus_keepout_mask.yaml").write_text(
        yaml_text("campus_keepout_mask.pgm", "scale"),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    here = Path(__file__).resolve()
    default_desc_root = here.parents[1]
    parser.add_argument(
        "--layout",
        default=str(default_desc_root / "config" / "campus_layout.yaml"),
    )
    parser.add_argument("--description-root", default=str(default_desc_root))
    parser.add_argument(
        "--navigation-root",
        default=str(default_desc_root.parent / "ugvcar_navigation2"),
    )
    args = parser.parse_args()

    layout_path = Path(args.layout)
    desc_root = Path(args.description_root)
    nav_root = Path(args.navigation_root)
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    road_paths = {
        road["name"]: smooth_path(road["centerline"], 0.9)
        for road in layout["roads"]
    }
    validate_layout(layout, road_paths)

    world_path = desc_root / "world" / "campus_delivery.world"
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(make_world(layout, road_paths), encoding="utf-8")
    make_maps(layout, road_paths, nav_root / "maps")

    bounds = layout["world_bounds"]
    res = layout["map_resolution"]
    print("Generated campus_delivery scene")
    print(f"  buildings: {len(layout['buildings'])}")
    print(f"  roads: {len(layout['roads'])}")
    print(f"  trees: {len(layout.get('park', {}).get('trees', []))}")
    print(f"  world: {world_path}")
    width = int((bounds["x_max"] - bounds["x_min"]) / res)
    height = int((bounds["y_max"] - bounds["y_min"]) / res)
    print(f"  map size: {width} x {height} px")
    sp = layout["spawn_pose"]
    print(f"  UGV spawn: x={sp['x']} y={sp['y']} z={sp['z']} yaw={sp['yaw']}")


if __name__ == "__main__":
    main()
