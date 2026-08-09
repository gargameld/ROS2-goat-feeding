"""Translate arena footprint polygons into a Nav2 occupancy map."""

import math
from dataclasses import dataclass


FREE = 254
OCCUPIED = 0


@dataclass(frozen=True)
class MapInfo:
    """Occupancy-grid size, origin, and resolution."""

    width: int
    height: int
    origin_x: float
    origin_y: float
    resolution: float


def _point_in_polygon(point, polygon):
    """Return whether a point is inside a polygon."""
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            edge_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x <= edge_x:
                inside = not inside
        previous = current
    return inside


def _aligned_range(values, resolution, margin):
    """Return resolution-aligned bounds for one map axis."""
    minimum = math.floor((min(values) - margin) / resolution) * resolution
    maximum = math.ceil((max(values) + margin) / resolution) * resolution
    return minimum, maximum


def _map_info(polygons, resolution, margin):
    """Calculate map dimensions from the combined arena polygons."""
    points = [point for polygon in polygons for point in polygon]
    if not points:
        raise ValueError("No arena geom intersects the requested height")
    origin_x, max_x = _aligned_range(
        [point[0] for point in points], resolution, margin
    )
    origin_y, max_y = _aligned_range(
        [point[1] for point in points], resolution, margin
    )
    return MapInfo(
        width=round((max_x - origin_x) / resolution),
        height=round((max_y - origin_y) / resolution),
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
    )


def _cell_center(info, map_x, map_y):
    """Return one map cell's center in world coordinates."""
    x = info.origin_x + (map_x + 0.5) * info.resolution
    y = info.origin_y + (map_y + 0.5) * info.resolution
    return x, y


def _occupied(point, polygons):
    """Return whether any arena polygon contains the point."""
    return any(_point_in_polygon(point, polygon) for polygon in polygons)


def _rasterize(polygons, info):
    """Translate combined polygons into PGM pixel values."""
    pixels = bytearray()
    for image_y in range(info.height):
        map_y = info.height - 1 - image_y
        for map_x in range(info.width):
            point = _cell_center(info, map_x, map_y)
            value = OCCUPIED if _occupied(point, polygons) else FREE
            pixels.append(value)
    return pixels


def _write_pgm(output_path, pixels, info, height):
    """Write the occupancy image."""
    header = (
        f"P5\n# MuJoCo arena cross-section at z={height:g} m\n"
        f"{info.width} {info.height}\n255\n"
    )
    output_path.write_bytes(header.encode("ascii") + pixels)


def _write_yaml(output_path, info):
    """Write the ROS map metadata."""
    output_path.with_suffix(".yaml").write_text(
        f"image: {output_path.name}\n"
        "mode: trinary\n"
        f"resolution: {info.resolution:g}\n"
        f"origin: [{info.origin_x:g}, {info.origin_y:g}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )


def polygons_to_map(polygons, output_path, height, resolution, margin):
    """Translate combined arena polygons into PGM and YAML map files."""
    info = _map_info(polygons, resolution, margin)
    pixels = _rasterize(polygons, info)
    _write_pgm(output_path, pixels, info, height)
    _write_yaml(output_path, info)
    return info
