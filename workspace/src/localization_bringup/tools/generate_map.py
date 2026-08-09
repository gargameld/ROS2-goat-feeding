#!/usr/bin/env python3
"""Build a ROS occupancy map from a MuJoCo arena cross-section."""

import argparse
from pathlib import Path

from arena_geometry import (
    arena_geoms,
    combine_polygons,
    footprint_polygons,
    load_model,
)

from occupancy_map import polygons_to_map


DEFAULT_HEIGHT = 0.46
DEFAULT_RESOLUTION = 0.05
DEFAULT_MARGIN = 1.0
DEFAULT_ARENA_BODIES = ("quoridor", "parkings")


def default_paths():
    """Return the source model and generated-map paths."""
    package_path = Path(__file__).resolve().parents[1]
    source_path = package_path.parent
    model_path = source_path / "robot_description" / "mjcf" / "scene.xml"
    output_path = package_path / "maps" / "mujoco_arena.pgm"
    return model_path, output_path


def parse_arguments():
    """Parse command-line arguments."""
    default_model, default_output = default_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=default_model)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--height", type=float, default=DEFAULT_HEIGHT)
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    parser.add_argument("--margin", type=float, default=DEFAULT_MARGIN)
    parser.add_argument(
        "--arena-body",
        nargs="+",
        default=DEFAULT_ARENA_BODIES,
        help="Root body names whose descendant geoms compose the arena",
    )
    return parser.parse_args()


def prepare_output_path(path):
    """Resolve the output path and create its parent directory."""
    output_path = path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def load_arena(model_path, arena_bodies):
    """Load the requested arena bodies from the MuJoCo scene."""
    return load_model(model_path.resolve(), arena_bodies)


def write_occupancy_map(polygons, output_path, height, resolution, margin):
    """Translate combined polygons into occupancy-map files."""
    return polygons_to_map(
        polygons=polygons,
        output_path=output_path,
        height=height,
        resolution=resolution,
        margin=margin,
    )


def print_summary(arguments, geom_ids, polygons, map_info):
    """Print a short generation summary."""
    print(
        f"Loaded {len(geom_ids)} arena boxes; "
        f"{len(polygons)} intersect z={arguments.height:g} m"
    )
    print(
        f"Wrote {map_info.width}x{map_info.height} map at "
        f"origin ({map_info.origin_x:g}, {map_info.origin_y:g}): "
        f"{arguments.output.resolve()}"
    )


def main():
    """Coordinate the map-generation pipeline."""
    arguments = parse_arguments()
    output_path = prepare_output_path(arguments.output)
    model = load_arena(arguments.model, arguments.arena_body)
    geom_ids = arena_geoms(model)
    polygons = footprint_polygons(model, geom_ids, arguments.height)
    arena_polygons = combine_polygons(polygons)
    map_info = write_occupancy_map(
        arena_polygons,
        output_path,
        arguments.height,
        arguments.resolution,
        arguments.margin,
    )
    print_summary(arguments, geom_ids, arena_polygons, map_info)


if __name__ == "__main__":
    main()
