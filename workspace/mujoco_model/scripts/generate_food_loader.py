#!/usr/bin/env python3
"""Generate a MuJoCo include file with one free body per food STL.

The script scans a directory of STL meshes (default:
``mujoco_model/food_items_stl_files``) and writes a ``<mujocoinclude>`` XML
file (default: ``mujoco_model/food_loader.xml``). For every ``*.stl`` file it
emits:

  * a ``<mesh>`` asset whose name is the STL file stem, and
  * a free-floating ``<body>`` named ``food_<stem>``, placed far away from the
    robot/arena so the food items do not interfere with the scene at startup.

It also inserts ``<include file="food_loader.xml"/>`` into ``scene.xml`` (right
after the ``robot.xml`` include) unless it is already present, so the scene
loads the generated file automatically.

Mesh, geom and joint names are taken verbatim from the STL file names, but the
body name carries the ``food_`` prefix, e.g. ``box.stl -> body "food_box"``.
The prefix is not cosmetic: the simulation GUI and the state-capture plugin
discover the food items by scanning MuJoCo body names for it
(``food_body_prefix``), so an unprefixed body is invisible to both.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

# Directory of this script -> mujoco_model/ is its parent.
SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_DIR = SCRIPT_DIR.parent

# Contact parameters applied to every generated food geom.
#
# condim 6 is required, not cosmetic. An arbitrary convex mesh resting flat on a
# primitive collides through MuJoCo's convex-convex (MPR) path, which returns a
# single contact point, and that point lies on the object's own spin axis. Slip
# velocity there is zero, so sliding friction cannot resist rotation about the
# contact normal -- no matter what the sliding coefficient is. Torsional friction
# resists it, and rolling friction stops round meshes rolling away, but neither
# constraint is allocated below condim 4 / 6 respectively. At MuJoCo's default
# condim of 3 a dropped item settles and then spins forever at whatever rate the
# landing impact left it with.
#
# Deliberately no priority attribute: at equal priority MuJoCo combines pairs by
# condim = max and friction = elementwise max, so these values win over the
# shelf and floor geoms while the gripper pads (priority 1) keep full ownership
# of grasp contacts.
FOOD_CONDIM = 6
FOOD_FRICTION = "1 0.02 0.002"  # sliding, torsional, rolling

# Prefix prepended to every generated body name. Must match the
# `food_body_prefix` parameter of the state-capture plugin and of the
# simulation GUI's MuJoCo client -- both enumerate the food items by scanning
# body names for it.
FOOD_BODY_PREFIX = "food_"


def find_stl_files(stl_dir: Path) -> list[Path]:
    """Return STL files in *stl_dir* sorted by name."""
    files = sorted(p for p in stl_dir.iterdir() if p.suffix.lower() == ".stl")
    if not files:
        raise FileNotFoundError(f"No .stl files found in {stl_dir}")
    return files


def read_stl_triangles(stl_path: Path) -> list[tuple[tuple[float, float, float], ...]]:
    """Return a mesh's triangles as ((ax,ay,az),(bx,by,bz),(cx,cy,cz)) tuples.

    Handles both binary and ASCII STL.
    """
    data = stl_path.read_bytes()

    # Binary STL: 80-byte header, uint32 triangle count, 50 bytes/triangle.
    if len(data) >= 84:
        (n_tri,) = struct.unpack_from("<I", data, 80)
        if len(data) == 84 + n_tri * 50:
            tris = []
            offset = 84
            for _ in range(n_tri):
                # 12 floats per triangle: normal (3) + 3 vertices (9).
                v = struct.unpack_from("<12f", data, offset)
                tris.append((v[3:6], v[6:9], v[9:12]))
                offset += 50
            return tris

    # ASCII STL fallback: gather "vertex x y z" lines in groups of three.
    verts = []
    for line in data.decode("ascii", errors="ignore").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            verts.append(tuple(float(x) for x in parts[1:4]))
    return [tuple(verts[i : i + 3]) for i in range(0, len(verts) - 2, 3)]


def mesh_com_and_min_z(stl_path: Path) -> tuple[tuple[float, float, float], float]:
    """Return (center_of_mass, min_z) of a mesh in its raw file coordinates.

    The center of mass is the *volumetric* centroid of the closed surface,
    assuming uniform density (signed-tetrahedron method). This matches the CoM
    MuJoCo computes and places the body's inertial frame on, so offsetting a
    geom by ``-com`` makes the body origin coincide with the center of mass.
    """
    tris = read_stl_triangles(stl_path)
    total_v = 0.0
    cx = cy = cz = 0.0
    min_z = float("inf")
    for a, b, c in tris:
        # Signed volume of the tetrahedron (origin, a, b, c).
        vol = (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        ) / 6.0
        total_v += vol
        cx += vol * (a[0] + b[0] + c[0]) / 4.0
        cy += vol * (a[1] + b[1] + c[1]) / 4.0
        cz += vol * (a[2] + b[2] + c[2]) / 4.0
        min_z = min(min_z, a[2], b[2], c[2])

    if total_v == 0.0:  # degenerate / non-closed mesh: fall back to origin.
        com = (0.0, 0.0, 0.0)
    else:
        com = (cx / total_v, cy / total_v, cz / total_v)
    return com, (min_z if min_z != float("inf") else 0.0)


def build_include(
    stl_files: list[Path],
    mesh_ref_dir: str,
    scale: float,
    start: tuple[float, float],
    spacing: float,
    clearance: float,
    condim: int = FOOD_CONDIM,
    friction: str = FOOD_FRICTION,
    body_prefix: str = FOOD_BODY_PREFIX,
) -> ET.ElementTree:
    """Build the <mujocoinclude> element tree.

    Each body gets a freejoint so it is a movable rigid body, and is placed on
    a line starting at *start* (x, y), separated by *spacing* metres along +x.
    Body names are *body_prefix* + the STL stem so the runtime can find them by
    prefix; the mesh, geom and joint keep the bare stem.

    The geom is offset by ``-com`` so the body's origin (pos 0 0 0) coincides
    with the mesh's center of mass. Each body's z is then chosen so the mesh's
    lowest point rests *clearance* metres above the floor plane (z=0), so
    nothing spawns sunk into or floating above the ground and then falls.
    """
    root = ET.Element("mujocoinclude")

    asset = ET.SubElement(root, "asset")
    worldbody = ET.SubElement(root, "worldbody")

    scale_str = f"{scale} {scale} {scale}"
    x0, y0 = start

    for i, stl in enumerate(stl_files):
        name = stl.stem  # mesh/geom/joint name == file name (without extension)
        mesh_file = f"{mesh_ref_dir}/{stl.name}"

        ET.SubElement(asset, "mesh", name=name, file=mesh_file, scale=scale_str)

        # MuJoCo places the mesh at its raw file coordinates (offset by the geom
        # pos). Shift the geom by -com so the body origin lands on the CoM.
        com, min_z = mesh_com_and_min_z(stl)
        cx, cy, cz = (c * scale for c in com)
        geom_pos = f"{-cx:g} {-cy:g} {-cz:g}"

        # With the geom shifted by -com, the mesh's lowest point is at
        # (min_z - com_z)*scale below the body origin; lift so it clears the floor.
        z = clearance - (min_z * scale - cz)
        pos = f"{x0 + i * spacing:g} {y0:g} {z:g}"
        body = ET.SubElement(worldbody, "body", name=f"{body_prefix}{name}", pos=pos)
        ET.SubElement(body, "freejoint", name=f"{name}_free")
        ET.SubElement(
            body,
            "geom",
            name=name,
            type="mesh",
            mesh=name,
            pos=geom_pos,
            condim=str(condim),
            friction=friction,
            rgba="0.85 0.6 0.2 1",
        )

    return ET.ElementTree(root)


def write_pretty(tree: ET.ElementTree, out_path: Path) -> None:
    """Write *tree* to *out_path* with indentation."""
    rough = ET.tostring(tree.getroot(), encoding="unicode")
    pretty = minidom.parseString(rough).toprettyxml(indent="  ")
    # Drop the <?xml ...?> line minidom adds; MuJoCo include files don't need it.
    lines = [ln for ln in pretty.splitlines() if ln.strip() and not ln.startswith("<?xml")]
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def ensure_scene_include(scene_path: Path, include_name: str) -> str:
    """Ensure scene.xml includes *include_name*. Returns a status message."""
    if not scene_path.exists():
        return f"WARNING: {scene_path} not found; add <include file=\"{include_name}\"/> manually."

    text = scene_path.read_text(encoding="utf-8")
    include_line = f'<include file="{include_name}"/>'
    if include_line in text:
        return f"scene.xml already includes {include_name}."

    robot_include = '<include file="robot.xml"/>'
    if robot_include in text:
        # Match the indentation of the robot include and insert right after it.
        idx = text.index(robot_include)
        line_start = text.rfind("\n", 0, idx) + 1
        indent = text[line_start:idx]
        text = text.replace(
            robot_include,
            f"{robot_include}\n{indent}{include_line}",
            1,
        )
    else:
        # Fall back to inserting right after the opening <mujoco> tag.
        marker = "<mujoco>"
        if marker not in text:
            return f"WARNING: could not locate insertion point; add {include_line} to scene.xml manually."
        idx = text.index(marker) + len(marker)
        text = text[:idx] + f"\n  {include_line}" + text[idx:]

    scene_path.write_text(text, encoding="utf-8")
    return f"Added {include_line} to scene.xml."


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--stl-dir",
        type=Path,
        default=MODEL_DIR / "food_items_stl_files",
        help="Directory containing the food STL files.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=MODEL_DIR / "food_loader.xml",
        help="Path of the generated MuJoCo include file.",
    )
    parser.add_argument(
        "--scene",
        type=Path,
        default=MODEL_DIR / "scene.xml",
        help="Scene file to add the <include> to.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=0.001,
        help="Uniform mesh scale (STLs are in millimetres, so 0.001 -> metres).",
    )
    parser.add_argument(
        "--start",
        type=float,
        nargs=2,
        metavar=("X", "Y"),
        default=(50.0, 50.0),
        help="X, Y of the first food body (far from the robot); z is computed from the mesh.",
    )
    parser.add_argument("--spacing", type=float, default=1.0, help="Gap between consecutive bodies along +x.")
    parser.add_argument(
        "--clearance",
        type=float,
        default=0.005,
        help="Gap (m) between each mesh's lowest point and the floor at spawn.",
    )
    parser.add_argument(
        "--condim",
        type=int,
        choices=(1, 3, 4, 6),
        default=FOOD_CONDIM,
        help="Contact dimensionality of the food geoms. Below 4 there is no "
        "torsional friction and settled items spin about the contact normal "
        "forever; below 6 no rolling friction.",
    )
    parser.add_argument(
        "--friction",
        default=FOOD_FRICTION,
        help="Sliding, torsional and rolling friction of the food geoms.",
    )
    parser.add_argument(
        "--body-prefix",
        default=FOOD_BODY_PREFIX,
        help="Prefix prepended to every body name. Must match the "
        "`food_body_prefix` parameter used by the state-capture plugin and the "
        "simulation GUI, which discover the food items by this prefix.",
    )
    parser.add_argument(
        "--no-scene-include",
        action="store_true",
        help="Do not modify scene.xml (only generate the include file).",
    )
    args = parser.parse_args(argv)

    stl_dir = args.stl_dir.resolve()
    out_path = args.output.resolve()

    try:
        stl_files = find_stl_files(stl_dir)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 1

    # Mesh file paths inside the include are resolved by MuJoCo relative to the
    # model directory, so express them relative to the include file's folder.
    mesh_ref_dir = Path(stl_dir).relative_to(out_path.parent).as_posix()

    tree = build_include(
        stl_files,
        mesh_ref_dir=mesh_ref_dir,
        scale=args.scale,
        start=tuple(args.start),
        spacing=args.spacing,
        clearance=args.clearance,
        condim=args.condim,
        friction=args.friction,
        body_prefix=args.body_prefix,
    )
    write_pretty(tree, out_path)
    body_names = ", ".join(f"{args.body_prefix}{p.stem}" for p in stl_files)
    print(f"Wrote {out_path} with {len(stl_files)} food bodies: {body_names}")

    if not args.no_scene_include:
        include_name = out_path.relative_to(args.scene.resolve().parent).as_posix()
        print(ensure_scene_include(args.scene.resolve(), include_name))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
