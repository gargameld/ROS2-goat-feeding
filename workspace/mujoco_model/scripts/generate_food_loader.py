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

The contact parameters live in a single ``<default class="food">`` block at the
top of the generated file and every food geom inherits from it, so the whole
set can be retuned in one place. See FOOD_* below for why each value is what it
is -- several of them are counter-intuitive and were measured, not guessed.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path
from typing import Callable, NamedTuple
from xml.dom import minidom
from xml.etree import ElementTree as ET

# Directory of this script -> mujoco_model/ is its parent.
SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_DIR = SCRIPT_DIR.parent

# Contact parameters applied to every generated food geom, emitted as a single
# <default class="food"> block that every food geom inherits from.
#
# condim 6 allocates the torsional and rolling friction constraints on top of
# the usual sliding ones. Sliding friction alone cannot stop a settled item
# spinning about the contact normal (slip velocity on the spin axis is zero, so
# there is nothing for it to resist) and cannot stop a rounded mesh rolling off
# a shelf. Torsional friction handles the first, rolling friction the second,
# and neither constraint exists below condim 4 / 6 respectively.
#
# The friction coefficients are deliberately ordinary, and that is the fix for
# "the food slides out of the gripper even though I set sliding friction to
# 10000". In MuJoCo a large coefficient does not mean a strong grip: with
# cone="elliptic" (scene.xml) the friction cone's tangential axes are scaled by
# 1/mu, so as mu grows the tangential directions carry vanishing weight in the
# solver and the friction impulse rounds towards zero. The contact ends up
# *less* able to hold than a plain mu=1 one. Measured on the cone mesh, clamped
# in the jaws, ramping a pull-out force until the item moves 5 mm or turns 10
# degrees:
#
#     mu:        0.5     1      2      5     10     100    1000   10000
#     holds to:  >100N  >100N  >100N  >100N  >100N  0.09N  0.02N  0.016N
#
# Everything up to mu=10 survived the full 100 N ramp; past mu~50 the grip
# collapses to nothing. (That sweep was run on a pip build of MuJoCo, which is
# several releases ahead of the vendored 3.4.0 the simulation loads. The cone
# scaling it measures is a property of the elliptic cone and holds on both;
# the contact-manifold behaviour above is the part that does not, so anything
# about resting or grasp *stability* has to be measured on 3.4.0.) The same mechanism applies to the torsional and rolling
# coefficients, which carry units of length: 10 (metres!) is not "very grippy",
# it is degenerate. Sane magnitudes for a 5-10 cm item are a contact-patch
# radius for torsional and well under a millimetre-equivalent for rolling.
#
# priority 0 hands every grasp contact to the gripper jaw pads, which carry
# priority 1 (gripper.xml). MuJoCo resolves a contact pair by priority: the
# higher-priority geom's condim, friction, solref and solimp are used outright,
# and only at *equal* priority are the two combined (condim = max, friction =
# elementwise max, solref/solimp mixed). The pads are the tuned, compliant
# surface -- their solref/solimp are what make the grasp hold -- so they should
# win. Overriding them is actively harmful: at priority 2 the food's own
# solref/solimp replace the pads' and the same cone lets go at 0.002 N.
# Against the shelf and floor (priority 0, MuJoCo defaults) the priorities are
# equal, so the parameters combine and the food's larger values win the
# elementwise max anyway -- it gets condim 6 and its own friction there.
#
# The rolling coefficient is 0.001 rather than the 0.002 it started at, and
# that one digit is worth explaining, because it is the difference between a
# box that sits still on a shelf and one that does not. The engine this model
# runs under is the vendored MuJoCo 3.4.0, and the contact a mesh makes with a
# shelf there is a *single* point unless scene.xml enables multiccd (it now
# does; see the comment on the flag). Torsional and rolling friction resolved
# at a single point are ill conditioned: the solver leaves a small residual
# every step, the residual is always in the same direction, and the item yaws
# in place forever. It scales with the rolling coefficient, and it is not a
# smooth function of it, which is what makes it look like a physics bug rather
# than a tuning problem. Creep measured 30 s after each item settles:
#
#     rolling:        0.002    0.001    0.0002
#     single point:  21.5 deg  4.4 deg   0.3 deg     <- multiccd off
#     full manifold:  2.1 deg  0.01 deg  0.04 deg    <- multiccd on
#
# multiccd is the real fix and it flattens four of the five items to 0.00 deg.
# The elipsoid keeps one contact point even so, because a genuinely curved
# surface touching a plane *is* one point, and it is the only item that still
# responds to this coefficient at all. 0.001 m is where it settles quietly and
# is a sane number in its own right: a millimetre of rolling lever arm on a
# 5 cm item.
FOOD_CONDIM = 6
FOOD_FRICTION = "1.0 0.02 0.001"  # sliding, torsional (m), rolling (m)
FOOD_PRIORITY = 0

# Contact softness. The MuJoCo default solref "0.02 1" is a 20 ms time constant,
# which at the 1 ms timestep lets a dropped item sink visibly into the shelf
# before the constraint pushes back. 5 ms is stiff enough to look rigid and
# still an order of magnitude above the timestep, so it stays stable. solimp
# ramps the constraint impedance from 0.95 to 0.99 over 1 mm of penetration:
# nearly rigid on contact, with a narrow soft band that keeps the solver
# well-conditioned at the moment of touchdown.
FOOD_SOLREF = "0.005 1"
FOOD_SOLIMP = "0.95 0.99 0.001"

# Uniform density in kg/m^3 used to derive each item's mass from its mesh
# volume. 1000 (water) is MuJoCo's own default and a fair stand-in for food;
# it is written out explicitly so the value is visible in the generated XML.
FOOD_DENSITY = 1000.0

# Inertia mode for the <mesh> assets. MuJoCo *collides* a mesh as its convex
# hull no matter what, but it does not have to take the mass and inertia from
# that hull. "exact" integrates over the actual triangles, which matters for
# any item with a hole or a concavity: the ring is a torus whose hull is a
# solid disc, so the default hull-based inertia hands it 0.318 kg instead of
# its true 0.207 kg -- a 54% error in every contact force it takes part in.
# Falls back to "convex" for meshes that are not closed solids (see
# measure_mesh), because "exact" needs a watertight surface to integrate.
FOOD_MESH_INERTIA = "exact"
FOOD_MESH_INERTIA_FALLBACK = "convex"

# Name of the generated <default> class that carries all of the above.
FOOD_DEFAULT_CLASS = "food"

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


class MeshInfo(NamedTuple):
    """Geometry of one STL, measured in the file's own (unscaled) coordinates."""

    com: tuple[float, float, float]  # volumetric centroid
    min_z: float  # lowest vertex, for the spawn height
    volume: float  # signed volume; <= 0 means the surface is not a closed solid
    convex: bool  # False only when a concavity was actually found


def measure_mesh(stl_path: Path) -> MeshInfo:
    """Measure the centroid, lowest point, volume and convexity of a mesh.

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
    return MeshInfo(
        com=com,
        min_z=(min_z if min_z != float("inf") else 0.0),
        volume=total_v,
        convex=is_probably_convex(tris),
    )


def is_probably_convex(tris, sample_cap: int = 400, rel_tol: float = 1e-4) -> bool:
    """Return False only if a vertex was found on the outer side of a face plane.

    MuJoCo always collides a mesh geom as its *convex hull*, so a concave item
    (the torus-shaped ``ring``, say) physically behaves as the solid it is
    wrapped in: the gripper cannot reach into the hole and the shelf cannot
    touch the underside. That is worth telling the user about, but it is only a
    warning, so a cheap conservative test is enough. Each face plane is checked
    against an evenly strided sample of at most *sample_cap* vertices; sampling
    can only miss a violation, never invent one, so a False result means a real
    concavity while a True result only means "no concavity found".

    The tolerance is *rel_tol* times the bounding-box diagonal rather than an
    absolute distance. STL stores single-precision coordinates, so on a 100 mm
    tessellated sphere neighbouring facet planes disagree by ~1e-3 mm purely
    from rounding; an absolute micrometre tolerance would call every curved
    surface concave.
    """
    verts = [v for tri in tris for v in tri]
    if not verts:
        return True
    span = [max(v[axis] for v in verts) - min(v[axis] for v in verts) for axis in range(3)]
    diag = (span[0] ** 2 + span[1] ** 2 + span[2] ** 2) ** 0.5
    tol = rel_tol * diag if diag > 0.0 else rel_tol
    stride = max(1, len(verts) // sample_cap)
    sample = verts[::stride]
    for a, b, c in tris:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        norm = (nx * nx + ny * ny + nz * nz) ** 0.5
        if norm <= 0.0:  # degenerate sliver, no usable plane
            continue
        nx, ny, nz = nx / norm, ny / norm, nz / norm
        for px, py, pz in sample:
            if (px - a[0]) * nx + (py - a[1]) * ny + (pz - a[2]) * nz > tol:
                return False
    return True


def build_include(
    stl_files: list[Path],
    mesh_ref_dir: str,
    scale: float,
    start: tuple[float, float],
    spacing: float,
    clearance: float,
    condim: int = FOOD_CONDIM,
    friction: str = FOOD_FRICTION,
    priority: int = FOOD_PRIORITY,
    solref: str = FOOD_SOLREF,
    solimp: str = FOOD_SOLIMP,
    density: float = FOOD_DENSITY,
    body_prefix: str = FOOD_BODY_PREFIX,
    warn: Callable[[str], None] = lambda message: None,
) -> ET.ElementTree:
    """Build the <mujocoinclude> element tree.

    Each body gets a freejoint so it is a movable rigid body, and is placed on
    a line starting at *start* (x, y), separated by *spacing* metres along +x.
    Body names are *body_prefix* + the STL stem so the runtime can find them by
    prefix; the mesh, geom and joint keep the bare stem.

    The contact parameters are written once into a ``<default class="food">``
    block and every geom inherits from it, so retuning the whole set is a
    one-line edit in the generated file (or a re-run with the matching flags).

    The geom is offset by ``-com`` so the body's origin (pos 0 0 0) coincides
    with the mesh's center of mass. Each body's z is then chosen so the mesh's
    lowest point rests *clearance* metres above the floor plane (z=0), so
    nothing spawns sunk into or floating above the ground and then falls.

    *warn* is called with a human-readable message for every mesh that cannot
    take the exact-inertia path or that MuJoCo will collide as something other
    than its own shape.
    """
    root = ET.Element("mujocoinclude")

    # One <default> class carrying every contact parameter. MuJoCo merges
    # top-level <default> sections across included files, so this sits happily
    # alongside the "robot" class scene.xml defines.
    defaults = ET.SubElement(root, "default")
    food_class = ET.SubElement(defaults, "default", {"class": FOOD_DEFAULT_CLASS})
    ET.SubElement(
        food_class,
        "geom",
        type="mesh",
        condim=str(condim),
        friction=friction,
        priority=str(priority),
        solref=solref,
        solimp=solimp,
        density=f"{density:g}",
        rgba="0.85 0.6 0.2 1",
    )

    asset = ET.SubElement(root, "asset")
    worldbody = ET.SubElement(root, "worldbody")

    scale_str = f"{scale} {scale} {scale}"
    x0, y0 = start

    for i, stl in enumerate(stl_files):
        name = stl.stem  # mesh/geom/joint name == file name (without extension)
        mesh_file = f"{mesh_ref_dir}/{stl.name}"
        info = measure_mesh(stl)

        # Exact inertia integrates over the triangles and needs a watertight
        # surface; a non-positive signed volume means this one is not closed
        # (or is inside-out), so fall back to the convex hull for it.
        if info.volume > 0.0:
            inertia = FOOD_MESH_INERTIA
        else:
            inertia = FOOD_MESH_INERTIA_FALLBACK
            warn(
                f"{stl.name}: signed volume is {info.volume:g}, so the surface is not a "
                f"closed solid; using inertia=\"{inertia}\" and a CoM at the mesh origin."
            )
        if not info.convex:
            warn(
                f"{stl.name}: mesh is concave. MuJoCo collides mesh geoms as their convex "
                f"hull, so this item will contact the world as the solid shape wrapped "
                f"around it. Its mass and inertia stay correct via inertia=\"{inertia}\"; "
                f"split the STL into convex parts if the concavity has to be graspable."
            )

        ET.SubElement(asset, "mesh", name=name, file=mesh_file, scale=scale_str, inertia=inertia)

        # MuJoCo places the mesh at its raw file coordinates (offset by the geom
        # pos). Shift the geom by -com so the body origin lands on the CoM.
        cx, cy, cz = (c * scale for c in info.com)
        geom_pos = f"{-cx:g} {-cy:g} {-cz:g}"

        # With the geom shifted by -com, the mesh's lowest point is at
        # (min_z - com_z)*scale below the body origin; lift so it clears the floor.
        z = clearance - (info.min_z * scale - cz)
        pos = f"{x0 + i * spacing:g} {y0:g} {z:g}"
        body = ET.SubElement(worldbody, "body", name=f"{body_prefix}{name}", pos=pos)
        ET.SubElement(body, "freejoint", name=f"{name}_free")
        ET.SubElement(
            body,
            "geom",
            {"name": name, "class": FOOD_DEFAULT_CLASS, "mesh": name, "pos": geom_pos},
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
        help="Sliding, torsional and rolling friction of the food geoms. Keep "
        "the sliding coefficient in the ordinary 0.5-2 range: with the "
        "elliptic friction cone the scene uses, coefficients above roughly 50 "
        "make the contact numerically degenerate and the grip collapses to "
        "nothing. The torsional and rolling terms have units of length.",
    )
    parser.add_argument(
        "--priority",
        type=int,
        default=FOOD_PRIORITY,
        help="Contact priority of the food geoms. The default is below the "
        "gripper jaw pads, so the pads' tuned friction and solref/solimp own "
        "every grasp contact outright, while shelf and floor contacts (equal "
        "priority) combine and the food's own values win the elementwise max. "
        "Raising it above the pads replaces their solver parameters with these "
        "and measurably weakens the grasp.",
    )
    parser.add_argument(
        "--solref",
        default=FOOD_SOLREF,
        help="Contact solver reference (time constant, damping ratio) of the "
        "food geoms. Keep the time constant at least a few times the "
        "simulation timestep.",
    )
    parser.add_argument(
        "--solimp",
        default=FOOD_SOLIMP,
        help="Contact solver impedance (dmin, dmax, width) of the food geoms.",
    )
    parser.add_argument(
        "--density",
        type=float,
        default=FOOD_DENSITY,
        help="Uniform density (kg/m^3) used to derive each item's mass from "
        "its mesh volume.",
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
        priority=args.priority,
        solref=args.solref,
        solimp=args.solimp,
        density=args.density,
        body_prefix=args.body_prefix,
        warn=lambda message: print(f"WARNING: {message}", file=sys.stderr),
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
