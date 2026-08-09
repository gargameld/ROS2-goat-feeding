"""Extract simple 2D footprints from the MuJoCo arena."""

import xml.etree.ElementTree as element_tree

import mujoco


IGNORED_MODEL_SECTIONS = {
    "extension", "include", "contact", "tendon", "equality", "actuator",
    "sensor", "keyframe",
}


def _remove_robot_sections(root):
    """Remove model sections that are not needed for the static arena."""
    for child in list(root):
        if child.tag in IGNORED_MODEL_SECTIONS:
            root.remove(child)


def _remove_robot_meshes(root):
    """Remove mesh assets referenced only by the omitted robot."""
    asset = root.find("asset")
    if asset is None:
        return
    for mesh in asset.findall("mesh"):
        asset.remove(mesh)


def _keep_arena_bodies(root, arena_body_names):
    """Keep only the requested top-level arena bodies."""
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("The MJCF model does not contain a worldbody")
    arena_body_names = set(arena_body_names)
    for body in worldbody.findall("body"):
        if body.get("name") not in arena_body_names:
            worldbody.remove(body)


def load_model(model_path, arena_body_names):
    """Load an arena-only model without MuJoCo extensions."""
    root = element_tree.parse(model_path).getroot()
    _remove_robot_sections(root)
    _remove_robot_meshes(root)
    _keep_arena_bodies(root, arena_body_names)
    xml = element_tree.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml)


def _is_arena_box(model, geom_id):
    """Return whether a geom is a box attached outside the world body."""
    has_body = model.geom_bodyid[geom_id] != 0
    is_box = model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX
    return has_body and is_box


def arena_geoms(model):
    """Return the box geoms attached to arena bodies."""
    return [
        geom_id
        for geom_id in range(model.ngeom)
        if _is_arena_box(model, geom_id)
    ]


def _is_upright(rotation, tolerance=1.0e-9):
    """Return whether a geom's local Z axis is parallel to world Z."""
    x_is_zero = abs(rotation[0, 2]) <= tolerance
    y_is_zero = abs(rotation[1, 2]) <= tolerance
    z_is_one = abs(abs(rotation[2, 2]) - 1.0) <= tolerance
    return x_is_zero and y_is_zero and z_is_one


def footprint_polygon(model, data, geom_id, height):
    """Return an upright box's XY footprint when it intersects ``height``."""
    center = data.geom_xpos[geom_id]
    half_size = model.geom_size[geom_id]
    bottom = center[2] - half_size[2]
    top = center[2] + half_size[2]
    if not bottom <= height <= top:
        return []

    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    if not _is_upright(rotation):
        raise ValueError("Arena boxes must be upright")

    local_corners = (
        (-half_size[0], -half_size[1], 0.0),
        (half_size[0], -half_size[1], 0.0),
        (half_size[0], half_size[1], 0.0),
        (-half_size[0], half_size[1], 0.0),
    )
    world_corners = [center + rotation @ corner for corner in local_corners]
    return [(float(corner[0]), float(corner[1])) for corner in world_corners]


def footprint_polygons(model, geom_ids, height):
    """Calculate one footprint polygon for every arena geom."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return [
        footprint_polygon(model, data, geom_id, height)
        for geom_id in geom_ids
    ]


def combine_polygons(polygons):
    """Combine visible geom footprints into one arena polygon collection."""
    return [polygon for polygon in polygons if polygon]
