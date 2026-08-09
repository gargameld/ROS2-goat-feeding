# MuJoCo arena occupancy map

`mujoco_arena.pgm` is generated directly from a loaded `mujoco.MjModel`. The
generator collects all box geoms beneath the `quoridor` and `parkings` bodies,
uses MuJoCo's compiled world transforms, and intersects every box with a
horizontal plane at `z = 0.46 m`. This is the settled height of both lidar sites
(`0.26 + 0.20 m`).

Consequently, the 0.5 m-wide center apertures are represented as follows:

- magenta wall (`x=-2.5`, `y=1.75..2.25`): open at lidar height;
- red wall (`x=2.5`, `y=5.75..6.25`): open at lidar height;
- black and blue walls: closed at lidar height.

The thin parking shelves and shelf back walls do not intersect the lidar plane,
so they are omitted. Their full-height side and rear walls remain occupied.

The generator also derives the map bounds from the resulting polygons and
writes both the PGM image and its YAML metadata.
