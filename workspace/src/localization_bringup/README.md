# localization_bringup

Starts the Nav2 map server, keepout-mask servers, and AMCL for the MuJoCo arena.
The default scan is the 280-degree front lidar; select the rear lidar with a
launch argument.

```bash
ros2 launch localization_bringup localization.launch.py
ros2 launch localization_bringup localization.launch.py scan_topic:=/back_scan
```

Set the initial pose with RViz's **2D Pose Estimate** tool or publish
`geometry_msgs/msg/PoseWithCovarianceStamped` on `/initialpose`.

## Regenerating the map

The generator loads an arena-only `MjModel` from the actual scene. It ignores
MuJoCo extensions and robot-only sections, so the custom lidar plugin does not
need to be loaded. Install the MuJoCo Python binding with:

```bash
python3 -m venv /tmp/localization-map-venv
/tmp/localization-map-venv/bin/pip install -r \
  src/localization_bringup/tools/requirements.txt
/tmp/localization-map-venv/bin/python \
  src/localization_bringup/tools/generate_map.py
```

Use `--height`, `--resolution`, `--margin`, `--model`, or `--output` to change
the generated cross-section. By default, the arena roots are the `quoridor`
and `parkings` bodies; they can be replaced with `--arena-body`.

The navigation keepout mask is the arena cross-section at 0.1 m. Regenerate it
with the same environment used above:

```bash
/tmp/localization-map-venv/bin/python \
  src/localization_bringup/tools/generate_map.py \
  --height 0.1 \
  --output src/localization_bringup/maps/mujoco_keepout.pgm
```
