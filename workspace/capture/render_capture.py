#!/usr/bin/env python3
"""Render a StateCapturePlugin CSV to a simulation-time-paced MP4 with MuJoCo."""

from __future__ import annotations

import argparse
from collections.abc import Iterator
import csv
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

# Use headless EGL rendering when no window-system display is available.
if "MUJOCO_GL" not in os.environ and "DISPLAY" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("NumPy is required. Install it with: python3 -m pip install numpy") from exc

try:
    import mujoco
except ImportError as exc:
    raise SystemExit(
        "The MuJoCo Python package is required. Install it with:\n"
        "  python3 -m pip install mujoco"
    ) from exc


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
DEFAULT_CSV = SCRIPT_DIRECTORY / "simulation_states.csv"
DEFAULT_MODEL = SCRIPT_DIRECTORY.parent / "src" / "robot_description" / "mjcf" / "scene.xml"
DEFAULT_OUTPUT = SCRIPT_DIRECTORY / "simulation.mp4"
CAMERA_NAME = "capture_render_camera"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Render qpos samples captured by StateCapturePlugin. CAMERA_X, CAMERA_Y, "
            "and CAMERA_Z set the camera position in MuJoCo world coordinates."
        )
    )
    parser.add_argument("camera_x", type=float, help="camera world X position")
    parser.add_argument("camera_y", type=float, help="camera world Y position")
    parser.add_argument("camera_z", type=float, help="camera world Z position")
    parser.add_argument(
        "--look-at",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.5),
        help="world position at which the camera points (default: 0 0 0.5)",
    )
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help=f"capture CSV (default: {DEFAULT_CSV})")
    parser.add_argument(
        "--model", type=Path, default=DEFAULT_MODEL, help=f"MJCF scene file (default: {DEFAULT_MODEL})"
    )
    parser.add_argument(
        "--output", type=Path, default=DEFAULT_OUTPUT, help=f"output MP4 file (default: {DEFAULT_OUTPUT})"
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="output video frame rate used for the simulation-time timeline (default: 30)",
    )
    parser.add_argument("--width", type=int, default=640, help="video width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="video height (default: 480)")
    parser.add_argument("--fovy", type=float, default=45.0, help="vertical field of view in degrees (default: 45)")
    parser.add_argument("--crf", type=int, default=18, help="H.264 quality, 0-51; lower is better (default: 18)")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="render only the first N frames (useful for testing a camera position)",
    )
    parser.add_argument("--overwrite", action="store_true", help="replace the output file if it already exists")
    return parser.parse_args()


def validate_arguments(args: argparse.Namespace) -> None:
    args.csv = args.csv.expanduser().resolve()
    args.model = args.model.expanduser().resolve()
    args.output = args.output.expanduser().resolve()

    if not args.csv.is_file():
        raise SystemExit(f"Capture CSV does not exist: {args.csv}")
    if not args.model.is_file():
        raise SystemExit(f"MJCF model does not exist: {args.model}")
    if args.output.exists() and not args.overwrite:
        raise SystemExit(f"Output already exists: {args.output}\nUse --overwrite to replace it.")
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("--width and --height must be positive")
    if args.width % 2 or args.height % 2:
        raise SystemExit("--width and --height must be even for H.264 video")
    if args.fps <= 0:
        raise SystemExit("--fps must be positive")
    if not 1.0 <= args.fovy < 180.0:
        raise SystemExit("--fovy must be in the range [1, 180)")
    if not 0 <= args.crf <= 51:
        raise SystemExit("--crf must be in the range [0, 51]")
    if args.max_frames is not None and args.max_frames <= 0:
        raise SystemExit("--max-frames must be positive")
    if shutil.which("ffmpeg") is None:
        raise SystemExit("FFmpeg was not found in PATH. Install ffmpeg before running this program.")


def camera_axes(position: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    forward = target - position
    length = float(np.linalg.norm(forward))
    if length < 1e-9:
        raise SystemExit("Camera position and --look-at position must be different")
    forward /= length

    world_up = np.array((0.0, 0.0, 1.0))
    if abs(float(np.dot(forward, world_up))) > 0.999:
        world_up = np.array((0.0, 1.0, 0.0))

    x_axis = np.cross(forward, world_up)
    x_axis /= np.linalg.norm(x_axis)
    z_axis = -forward
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    return x_axis, y_axis


def vector_attribute(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.17g}" for value in values)


def remove_mjcf_plugins(root: ET.Element) -> None:
    """Remove extension declarations and elements backed by those plugins."""
    for parent in root.iter():
        for child in list(parent):
            if child.tag in {"extension", "plugin"}:
                parent.remove(child)


def copy_sanitized_mjcf(source: Path, destination: Path, copied: dict[Path, Path]) -> ET.ElementTree:
    """Copy an MJCF file and its includes while omitting optional plugins."""
    source = source.resolve()
    if source in copied:
        return ET.parse(copied[source])

    tree = ET.parse(source)
    root = tree.getroot()
    remove_mjcf_plugins(root)
    destination.parent.mkdir(parents=True, exist_ok=True)
    copied[source] = destination

    for include in root.iter("include"):
        include_name = include.get("file")
        if not include_name:
            continue
        included_source = (source.parent / include_name).resolve()
        included_destination = destination.parent / include_name
        copy_sanitized_mjcf(included_source, included_destination, copied)

    # Included XML is copied, but meshes and other assets remain in the original
    # model tree. Absolute paths keep those resources available to MuJoCo.
    for element in root.iter():
        if element.tag == "include" or "file" not in element.attrib:
            continue
        asset_path = Path(element.attrib["file"])
        if not asset_path.is_absolute():
            element.set("file", str((source.parent / asset_path).resolve()))

    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return tree


def make_model_with_camera(
    model_path: Path,
    position: np.ndarray,
    target: np.ndarray,
    width: int,
    height: int,
    fovy: float,
) -> tuple[Path, Path]:
    temporary_directory = Path(tempfile.mkdtemp(prefix="capture_render_", dir=model_path.parent))
    temporary_path = temporary_directory / model_path.name
    try:
        tree = copy_sanitized_mjcf(model_path, temporary_path, {})
    except Exception:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise SystemExit(f"MJCF has no <worldbody>: {model_path}")

    for camera in root.iter("camera"):
        if camera.get("name") == CAMERA_NAME:
            raise SystemExit(f"MJCF already contains a camera named '{CAMERA_NAME}'")

    x_axis, y_axis = camera_axes(position, target)
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": CAMERA_NAME,
            "mode": "fixed",
            "pos": vector_attribute(position),
            "xyaxes": f"{vector_attribute(x_axis)} {vector_attribute(y_axis)}",
            "fovy": f"{fovy:.17g}",
        },
    )

    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", str(width))
    global_visual.set("offheight", str(height))

    try:
        tree.write(temporary_path, encoding="utf-8", xml_declaration=True)
    except Exception:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise
    return temporary_path, temporary_directory


def inspect_capture(csv_path: Path) -> tuple[list[int], int, float, float]:
    frame_count = 0
    first_timestamp: float | None = None
    previous_timestamp: float | None = None

    with csv_path.open("r", newline="") as stream:
        reader = csv.reader(stream)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise SystemExit(f"Capture CSV is empty: {csv_path}") from exc

        if not header or header[0] != "time":
            raise SystemExit("Capture CSV must start with a 'time' column")

        indexed_columns: list[tuple[int, int]] = []
        for column, name in enumerate(header[1:], start=1):
            if not name.startswith("qpos_"):
                continue
            try:
                qpos_index = int(name.removeprefix("qpos_"))
            except ValueError as exc:
                raise SystemExit(f"Invalid qpos column name: {name}") from exc
            indexed_columns.append((qpos_index, column))

        indexed_columns.sort()
        expected_indices = list(range(len(indexed_columns)))
        if [index for index, _ in indexed_columns] != expected_indices:
            raise SystemExit("CSV qpos columns must be contiguous and start at qpos_0")
        qpos_columns = [column for _, column in indexed_columns]

        for line_number, row in enumerate(reader, start=2):
            if not row:
                continue
            try:
                timestamp = float(row[0])
            except (ValueError, IndexError) as exc:
                raise SystemExit(f"Invalid timestamp at CSV line {line_number}") from exc
            if not math.isfinite(timestamp):
                raise SystemExit(f"Non-finite timestamp at CSV line {line_number}")
            if previous_timestamp is not None and timestamp < previous_timestamp:
                raise SystemExit(
                    f"Simulation time moves backwards at CSV line {line_number}: "
                    f"{timestamp} follows {previous_timestamp}"
                )
            if first_timestamp is None:
                first_timestamp = timestamp
            previous_timestamp = timestamp
            frame_count += 1

    if not qpos_columns:
        raise SystemExit("Capture CSV contains no qpos columns")
    if frame_count == 0 or first_timestamp is None or previous_timestamp is None:
        raise SystemExit("Capture CSV contains no state samples")
    return qpos_columns, frame_count, first_timestamp, previous_timestamp


def output_frame_count(first_timestamp: float, last_timestamp: float, fps: float) -> int:
    """Return enough fixed-rate frames to cover the captured simulation interval."""
    duration = last_timestamp - first_timestamp
    return max(1, int(math.ceil(duration * fps - 1e-12)))


def capture_samples(csv_path: Path, qpos_columns: list[int]) -> Iterator[tuple[float, np.ndarray]]:
    """Yield validated ``(simulation_time, qpos)`` samples from a capture file."""
    with csv_path.open("r", newline="") as stream:
        reader = csv.reader(stream)
        next(reader)
        for line_number, row in enumerate(reader, start=2):
            if not row:
                continue
            try:
                timestamp = float(row[0])
                qpos = np.asarray([float(row[column]) for column in qpos_columns])
            except (ValueError, IndexError) as exc:
                raise SystemExit(f"Invalid state sample at CSV line {line_number}") from exc
            if not np.all(np.isfinite(qpos)):
                raise SystemExit(f"Non-finite qpos value at CSV line {line_number}")
            yield timestamp, qpos


def ffmpeg_command(args: argparse.Namespace, fps: float) -> list[str]:
    overwrite_flag = "-y" if args.overwrite else "-n"
    return [
        "ffmpeg",
        overwrite_flag,
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{args.width}x{args.height}",
        "-framerate",
        f"{fps:.12g}",
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        str(args.crf),
        "-pix_fmt",
        "yuv420p",
        str(args.output),
    ]


def render_video(args: argparse.Namespace) -> None:
    qpos_columns, sample_count, first_timestamp, last_timestamp = inspect_capture(args.csv)
    capture_duration = last_timestamp - first_timestamp
    frame_count = output_frame_count(first_timestamp, last_timestamp, args.fps)
    if args.max_frames is not None:
        frame_count = min(frame_count, args.max_frames)
    fps = args.fps
    camera_position = np.asarray((args.camera_x, args.camera_y, args.camera_z), dtype=float)
    look_at = np.asarray(args.look_at, dtype=float)

    temporary_model, temporary_directory = make_model_with_camera(
        args.model, camera_position, look_at, args.width, args.height, args.fovy
    )
    encoder: subprocess.Popen[bytes] | None = None
    renderer = None
    try:
        model = mujoco.MjModel.from_xml_path(str(temporary_model))
        if len(qpos_columns) != model.nq:
            raise SystemExit(
                f"CSV contains {len(qpos_columns)} qpos values, but the MJCF model expects {model.nq}. "
                "Use the same MJCF model that was active during capture."
            )

        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, height=args.height, width=args.width)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        encoder = subprocess.Popen(
            ffmpeg_command(args, fps), stdin=subprocess.PIPE, stderr=subprocess.PIPE
        )
        if encoder.stdin is None:
            raise RuntimeError("Failed to open the FFmpeg input pipe")

        print(
            f"Rendering {frame_count} frames at {fps:.3f} FPS from {sample_count} samples "
            f"spanning {capture_duration:.3f} seconds of simulation time"
        )
        print(f"Camera: {vector_attribute(camera_position)} -> {vector_attribute(look_at)}")
        print(f"Output: {args.output}")

        progress_interval = max(1, int(round(fps * 5.0)))
        samples = iter(capture_samples(args.csv, qpos_columns))
        _, current_qpos = next(samples)
        next_sample = next(samples, None)

        for frame_index in range(frame_count):
            frame_time = first_timestamp + frame_index / fps
            while next_sample is not None and next_sample[0] <= frame_time + 1e-12:
                _, current_qpos = next_sample
                next_sample = next(samples, None)

            # Hold each captured state until the next sample's simulation timestamp.
            # This preserves recorded states while duplicate frames account for gaps.
            data.time = frame_time
            data.qpos[:] = current_qpos
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=CAMERA_NAME)
            encoder.stdin.write(renderer.render().tobytes())
            rendered = frame_index + 1

            if rendered % progress_interval == 0 or rendered == frame_count:
                percent = 100.0 * rendered / frame_count
                print(f"\rRendered {rendered}/{frame_count} frames ({percent:.1f}%)", end="", flush=True)

        print()
        encoder.stdin.close()
        stderr = encoder.stderr.read().decode(errors="replace") if encoder.stderr else ""
        return_code = encoder.wait()
        if return_code != 0:
            raise SystemExit(f"FFmpeg failed with exit code {return_code}:\n{stderr.strip()}")
        encoder = None
        print(f"Video created successfully: {args.output}")
    finally:
        if encoder is not None:
            if encoder.stdin is not None and not encoder.stdin.closed:
                encoder.stdin.close()
            encoder.kill()
            encoder.wait()
        if renderer is not None:
            renderer.close()
        shutil.rmtree(temporary_directory, ignore_errors=True)


def main() -> None:
    args = parse_arguments()
    validate_arguments(args)
    render_video(args)


if __name__ == "__main__":
    main()
