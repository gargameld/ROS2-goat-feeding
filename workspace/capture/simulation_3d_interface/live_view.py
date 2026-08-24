#!/usr/bin/env python3
"""On-screen viewer for StateCapturePlugin CSV output.

When you click *Apply* this program reads the *last* state written to
``simulation_states.csv`` and renders that MuJoCo state into the Tkinter window
from the camera described by the text boxes. It renders only on demand -- there
is no background loop -- which keeps the load light on a CPU-only container.

The window exposes text boxes that control the camera:

* Perspective point   -- camera position in MuJoCo world coordinates (x, y, z).
* Perspective vector  -- the direction the camera looks (a world-space vector).
* Roll (quaternion w) -- rotation about the perspective vector, given as the
                         ``w`` component of the roll quaternion. w = 1 means no
                         roll; the roll angle is ``2 * acos(w)`` radians.

*Render Video* writes an MP4 covering the requested number of seconds of
simulation time, taken either from the start or from the end of the capture,
seen through the very same camera the text boxes describe. That button needs
FFmpeg in PATH; everything else works without it.

This is a standalone program: it is NOT part of the ROS system and does not
import any ROS packages. It only needs ``mujoco``, ``numpy`` and the standard
library (``tkinter`` for the GUI).

Run it from the command line -- see the module ``__main__`` block / README notes
at the bottom of this file, or the answer accompanying this program.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import tkinter as tk
from tkinter import ttk
import xml.etree.ElementTree as ET

# The MuJoCo Renderer draws off-screen; Tkinter owns the on-screen window. Only
# fall back to EGL when there is no X display: when a display IS present, mixing
# an EGL off-screen context with Tkinter's GLX display can deadlock, so let
# MuJoCo use its display-backed default (glfw) instead.
if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit("NumPy is required. Install it with: python3 -m pip install numpy") from exc

try:
    import mujoco
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "The MuJoCo Python package is required. Install it with:\n"
        "  python3 -m pip install mujoco"
    ) from exc


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
CAPTURE_DIRECTORY = SCRIPT_DIRECTORY.parent
DEFAULT_CSV = CAPTURE_DIRECTORY / "simulation_states.csv"
DEFAULT_MODEL = CAPTURE_DIRECTORY.parent / "mujoco_model" / "scene.xml"
DEFAULT_VIDEO = CAPTURE_DIRECTORY / "live_view_video.mp4"
CAMERA_NAME = "live_view_camera"
ROBOT_CAMERA_NAME = "robot_camera"


# ---------------------------------------------------------------------------
# MJCF sanitising (mirrors render_capture.py so optional plugins are dropped).
# ---------------------------------------------------------------------------
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


def build_model_with_camera(model_path: Path, width: int, height: int, fovy: float) -> tuple[mujoco.MjModel, Path]:
    """Load the model with an added fixed camera whose pose we update per frame."""
    temporary_directory = Path(tempfile.mkdtemp(prefix="live_view_", dir=model_path.parent))
    temporary_path = temporary_directory / model_path.name
    try:
        tree = copy_sanitized_mjcf(model_path, temporary_path, {})
        root = tree.getroot()
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise SystemExit(f"MJCF has no <worldbody>: {model_path}")
        for camera in root.iter("camera"):
            if camera.get("name") == CAMERA_NAME:
                raise SystemExit(f"MJCF already contains a camera named '{CAMERA_NAME}'")

        # A placeholder pose; overwritten every frame via model.cam_pos/cam_quat.
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": CAMERA_NAME,
                "mode": "fixed",
                "pos": "0 0 1",
                "quat": "1 0 0 0",
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

        tree.write(temporary_path, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(temporary_path))
    except Exception:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise
    return model, temporary_directory


# ---------------------------------------------------------------------------
# CSV tail reading.
# ---------------------------------------------------------------------------
def read_qpos_column_count(csv_path: Path) -> int:
    with csv_path.open("r", newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader, None)
    if not header or header[0] != "time":
        raise SystemExit("Capture CSV must start with a 'time' column")
    return sum(1 for name in header[1:] if name.startswith("qpos_"))


def read_last_state(csv_path: Path, qpos_count: int) -> tuple[float, np.ndarray] | None:
    """Return ``(time, qpos)`` for the last complete data row, or ``None``."""
    try:
        with csv_path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            file_size = stream.tell()
            block = 4096
            data = b""
            # Read backwards until we have at least two newlines (last full line)
            # or reach the start of the file.
            while file_size > 0:
                step = min(block, file_size)
                file_size -= step
                stream.seek(file_size)
                data = stream.read(step) + data
                if data.count(b"\n") >= 2:
                    break
    except OSError:
        return None

    text = data.decode("utf-8", errors="replace")
    for line in reversed(text.splitlines()):
        line = line.strip()
        if not line or line.startswith("time"):
            continue
        fields = line.split(",")
        if len(fields) < qpos_count + 1:
            continue
        try:
            timestamp = float(fields[0])
            qpos = np.asarray([float(value) for value in fields[1 : qpos_count + 1]], dtype=float)
        except ValueError:
            continue
        if not np.all(np.isfinite(qpos)) or not math.isfinite(timestamp):
            continue
        return timestamp, qpos
    return None


def read_state_window(
    csv_path: Path, qpos_count: int, duration: float, from_start: bool
) -> list[tuple[float, np.ndarray]]:
    """Return the ``(time, qpos)`` samples inside a window of ``duration`` seconds.

    The window starts at the first captured sample when ``from_start`` is true,
    and ends at the last captured sample otherwise. Malformed rows are skipped
    the same way ``read_last_state`` skips them.
    """
    window_start: float | None = None
    if not from_start:
        last_state = read_last_state(csv_path, qpos_count)
        if last_state is None:
            return []
        window_start = last_state[0] - duration

    samples: list[tuple[float, np.ndarray]] = []
    with csv_path.open("r", newline="") as stream:
        reader = csv.reader(stream)
        next(reader, None)  # header
        for row in reader:
            if not row:
                continue
            values = row[1 : qpos_count + 1]
            if len(values) < qpos_count:
                continue
            try:
                timestamp = float(row[0])
                qpos = np.asarray([float(value) for value in values], dtype=float)
            except ValueError:
                continue
            if not math.isfinite(timestamp) or not np.all(np.isfinite(qpos)):
                continue
            if window_start is None:
                window_start = timestamp
            if timestamp < window_start:
                continue
            if from_start and timestamp > window_start + duration:
                break
            samples.append((timestamp, qpos))
    return samples


# ---------------------------------------------------------------------------
# Camera maths: build a camera orientation from a look direction plus roll.
# ---------------------------------------------------------------------------
def camera_quaternion(perspective_vector: np.ndarray, roll_w: float) -> np.ndarray:
    """Return the MuJoCo camera quaternion (w, x, y, z).

    The camera looks along ``perspective_vector`` (its local -Z axis). ``roll_w``
    is the ``w`` component of a quaternion rotating about that view direction;
    the roll angle is ``2 * acos(roll_w)``.
    """
    forward = np.asarray(perspective_vector, dtype=float)
    norm = float(np.linalg.norm(forward))
    if norm < 1e-9:
        forward = np.array((1.0, 0.0, 0.0))
    else:
        forward = forward / norm

    world_up = np.array((0.0, 0.0, 1.0))
    if abs(float(np.dot(forward, world_up))) > 0.999:
        world_up = np.array((0.0, 1.0, 0.0))

    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, forward)
    true_up /= np.linalg.norm(true_up)

    # MuJoCo camera frame columns: X = right, Y = up, Z = -forward (looks down -Z).
    rotation = np.column_stack((right, true_up, -forward))
    base_quat = np.zeros(4)
    mujoco.mju_mat2Quat(base_quat, rotation.flatten())

    # Roll quaternion about the world-space view direction.
    roll_w = max(-1.0, min(1.0, float(roll_w)))
    sin_half = math.sqrt(max(0.0, 1.0 - roll_w * roll_w))
    roll_quat = np.array(
        (roll_w, forward[0] * sin_half, forward[1] * sin_half, forward[2] * sin_half)
    )

    result = np.zeros(4)
    mujoco.mju_mulQuat(result, roll_quat, base_quat)
    mujoco.mju_normalize4(result)
    return result


# ---------------------------------------------------------------------------
# Tkinter application.
# ---------------------------------------------------------------------------
class LiveViewApp:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.args = args
        self.csv_path = args.csv
        self.qpos_count = read_qpos_column_count(self.csv_path)

        # Heavy MuJoCo resources are created lazily on the first render so the
        # window appears immediately instead of blocking on model/GL init.
        self.model = None
        self.data = None
        self.renderer = None
        self.camera_id = -1
        self.robot_camera_id = -1
        self.temporary_directory: Path | None = None
        self.active_camera = CAMERA_NAME
        self._video_in_progress = False

        self.root.title("Simulation 3D View")
        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _ensure_model(self) -> None:
        """Load the model, data and off-screen renderer once, on demand."""
        if self.renderer is not None:
            return
        self.model, self.temporary_directory = build_model_with_camera(
            self.args.model, self.args.width, self.args.height, self.args.fovy
        )
        if self.model.nq != self.qpos_count:
            shutil.rmtree(self.temporary_directory, ignore_errors=True)
            self.temporary_directory = None
            raise SystemExit(
                f"CSV has {self.qpos_count} qpos values but the model expects {self.model.nq}. "
                "Use the same MJCF model that was active during capture."
            )
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
        self.robot_camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, ROBOT_CAMERA_NAME)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=self.args.height, width=self.args.width)
        self._frame_path = self.temporary_directory / "frame.ppm"

    def _build_widgets(self) -> None:
        controls = ttk.Frame(self.root, padding=8)
        controls.grid(row=0, column=0, sticky="nw")

        self.point_vars = [tk.StringVar(value=v) for v in ("2.5", "2.5", "2.0")]
        self.vector_vars = [tk.StringVar(value=v) for v in ("-1.0", "-1.0", "-0.6")]
        self.roll_var = tk.StringVar(value="1.0")

        ttk.Label(controls, text="Perspective point (x y z):").grid(row=0, column=0, columnspan=3, sticky="w")
        for column, var in enumerate(self.point_vars):
            ttk.Entry(controls, textvariable=var, width=10).grid(row=1, column=column, padx=2, pady=2)

        ttk.Label(controls, text="Perspective vector (x y z):").grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))
        for column, var in enumerate(self.vector_vars):
            ttk.Entry(controls, textvariable=var, width=10).grid(row=3, column=column, padx=2, pady=2)

        ttk.Label(controls, text="Roll about vector (quaternion w):").grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Entry(controls, textvariable=self.roll_var, width=10).grid(row=5, column=0, padx=2, pady=2, sticky="w")

        self.apply_button = ttk.Button(controls, text="Apply", command=self._render)
        self.apply_button.grid(row=6, column=0, columnspan=3, sticky="w", pady=(8, 0))

        self.camera_view_button = ttk.Button(
            controls, text="View From Robot Camera", command=self._toggle_camera_view
        )
        self.camera_view_button.grid(row=7, column=0, columnspan=3, sticky="w", pady=(4, 0))

        # Video controls: length in seconds, which end of the capture to take it
        # from, and the button that renders it with the camera set above.
        ttk.Separator(controls, orient="horizontal").grid(
            row=8, column=0, columnspan=3, sticky="ew", pady=(10, 0)
        )
        ttk.Label(controls, text="Video length (seconds):").grid(
            row=9, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )
        self.video_seconds_var = tk.StringVar(value="5.0")
        ttk.Entry(controls, textvariable=self.video_seconds_var, width=10).grid(
            row=10, column=0, padx=2, pady=2, sticky="w"
        )

        self.video_origin_var = tk.StringVar(value="end")
        ttk.Radiobutton(
            controls, text="From start", variable=self.video_origin_var, value="start"
        ).grid(row=11, column=0, sticky="w")
        ttk.Radiobutton(
            controls, text="From end", variable=self.video_origin_var, value="end"
        ).grid(row=11, column=1, columnspan=2, sticky="w")

        self.video_button = ttk.Button(controls, text="Render Video", command=self._render_video)
        self.video_button.grid(row=12, column=0, columnspan=3, sticky="w", pady=(4, 0))

        self.status_var = tk.StringVar(value="Click Apply to render the last simulation state.")
        ttk.Label(controls, textvariable=self.status_var, foreground="#555", wraplength=200).grid(
            row=13, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )

        ttk.Label(controls, text="Active camera position (world):").grid(
            row=14, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )
        self.camera_pos_var = tk.StringVar(value="--")
        ttk.Label(controls, textvariable=self.camera_pos_var, foreground="#555").grid(
            row=15, column=0, columnspan=3, sticky="w"
        )

        ttk.Label(controls, text="Active camera orientation (world quat):").grid(
            row=16, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )
        self.camera_quat_var = tk.StringVar(value="--")
        ttk.Label(controls, textvariable=self.camera_quat_var, foreground="#555").grid(
            row=17, column=0, columnspan=3, sticky="w"
        )

        self.image_label = ttk.Label(self.root)
        self.image_label.grid(row=0, column=1, padx=8, pady=8)
        self._photo: tk.PhotoImage | None = None

    def _read_vector(self, variables: list[tk.StringVar], fallback: tuple[float, float, float]) -> np.ndarray:
        values = []
        for variable, default in zip(variables, fallback):
            try:
                values.append(float(variable.get()))
            except ValueError:
                values.append(default)
        return np.asarray(values, dtype=float)

    def _toggle_camera_view(self) -> None:
        """Switch between the free-roam perspective camera and the robot's camera."""
        if self.active_camera == CAMERA_NAME:
            self.active_camera = ROBOT_CAMERA_NAME
            self.camera_view_button.configure(text="View From Free Camera")
        else:
            self.active_camera = CAMERA_NAME
            self.camera_view_button.configure(text="View From Robot Camera")
        self._render()

    def _render(self) -> None:
        """Render the last CSV state from the current camera. Runs only on click."""
        self.status_var.set("Rendering...")
        self.root.update_idletasks()  # paint the status before the blocking work
        try:
            self._ensure_model()

            if self.active_camera == ROBOT_CAMERA_NAME and self.robot_camera_id < 0:
                self.status_var.set(f"Model has no camera named '{ROBOT_CAMERA_NAME}'.")
                return

            state = read_last_state(self.csv_path, self.qpos_count)
            if state is None:
                self.status_var.set(f"No state rows found yet in {self.csv_path.name}.")
                return

            timestamp, qpos = state

            self._apply_camera_from_fields()

            self.data.qpos[:] = qpos
            self.data.time = timestamp
            mujoco.mj_forward(self.model, self.data)
            self.renderer.update_scene(self.data, camera=self.active_camera)
            frame = self.renderer.render()
            self._show_frame(frame)
            self._update_camera_pose_labels()
            self.status_var.set(f"Rendered last state at sim time {timestamp:.3f}s.")
        except Exception as error:  # surface failures instead of freezing the UI
            import traceback

            traceback.print_exc()
            self.status_var.set(f"Error: {error}")

    def _apply_camera_from_fields(self) -> None:
        """Point the free camera where the perspective text boxes say to.

        The robot camera rides on the model, so its pose is left untouched.
        """
        if self.active_camera != CAMERA_NAME:
            return
        position = self._read_vector(self.point_vars, (2.5, 2.5, 2.0))
        vector = self._read_vector(self.vector_vars, (-1.0, -1.0, -0.6))
        try:
            roll_w = float(self.roll_var.get())
        except ValueError:
            roll_w = 1.0
        self.model.cam_pos[self.camera_id] = position
        self.model.cam_quat[self.camera_id] = camera_quaternion(vector, roll_w)

    def _set_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for button in (self.apply_button, self.camera_view_button, self.video_button):
            button.configure(state=state)

    def _render_video(self) -> None:
        """Encode the requested window of the capture to MP4. Runs only on click."""
        if self._video_in_progress:
            return

        try:
            duration = float(self.video_seconds_var.get())
        except ValueError:
            self.status_var.set("Video length must be a number of seconds.")
            return
        if not math.isfinite(duration) or duration <= 0.0:
            self.status_var.set("Video length must be a positive number of seconds.")
            return
        if self.args.width % 2 or self.args.height % 2:
            self.status_var.set("Video needs even --width and --height for H.264.")
            return
        if shutil.which("ffmpeg") is None:
            self.status_var.set("FFmpeg was not found in PATH; cannot encode a video.")
            return

        from_start = self.video_origin_var.get() == "start"
        self._video_in_progress = True
        self._set_controls_enabled(False)
        encoder: subprocess.Popen[bytes] | None = None
        try:
            self.status_var.set("Reading capture...")
            self.root.update_idletasks()
            self._ensure_model()

            if self.active_camera == ROBOT_CAMERA_NAME and self.robot_camera_id < 0:
                self.status_var.set(f"Model has no camera named '{ROBOT_CAMERA_NAME}'.")
                return

            samples = read_state_window(self.csv_path, self.qpos_count, duration, from_start)
            if not samples:
                self.status_var.set(f"No state rows found yet in {self.csv_path.name}.")
                return

            self._apply_camera_from_fields()

            fps = self.args.fps
            window_start = samples[0][0]
            window_end = samples[-1][0]
            # Fixed-rate frames covering the window, each holding the most recent
            # captured state, so gaps in the capture become duplicated frames.
            frame_count = max(1, int(math.ceil((window_end - window_start) * fps - 1e-12)))
            output = self.args.video
            output.parent.mkdir(parents=True, exist_ok=True)

            encoder = subprocess.Popen(
                self._ffmpeg_command(fps, output), stdin=subprocess.PIPE, stderr=subprocess.PIPE
            )
            if encoder.stdin is None:
                raise RuntimeError("Failed to open the FFmpeg input pipe")

            sample_index = 0
            current_qpos = samples[0][1]
            progress_interval = max(1, int(round(fps)))
            for frame_index in range(frame_count):
                frame_time = window_start + frame_index / fps
                while sample_index + 1 < len(samples) and samples[sample_index + 1][0] <= frame_time + 1e-12:
                    sample_index += 1
                    current_qpos = samples[sample_index][1]

                self.data.time = frame_time
                self.data.qpos[:] = current_qpos
                mujoco.mj_forward(self.model, self.data)
                self.renderer.update_scene(self.data, camera=self.active_camera)
                encoder.stdin.write(self.renderer.render().tobytes())

                rendered = frame_index + 1
                if rendered % progress_interval == 0 or rendered == frame_count:
                    percent = 100.0 * rendered / frame_count
                    self.status_var.set(f"Encoding {rendered}/{frame_count} frames ({percent:.0f}%)...")
                    self.root.update_idletasks()

            encoder.stdin.close()
            stderr = encoder.stderr.read().decode(errors="replace") if encoder.stderr else ""
            return_code = encoder.wait()
            encoder = None
            if return_code != 0:
                raise RuntimeError(f"FFmpeg failed with exit code {return_code}:\n{stderr.strip()}")

            window = "first" if from_start else "last"
            self.status_var.set(
                f"Wrote {frame_count} frames ({window_end - window_start:.3f}s of sim time, "
                f"{window} {duration:g}s) to {output}"
            )
        except Exception as error:  # surface failures instead of freezing the UI
            import traceback

            traceback.print_exc()
            self.status_var.set(f"Error: {error}")
        finally:
            if encoder is not None:
                if encoder.stdin is not None and not encoder.stdin.closed:
                    encoder.stdin.close()
                encoder.kill()
                encoder.wait()
            self._video_in_progress = False
            self._set_controls_enabled(True)

    def _ffmpeg_command(self, fps: float, output: Path) -> list[str]:
        return [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{self.args.width}x{self.args.height}",
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
            str(self.args.crf),
            "-pix_fmt",
            "yuv420p",
            str(output),
        ]

    def _update_camera_pose_labels(self) -> None:
        """Read the active camera's world-space pose from ``data`` and show it.

        ``data.cam_xpos``/``data.cam_xmat`` reflect the actual world pose after
        forward kinematics, which is the only way to know where a camera that
        is attached to a moving body (like ``robot_camera``) actually ended up.
        """
        active_id = self.camera_id if self.active_camera == CAMERA_NAME else self.robot_camera_id
        if active_id < 0:
            self.camera_pos_var.set("--")
            self.camera_quat_var.set("--")
            return
        position = self.data.cam_xpos[active_id]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.data.cam_xmat[active_id])
        self.camera_pos_var.set(f"{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}")
        self.camera_quat_var.set(f"{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}")

    def _show_frame(self, frame: np.ndarray) -> None:
        """Display an RGB uint8 array in the Tk label without needing PIL.

        Tk's PhotoImage reads the binary PPM (P6) format from a file, so the
        frame is written to a temporary .ppm and loaded via ``file=``.
        """
        height, width, _ = frame.shape
        header = f"P6\n{width} {height}\n255\n".encode("ascii")
        ppm = header + np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
        self._frame_path.write_bytes(ppm)
        self._photo = tk.PhotoImage(file=str(self._frame_path))
        self.image_label.configure(image=self._photo)

    def _on_close(self) -> None:
        try:
            if self.renderer is not None:
                self.renderer.close()
        finally:
            if self.temporary_directory is not None:
                shutil.rmtree(self.temporary_directory, ignore_errors=True)
            self.root.destroy()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help=f"capture CSV (default: {DEFAULT_CSV})")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help=f"MJCF scene (default: {DEFAULT_MODEL})")
    parser.add_argument("--width", type=int, default=640, help="render width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="render height (default: 480)")
    parser.add_argument("--fovy", type=float, default=45.0, help="vertical field of view in degrees (default: 45)")
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO, help=f"output MP4 for Render Video (default: {DEFAULT_VIDEO})")
    parser.add_argument("--fps", type=float, default=30.0, help="Render Video frame rate (default: 30)")
    parser.add_argument("--crf", type=int, default=18, help="H.264 quality, 0-51; lower is better (default: 18)")
    args = parser.parse_args()
    args.csv = args.csv.expanduser().resolve()
    args.model = args.model.expanduser().resolve()
    args.video = args.video.expanduser().resolve()
    if args.fps <= 0:
        raise SystemExit("--fps must be positive")
    if not 0 <= args.crf <= 51:
        raise SystemExit("--crf must be in the range [0, 51]")
    if not args.csv.is_file():
        raise SystemExit(f"Capture CSV does not exist: {args.csv}")
    if not args.model.is_file():
        raise SystemExit(f"MJCF model does not exist: {args.model}")
    return args


def main() -> None:
    args = parse_arguments()
    root = tk.Tk()
    LiveViewApp(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
