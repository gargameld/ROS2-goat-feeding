#!/usr/bin/env python3
"""Replays StateCapturePlugin's CSV output in MuJoCo's own viewer window.

This is the animated counterpart of ``live_view.py``: instead of rendering a
single frame into a Tkinter label when you press *Apply*, it opens MuJoCo's
interactive viewer (``mujoco.viewer``) and drives it at a fixed refresh rate --
5 Hz by default.

Playback starts at the *first* state in ``simulation_states.csv`` and walks
forward through the file, so you watch the run from the beginning. The frames
are paced by captured simulation time, not by rows: each tick the playback clock
advances by one refresh period (times ``--speed``) and the window shows the most
recent captured state at or before that clock, exactly the way the video
renderer in ``live_view.py`` builds its frames. A capture that is still being
written is picked up as it grows; when playback reaches the last state the clock
holds there until more states are appended, and if the capture restarts (the CSV
shrinks) playback restarts from the top with it.

No physics is stepped. The viewer is *passive*: the qpos vector comes straight
out of the CSV, ``mj_forward`` places the bodies, and the window is synced. Mouse
and keyboard work as usual, so you can orbit, zoom, toggle visualisation flags
and pick bodies while the replay runs.

This is a standalone program: it is NOT part of the ROS system and does not
import any ROS packages. It needs ``mujoco``, ``numpy`` and a display; the MJCF
sanitising and CSV header handling are reused from ``live_view.py``.

Typical use::

    python3 live_mujoco_viewer.py                 # 5 Hz, real time, from the start
    python3 live_mujoco_viewer.py --speed 5       # replay five times faster
    python3 live_mujoco_viewer.py --rate 10       # smoother, still real time
"""

from __future__ import annotations

import argparse
import collections
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import tempfile
import threading
import time

# The interactive viewer draws into a real window, so it needs a windowed GL
# backend. Claim one before importing live_view, which otherwise falls back to
# off-screen EGL when it cannot see a display.
os.environ.setdefault("MUJOCO_GL", "glfw")

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit("NumPy is required. Install it with: python3 -m pip install numpy") from exc

try:
    import mujoco
    import mujoco.viewer
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "The MuJoCo Python package is required. Install it with:\n"
        "  python3 -m pip install mujoco"
    ) from exc

from live_view import DEFAULT_CSV, DEFAULT_MODEL, copy_sanitized_mjcf, read_qpos_column_count


DEFAULT_RATE = 5.0

# Set when the process is asked to terminate. The loop below checks it rather
# than unwinding from inside the handler, which would tear the GL window down
# from the middle of a viewer call.
STOP_REQUESTED = threading.Event()


def build_model(model_path: Path) -> tuple[mujoco.MjModel, Path]:
    """Load the MJCF with optional plugins stripped, as render_capture does.

    The sanitised copy lives next to the original model so that relative asset
    paths inside the includes keep resolving; the caller removes it afterwards.
    """
    temporary_directory = Path(tempfile.mkdtemp(prefix="live_viewer_", dir=model_path.parent))
    temporary_path = temporary_directory / model_path.name
    try:
        copy_sanitized_mjcf(model_path, temporary_path, {})
        model = mujoco.MjModel.from_xml_path(str(temporary_path))
    except Exception:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise
    return model, temporary_directory


def row_time(line: bytes) -> float | None:
    """Return the simulation time of a CSV row, or ``None`` if it has none.

    Only the first field is touched: the qpos values of a row are parsed later,
    and only for the rows that actually reach the window.
    """
    field, _, rest = line.partition(b",")
    if not rest:
        return None
    try:
        timestamp = float(field)
    except ValueError:  # the header row, or a partial write
        return None
    return timestamp if math.isfinite(timestamp) else None


def parse_qpos(line: bytes, qpos_count: int) -> np.ndarray | None:
    """Return the qpos vector of a CSV row, or ``None`` if the row is unusable."""
    fields = line.decode("utf-8", errors="replace").split(",")
    if len(fields) < qpos_count + 1:
        return None
    try:
        qpos = np.asarray([float(value) for value in fields[1 : qpos_count + 1]], dtype=float)
    except ValueError:
        return None
    return qpos if np.all(np.isfinite(qpos)) else None


class StateStream:
    """Reads the capture CSV from the top and hands out its rows in order.

    The file handle stays open between calls and only the bytes appended since
    the previous call are read, so following a growing capture stays cheap. A
    partial final line is held back until its newline arrives.
    """

    def __init__(self, csv_path: Path, qpos_count: int) -> None:
        self.csv_path = csv_path
        self.qpos_count = qpos_count
        self._stream = None
        self._partial = b""

    def _open(self) -> None:
        self._stream = self.csv_path.open("rb")
        self._partial = b""

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None

    def new_rows(self) -> tuple[bool, list[tuple[float, bytes]]]:
        """Return ``(restarted, rows)`` for everything appended since last call.

        Each row is ``(time, line)``. ``restarted`` is true when the capture
        truncated the file -- a new run -- and reading began again from the top.
        """
        restarted = False
        if self._stream is None:
            try:
                self._open()
            except OSError:
                return False, []
        try:
            # A shrunken file means the capture restarted: re-read from the top.
            if self.csv_path.stat().st_size < self._stream.tell():
                self.close()
                self._open()
                restarted = True
            chunk = self._stream.read()
        except OSError:
            self.close()
            return restarted, []

        if not chunk:
            return restarted, []
        lines = (self._partial + chunk).split(b"\n")
        self._partial = lines.pop()
        rows = []
        for line in lines:
            timestamp = row_time(line)
            if timestamp is not None:
                rows.append((timestamp, line))
        return restarted, rows


def run_viewer(args: argparse.Namespace) -> None:
    qpos_count = read_qpos_column_count(args.csv)
    model, temporary_directory = build_model(args.model)
    try:
        if model.nq != qpos_count:
            raise SystemExit(
                f"CSV has {qpos_count} qpos values but the model expects {model.nq}. "
                "Use the same MJCF model that was active during capture."
            )

        data = mujoco.MjData(model)
        stream = StateStream(args.csv, qpos_count)
        period = 1.0 / args.rate
        step = period * args.speed  # simulation seconds covered by one frame

        print(
            f"Replaying {args.csv} from the start at {args.rate:g} Hz "
            f"({args.speed:g}x). Close the window to stop."
        )
        with mujoco.viewer.launch_passive(model, data) as viewer:
            upcoming: collections.deque[tuple[float, bytes]] = collections.deque()
            playback_time: float | None = None
            shown_time: float | None = None
            frames = 0
            next_tick = time.perf_counter()
            while viewer.is_running() and not STOP_REQUESTED.is_set():
                restarted, rows = stream.new_rows()
                if restarted:
                    upcoming.clear()
                    playback_time = None
                    shown_time = None
                upcoming.extend(rows)

                if playback_time is None:
                    # First frame: begin at the first state the capture holds.
                    if upcoming:
                        playback_time = upcoming[0][0]
                elif upcoming:
                    # Advance only while there is something ahead to show, so
                    # reaching the end of a growing capture waits instead of
                    # running the clock off into rows that do not exist yet.
                    playback_time += step

                line = None
                while upcoming and upcoming[0][0] <= playback_time:
                    shown_time, line = upcoming.popleft()
                if line is not None:
                    qpos = parse_qpos(line, qpos_count)
                    if qpos is not None:
                        # Replay only: velocities stay zero and no step is taken,
                        # so the window shows exactly what was captured.
                        data.qpos[:] = qpos
                        data.qvel[:] = 0.0
                        data.time = shown_time
                        mujoco.mj_forward(model, data)
                        frames += 1
                viewer.sync()

                clock = "--" if shown_time is None else f"{shown_time:.3f}s"
                waiting = "  (waiting for new states)" if line is None else ""
                print(
                    f"\rframe {frames}   sim time {clock}   queued {len(upcoming)}{waiting}   ",
                    end="",
                    flush=True,
                )

                next_tick += period
                remaining = next_tick - time.perf_counter()
                if remaining > 0.0:
                    STOP_REQUESTED.wait(remaining)
                else:
                    next_tick = time.perf_counter()  # fell behind; drop the backlog
        stream.close()
        print()
    finally:
        shutil.rmtree(temporary_directory, ignore_errors=True)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help=f"capture CSV (default: {DEFAULT_CSV})")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help=f"MJCF scene (default: {DEFAULT_MODEL})")
    parser.add_argument(
        "--rate", type=float, default=DEFAULT_RATE, help=f"window refresh rate in Hz (default: {DEFAULT_RATE:g})"
    )
    parser.add_argument(
        "--speed", type=float, default=1.0, help="playback speed relative to simulation time (default: 1)"
    )
    args = parser.parse_args()
    args.csv = args.csv.expanduser().resolve()
    args.model = args.model.expanduser().resolve()
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        raise SystemExit("--rate must be a positive number of hertz")
    if not math.isfinite(args.speed) or args.speed <= 0.0:
        raise SystemExit("--speed must be a positive multiplier")
    if not args.csv.is_file():
        raise SystemExit(f"Capture CSV does not exist: {args.csv}")
    if not args.model.is_file():
        raise SystemExit(f"MJCF model does not exist: {args.model}")
    return args


def main() -> None:
    # Leave the loop on SIGTERM the same way closing the window does, so the
    # sanitised copy of the model is removed instead of being left behind.
    signal.signal(signal.SIGTERM, lambda signum, frame: STOP_REQUESTED.set())
    try:
        run_viewer(parse_arguments())
    except KeyboardInterrupt:
        print()
    # MuJoCo's viewer leaves GL resources that segfault when the interpreter
    # frees them after the window is gone. Everything the program had to do is
    # finished by now, so leave without running that teardown.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
