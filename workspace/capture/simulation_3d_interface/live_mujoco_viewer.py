#!/usr/bin/env python3
"""Replay a StateCapturePlugin CSV in MuJoCo's interactive viewer.

The MJCF model is loaded once. Each CSV row then replaces ``data.qpos`` at the
captured simulation time, followed by ``mj_forward`` and a viewer sync. Physics
is never stepped.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import tempfile
import threading
import time
from collections.abc import Iterator

os.environ.setdefault("MUJOCO_GL", "glfw")

try:
    import mujoco
    import mujoco.viewer
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "The MuJoCo Python package is required. Install it with:\n"
        "  python3 -m pip install mujoco"
    ) from exc

from live_view import DEFAULT_CSV, DEFAULT_MODEL, copy_sanitized_mjcf


STOP_REQUESTED = threading.Event()


def build_model(model_path: Path) -> tuple[mujoco.MjModel, Path]:
    """Load one temporary MJCF copy with optional plugins removed."""
    temporary_directory = Path(tempfile.mkdtemp(prefix="live_viewer_"))
    temporary_path = temporary_directory / model_path.name
    try:
        copy_sanitized_mjcf(model_path, temporary_path, {})
        return mujoco.MjModel.from_xml_path(str(temporary_path)), temporary_directory
    except Exception:
        shutil.rmtree(temporary_directory, ignore_errors=True)
        raise


def read_states(csv_path: Path) -> tuple[int, Iterator[tuple[float, list[float]]]]:
    """Return the qpos width and an iterator over valid captured states."""
    stream = csv_path.open("r", encoding="utf-8", newline="")
    reader = csv.reader(stream)
    header = next(reader, None)
    if not header or header[0] != "time":
        stream.close()
        raise SystemExit("Capture CSV must start with a 'time' column")

    qpos_count = sum(name.startswith("qpos_") for name in header[1:])

    def states() -> Iterator[tuple[float, list[float]]]:
        try:
            for row in reader:
                if len(row) < qpos_count + 1:
                    continue
                try:
                    timestamp = float(row[0])
                    qpos = [float(value) for value in row[1 : qpos_count + 1]]
                except ValueError:
                    continue
                if math.isfinite(timestamp) and all(math.isfinite(value) for value in qpos):
                    yield timestamp, qpos
        finally:
            stream.close()

    return qpos_count, states()


def wait_for_frame(deadline: float, viewer: mujoco.viewer.Handle) -> bool:
    """Wait until a frame is due; return false if playback should stop."""
    while viewer.is_running() and not STOP_REQUESTED.is_set():
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            return True
        STOP_REQUESTED.wait(min(remaining, 0.05))
    return False


def show_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer: mujoco.viewer.Handle,
    timestamp: float,
    qpos: list[float],
) -> None:
    """Apply one captured state to the passive viewer."""
    with viewer.lock():
        data.qpos[:] = qpos
        data.qvel[:] = 0.0
        data.time = timestamp
        mujoco.mj_forward(model, data)
    viewer.sync()


def run_viewer(args: argparse.Namespace) -> None:
    qpos_count, states = read_states(args.csv)
    first_state = next(states, None)
    if first_state is None:
        raise SystemExit(f"Capture CSV has no complete state rows: {args.csv}")

    try:
        model, temporary_directory = build_model(args.model)
    except Exception:
        states.close()
        raise
    try:
        if model.nq != qpos_count:
            raise SystemExit(
                f"CSV has {qpos_count} qpos values but the model expects {model.nq}. "
                "Use the same MJCF model that was active during capture."
            )

        data = mujoco.MjData(model)
        first_time = first_state[0]
        frame_count = 0

        print(f"Replaying {args.csv} at {args.speed:g}x. Close the window to stop.")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            wall_start = time.perf_counter()
            for timestamp, qpos in itertools.chain((first_state,), states):
                deadline = wall_start + (timestamp - first_time) / args.speed
                if not wait_for_frame(deadline, viewer):
                    break
                show_state(model, data, viewer, timestamp, qpos)
                frame_count += 1

            while viewer.is_running() and not STOP_REQUESTED.wait(0.05):
                viewer.sync()

        print(f"Replayed {frame_count} frame(s).")
    finally:
        states.close()
        shutil.rmtree(temporary_directory, ignore_errors=True)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help=f"capture CSV (default: {DEFAULT_CSV})")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help=f"MJCF scene (default: {DEFAULT_MODEL})")
    parser.add_argument(
        "--speed", type=float, default=1.0, help="playback speed relative to simulation time (default: 1)"
    )
    args = parser.parse_args()
    args.csv = args.csv.expanduser().resolve()
    args.model = args.model.expanduser().resolve()
    if not math.isfinite(args.speed) or args.speed <= 0.0:
        raise SystemExit("--speed must be a positive multiplier")
    if not args.csv.is_file():
        raise SystemExit(f"Capture CSV does not exist: {args.csv}")
    if not args.model.is_file():
        raise SystemExit(f"MJCF model does not exist: {args.model}")
    return args


def main() -> None:
    signal.signal(signal.SIGTERM, lambda signum, frame: STOP_REQUESTED.set())
    try:
        run_viewer(parse_arguments())
    except KeyboardInterrupt:
        print()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
