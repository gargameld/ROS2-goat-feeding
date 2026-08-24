#!/usr/bin/env python3
"""Split a ROS 2 launch log into one live log file per component.

By default this reads ``launch.log`` beside this script from the beginning,
writes component logs to ``splitted_log/``, and keeps following the input.
Stop it with Ctrl-C.
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import time
from pathlib import Path
from typing import BinaryIO, TextIO


ANSI_ESCAPE_RE = re.compile(rb"\x1b\[[0-?]*[ -/]*[@-~]")
PREFIX_RE = re.compile(rb"^\[([^]\r\n]+)\](?:\s+\[([^]\r\n]+)\])?")
LOG_LEVELS = {"DEBUG", "INFO", "WARN", "WARNING", "ERROR", "FATAL"}
SAFE_FILENAME_RE = re.compile(r"[^A-Za-z0-9_.-]+")


def component_for_line(line: bytes) -> str:
    """Return the launch component name found at the start of *line*."""
    clean_line = ANSI_ESCAPE_RE.sub(b"", line)
    match = PREFIX_RE.match(clean_line)
    if match is None:
        return "unattributed"

    first = match.group(1).decode("utf-8", errors="replace").strip()
    second_bytes = match.group(2)
    if first.upper() in LOG_LEVELS:
        # Launch's own messages look like: [INFO] [launch]: ...
        # Process status messages look like: [INFO] [node-1]: ...
        if second_bytes:
            return second_bytes.decode("utf-8", errors="replace").strip()
        return "launch"
    return first


def safe_filename(component: str) -> str:
    """Convert a component label into a safe, readable filename."""
    name = SAFE_FILENAME_RE.sub("_", component).strip("._")
    return f"{name or 'unattributed'}.log"


class ComponentLogs:
    def __init__(self, output_dir: Path, append: bool) -> None:
        self.output_dir = output_dir
        self.mode = "a" if append else "w"
        self.handles: dict[str, TextIO] = {}
        output_dir.mkdir(parents=True, exist_ok=True)

    def write(self, line: bytes) -> None:
        component = component_for_line(line)
        handle = self.handles.get(component)
        if handle is None:
            path = self.output_dir / safe_filename(component)
            handle = path.open(self.mode, encoding="utf-8", errors="replace")
            self.handles[component] = handle
        handle.write(line.decode("utf-8", errors="replace"))
        handle.flush()

    def close(self) -> None:
        for handle in self.handles.values():
            handle.close()
        self.handles.clear()


def file_identity(handle: BinaryIO) -> tuple[int, int]:
    info = os.fstat(handle.fileno())
    return info.st_dev, info.st_ino


def split_log(
    input_path: Path,
    output_dir: Path,
    *,
    follow: bool,
    from_end: bool,
    append: bool,
    poll_interval: float,
) -> None:
    """Split existing lines and optionally follow file truncation/replacement."""
    running = True

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    outputs = ComponentLogs(output_dir, append)
    source: BinaryIO | None = None
    identity: tuple[int, int] | None = None
    pending = b""

    try:
        while running:
            if source is None:
                try:
                    source = input_path.open("rb")
                except FileNotFoundError:
                    if not follow:
                        raise
                    time.sleep(poll_interval)
                    continue
                identity = file_identity(source)
                if from_end:
                    source.seek(0, 2)
                    from_end = False

            chunk = source.read(64 * 1024)
            if chunk:
                pending += chunk
                lines = pending.splitlines(keepends=True)
                if lines and not lines[-1].endswith((b"\n", b"\r")):
                    pending = lines.pop()
                else:
                    pending = b""
                for line in lines:
                    outputs.write(line)
                continue

            if not follow:
                if pending:
                    outputs.write(pending)
                break

            try:
                current = input_path.stat()
            except FileNotFoundError:
                current = None

            replaced = current is None or (current.st_dev, current.st_ino) != identity
            truncated = current is not None and current.st_size < source.tell()
            if replaced or truncated:
                if pending:
                    outputs.write(pending)
                    pending = b""
                source.close()
                source = None
                identity = None
                continue

            time.sleep(poll_interval)
    finally:
        if source is not None:
            source.close()
        outputs.close()


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=script_dir / "launch.log",
        help="launch log to read (default: launch.log beside this script)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=script_dir / "splitted_log",
        help="directory for component logs (default: splitted_log beside this script)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="split the current file and exit instead of following it",
    )
    parser.add_argument(
        "--from-end",
        action="store_true",
        help="ignore existing input and only split lines written after startup",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="append to existing component logs (the default replaces each one)",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.2,
        metavar="SECONDS",
        help="delay between checks for new data (default: 0.2)",
    )
    args = parser.parse_args()
    if args.poll_interval <= 0:
        parser.error("--poll-interval must be greater than zero")
    return args


def main() -> None:
    args = parse_args()
    split_log(
        args.input,
        args.output_dir,
        follow=not args.once,
        from_end=args.from_end,
        append=args.append,
        poll_interval=args.poll_interval,
    )


if __name__ == "__main__":
    main()
