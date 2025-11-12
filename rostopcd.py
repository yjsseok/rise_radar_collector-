#!/usr/bin/env python3
"""Extract PointCloud2 messages from rosbag2 directories into PCD files."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import rosbag2_py  # type: ignore
    from rclpy.serialization import deserialize_message  # type: ignore
    from rosidl_runtime_py.utilities import get_message  # type: ignore
    from sensor_msgs.msg import PointCloud2, PointField  # type: ignore
    from sensor_msgs_py.point_cloud2 import read_points  # type: ignore
except ImportError as exc:  # pragma: no cover - guarded execution
    rosbag2_py = None  # type: ignore
    PointCloud2 = None  # type: ignore
    PointField = None  # type: ignore
    deserialize_message = None  # type: ignore
    get_message = None  # type: ignore
    read_points = None  # type: ignore
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


_PCD_TYPE_MAP: Dict[int, Tuple[str, int]] = {
    # PointField datatype => (PCD TYPE, SIZE bytes)
    1: ("I", 1),  # INT8
    2: ("U", 1),  # UINT8
    3: ("I", 2),  # INT16
    4: ("U", 2),  # UINT16
    5: ("I", 4),  # INT32
    6: ("U", 4),  # UINT32
    7: ("F", 4),  # FLOAT32
    8: ("F", 8),  # FLOAT64
}


@dataclass
class ExtractionResult:
    """Aggregate data about a conversion run."""

    bag_path: Path
    output_dir: Path
    processed_messages: int
    written_files: List[Path]


def _require_ros_dependencies() -> None:
    """Ensure the script runs inside a ROS 2 environment with required packages."""

    if _IMPORT_ERROR is None:
        return
    raise RuntimeError(
        "Failed to import ROS 2 Python APIs. Make sure you have sourced the ROS 2 "
        "environment before running this script. Original error: "
        f"{_IMPORT_ERROR}"
    )


def _normalise_topic(topic: str) -> str:
    """Sanitise a ROS topic into a filesystem-friendly token."""

    cleaned = topic.strip().replace("/", "_")
    return cleaned.lstrip("_") or "pointcloud"


def _format_numeric(value: object) -> str:
    """Convert numeric values to strings that PCD readers understand."""

    if isinstance(value, float):
        if math.isnan(value):
            return "nan"
        if math.isinf(value):
            return "inf" if value > 0 else "-inf"
        return format(value, ".8f").rstrip("0").rstrip(".") or "0"
    return str(value)


def _pcd_field_entry(field: PointField) -> Tuple[str, int]:
    """Return (type, size) tuple for a point field."""

    if field.datatype not in _PCD_TYPE_MAP:
        raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
    return _PCD_TYPE_MAP[field.datatype]


def _write_pcd(message: PointCloud2, destination: Path, skip_nans: bool) -> int:
    """Write a single PointCloud2 message into a PCD file.

    Returns the number of points written.
    """

    field_names = [field.name for field in message.fields]
    points_iter = read_points(  # type: ignore[operator]
        message,
        field_names=field_names,
        skip_nans=skip_nans,
    )
    points = list(points_iter)

    type_tokens = []
    size_tokens = []
    count_tokens = []
    for field in message.fields:
        pcd_type, size = _pcd_field_entry(field)
        type_tokens.append(pcd_type)
        size_tokens.append(str(size))
        count_tokens.append(str(field.count or 1))

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="ascii", newline="\n") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write("FIELDS " + " ".join(field_names) + "\n")
        handle.write("SIZE " + " ".join(size_tokens) + "\n")
        handle.write("TYPE " + " ".join(type_tokens) + "\n")
        handle.write("COUNT " + " ".join(count_tokens) + "\n")
        handle.write(f"WIDTH {len(points)}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(points)}\n")
        handle.write("DATA ascii\n")
        for point in points:
            handle.write(" ".join(_format_numeric(value) for value in point) + "\n")

    return len(points)


def _open_reader(bag_path: Path) -> Tuple[rosbag2_py.SequentialReader, Dict[str, str]]:
    """Create a rosbag2 sequential reader and return it alongside topic mapping."""

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topics = reader.get_all_topics_and_types()
    return reader, {topic.name: topic.type for topic in topics}


def _extract_pointclouds(
    bag_path: Path,
    output_dir: Path,
    topics: Optional[Sequence[str]],
    skip_nans: bool,
    max_messages: Optional[int],
) -> ExtractionResult:
    """Iterate through the bag and persist PointCloud2 payloads as PCD files."""

    _require_ros_dependencies()

    reader, topic_type_map = _open_reader(bag_path)
    target_topics = {
        topic
        for topic, type_name in topic_type_map.items()
        if type_name == "sensor_msgs/msg/PointCloud2"
    }

    if topics:
        requested = set(topics)
        missing = requested - set(topic_type_map.keys())
        if missing:
            raise ValueError(f"Topics not found in bag: {', '.join(sorted(missing))}")
        target_topics &= requested

    if not target_topics:
        raise ValueError("No PointCloud2 topics found for extraction.")

    written_files: List[Path] = []
    processed = 0

    try:
        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()
            if topic not in target_topics:
                continue

            message_type = get_message(topic_type_map[topic])  # type: ignore[operator]
            message = deserialize_message(data, message_type)  # type: ignore[operator]
            assert isinstance(message, PointCloud2)

            sanitized_topic = _normalise_topic(topic)
            file_name = f"{sanitized_topic}_{timestamp_ns}.pcd"
            destination = output_dir / sanitized_topic / file_name

            written_points = _write_pcd(message, destination, skip_nans)
            if written_points == 0:
                destination.unlink(missing_ok=True)
                continue

            written_files.append(destination)
            processed += 1

            if max_messages is not None and processed >= max_messages:
                break
    finally:
        reader = None

    return ExtractionResult(
        bag_path=bag_path,
        output_dir=output_dir,
        processed_messages=processed,
        written_files=written_files,
    )


def _run_playback(bag_path: Path, ros2_binary: str, extra_args: Sequence[str]) -> None:
    """Invoke ros2 bag play with optional additional arguments."""

    command = [ros2_binary, "bag", "play", str(bag_path)]
    if extra_args:
        command.extend(extra_args)

    subprocess.run(command, check=True)


def _parse_arguments(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert ROS 2 rosbag2 directories into PCD files or play them back."
        )
    )
    subcommands = parser.add_subparsers(dest="command", required=True)

    extract_parser = subcommands.add_parser(
        "extract",
        help="Export PointCloud2 messages from a rosbag2 directory into PCD files.",
    )
    extract_parser.add_argument("bag_path", type=Path, help="Path to the rosbag2 directory")
    extract_parser.add_argument(
        "output_dir",
        type=Path,
        help="Directory where PCD files will be written",
    )
    extract_parser.add_argument(
        "--topics",
        nargs="+",
        help="Specific PointCloud2 topics to export (defaults to all)",
    )
    extract_parser.add_argument(
        "--skip-nans",
        action="store_true",
        help="Drop NaN points while writing PCD files",
    )
    extract_parser.add_argument(
        "--max-messages",
        type=int,
        help="Limit the number of PointCloud2 messages processed",
    )
    extract_parser.add_argument(
        "--play-after",
        action="store_true",
        help="Play the bag once extraction finishes",
    )
    extract_parser.add_argument(
        "--ros2-binary",
        default="ros2",
        help="ros2 executable to use when --play-after is supplied",
    )
    extract_parser.add_argument(
        "--ros2-args",
        nargs=argparse.REMAINDER,
        help="Additional arguments passed to ros2 bag play",
    )

    play_parser = subcommands.add_parser(
        "play",
        help="Replay a rosbag2 directory using ros2 bag play.",
    )
    play_parser.add_argument("bag_path", type=Path, help="Path to the rosbag2 directory")
    play_parser.add_argument(
        "--ros2-binary",
        default="ros2",
        help="ros2 executable to invoke",
    )
    play_parser.add_argument(
        "--ros2-args",
        nargs=argparse.REMAINDER,
        help="Additional ros2 bag play arguments",
    )

    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_arguments(argv)

    if args.command == "extract":
        bag_path = args.bag_path.expanduser().resolve()
        output_dir = args.output_dir.expanduser().resolve()

        if not bag_path.exists():
            raise FileNotFoundError(f"Bag directory not found: {bag_path}")
        if not (bag_path / "metadata.yaml").exists():
            raise FileNotFoundError(
                f"metadata.yaml missing inside bag directory: {bag_path}"
            )

        output_dir.mkdir(parents=True, exist_ok=True)

        result = _extract_pointclouds(
            bag_path=bag_path,
            output_dir=output_dir,
            topics=args.topics,
            skip_nans=bool(args.skip_nans),
            max_messages=args.max_messages,
        )

        print(
            f"Extracted {result.processed_messages} PointCloud2 messages into "
            f"{len(result.written_files)} PCD files under {result.output_dir}"
        )

        if args.play_after:
            extra_args = args.ros2_args or []
            _run_playback(bag_path, args.ros2_binary, extra_args)

        return 0

    if args.command == "play":
        bag_path = args.bag_path.expanduser().resolve()
        if not bag_path.exists():
            raise FileNotFoundError(f"Bag directory not found: {bag_path}")
        extra_args = args.ros2_args or []
        _run_playback(bag_path, args.ros2_binary, extra_args)
        return 0

    raise ValueError(f"Unknown command: {args.command}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except Exception as exc:  # pragma: no cover - top-level guard
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
