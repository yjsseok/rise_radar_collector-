#!/usr/bin/env python3
"""ROS2 Bag Reader for point cloud playback"""

import logging
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
from collections import defaultdict

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import PointCloud2
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rosbag2_py = None
    deserialize_message = None
    get_message = None
    PointCloud2 = None


class BagReader:
    """ROS2 Bag file reader for PointCloud2 messages"""

    def __init__(self):
        """Initialize BagReader"""
        if not ROS2_AVAILABLE:
            raise ImportError(
                "ROS2 packages not found. Please install: "
                "rosbag2-py, rclpy, sensor-msgs, rosidl-runtime-py"
            )

        self.logger = logging.getLogger(__name__)
        self.bag_path: Optional[Path] = None
        self.reader: Optional[rosbag2_py.SequentialReader] = None
        self.topic_type_map: Dict[str, str] = {}
        self.pointcloud_topics: List[str] = []
        self.messages_by_topic: Dict[str, List[Tuple[int, bytes, int]]] = defaultdict(list)
        self.all_messages: List[Tuple[str, bytes, int]] = []  # (topic, data, timestamp)
        self.is_open = False

    def open(self, bag_path: Path) -> bool:
        """
        Open a ROS2 bag file

        Args:
            bag_path: Path to bag directory

        Returns:
            True if successful, False otherwise
        """
        try:
            self.bag_path = bag_path

            # Create reader
            storage_options = rosbag2_py.StorageOptions(
                uri=str(bag_path),
                storage_id='sqlite3'
            )
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )

            self.reader = rosbag2_py.SequentialReader()
            self.reader.open(storage_options, converter_options)

            # Get topic information
            topics = self.reader.get_all_topics_and_types()
            self.topic_type_map = {topic.name: topic.type for topic in topics}

            # Find PointCloud2 topics
            self.pointcloud_topics = [
                topic for topic, type_name in self.topic_type_map.items()
                if type_name == 'sensor_msgs/msg/PointCloud2'
            ]

            # Load all messages into memory for fast access
            self._load_all_messages()

            self.is_open = True
            self.logger.info(f"Opened bag: {bag_path}")
            self.logger.info(f"Found {len(self.pointcloud_topics)} PointCloud2 topics")
            self.logger.info(f"Loaded {len(self.all_messages)} total messages")

            return True

        except Exception as e:
            self.logger.error(f"Failed to open bag: {e}")
            return False

    def _load_all_messages(self):
        """Load all messages into memory for fast indexed access"""
        self.all_messages.clear()
        self.messages_by_topic.clear()

        if not self.reader:
            return

        # Reset reader
        self.reader.seek(0)

        # Read all messages
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()

            # Only store PointCloud2 messages
            if topic in self.pointcloud_topics:
                self.all_messages.append((topic, data, timestamp))

                # Also store by topic for topic-specific access
                topic_index = len(self.messages_by_topic[topic])
                self.messages_by_topic[topic].append((topic_index, data, timestamp))

        # Sort all messages by timestamp
        self.all_messages.sort(key=lambda x: x[2])

        self.logger.debug(f"Loaded {len(self.all_messages)} messages in memory")

    def get_topics(self) -> List[str]:
        """
        Get list of PointCloud2 topics

        Returns:
            List of topic names
        """
        return self.pointcloud_topics.copy()

    def get_message_count(self, topic: Optional[str] = None) -> int:
        """
        Get total message count for a topic or all messages

        Args:
            topic: Topic name, or None for all messages

        Returns:
            Number of messages
        """
        if topic is None:
            return len(self.all_messages)
        return len(self.messages_by_topic.get(topic, []))

    def read_message(self, index: int, topic: Optional[str] = None) -> Optional[PointCloud2]:
        """
        Read a message by index

        Args:
            index: Message index
            topic: Topic name, or None to read from all messages chronologically

        Returns:
            PointCloud2 message or None
        """
        try:
            if topic is None:
                # Read from all messages
                if index < 0 or index >= len(self.all_messages):
                    return None
                topic_name, data, timestamp = self.all_messages[index]
            else:
                # Read from specific topic
                if topic not in self.messages_by_topic:
                    return None
                messages = self.messages_by_topic[topic]
                if index < 0 or index >= len(messages):
                    return None
                _, data, timestamp = messages[index]
                topic_name = topic

            # Deserialize message
            message_type = get_message(self.topic_type_map[topic_name])
            message = deserialize_message(data, message_type)

            return message

        except Exception as e:
            self.logger.error(f"Failed to read message at index {index}: {e}")
            return None

    def get_timestamp(self, index: int, topic: Optional[str] = None) -> Optional[int]:
        """
        Get timestamp for a message

        Args:
            index: Message index
            topic: Topic name, or None for all messages

        Returns:
            Timestamp in nanoseconds or None
        """
        try:
            if topic is None:
                if index < 0 or index >= len(self.all_messages):
                    return None
                return self.all_messages[index][2]
            else:
                if topic not in self.messages_by_topic:
                    return None
                messages = self.messages_by_topic[topic]
                if index < 0 or index >= len(messages):
                    return None
                return messages[index][2]
        except:
            return None

    def get_topic_at_index(self, index: int) -> Optional[str]:
        """
        Get topic name at global message index

        Args:
            index: Global message index

        Returns:
            Topic name or None
        """
        if index < 0 or index >= len(self.all_messages):
            return None
        return self.all_messages[index][0]

    def get_all_topics_info(self) -> Dict[str, Any]:
        """
        Get detailed information about all topics

        Returns:
            Dictionary with topic information
        """
        info = {}
        for topic in self.pointcloud_topics:
            count = len(self.messages_by_topic[topic])
            if count > 0:
                first_ts = self.messages_by_topic[topic][0][2]
                last_ts = self.messages_by_topic[topic][-1][2]
                duration = (last_ts - first_ts) / 1e9  # Convert to seconds
            else:
                first_ts = last_ts = duration = 0

            info[topic] = {
                'type': self.topic_type_map[topic],
                'count': count,
                'first_timestamp': first_ts,
                'last_timestamp': last_ts,
                'duration': duration,
            }
        return info

    def close(self):
        """Close the bag reader and free resources"""
        if self.reader:
            self.reader = None

        self.all_messages.clear()
        self.messages_by_topic.clear()
        self.is_open = False
        self.logger.info("Closed bag reader")

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()
