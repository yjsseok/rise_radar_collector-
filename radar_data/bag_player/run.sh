#!/bin/bash
# Quick launcher for ROS2 Bag Player

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "⚠️  ROS2 not found!"
        echo "Please install ROS2 Humble or source your ROS2 installation."
        exit 1
    fi
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run player
cd "$SCRIPT_DIR"
python3 rosbag_player.py "$@"
