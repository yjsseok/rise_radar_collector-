#!/bin/bash
# Installation script for ROS2 Bag Player

set -e

echo "========================================="
echo "ROS2 Bag Player Installation"
echo "========================================="
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 environment not sourced!"
    echo ""
    echo "Please source ROS2 first:"
    echo "  source /opt/ros/humble/setup.bash"
    echo ""
    echo "Or add to ~/.bashrc:"
    echo "  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
    echo ""
    exit 1
fi

echo "✅ ROS2 $ROS_DISTRO detected"
echo ""

# Install ROS2 dependencies
echo "📦 Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-rosbag2-py \
    ros-$ROS_DISTRO-rclpy \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-sensor-msgs-py \
    ros-$ROS_DISTRO-rosidl-runtime-py \
    python3-tk

echo ""
echo "✅ ROS2 dependencies installed"
echo ""

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip install open3d numpy PyYAML

echo ""
echo "✅ Python dependencies installed"
echo ""

# Create run script
cat > run.sh << 'EOF'
#!/bin/bash
# Quick launcher for ROS2 Bag Player

# Source ROS2
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# Run player
python3 rosbag_player.py "$@"
EOF

chmod +x run.sh

echo "========================================="
echo "✅ Installation Complete!"
echo "========================================="
echo ""
echo "Quick Start:"
echo "  1. Run with GUI file picker:"
echo "     ./run.sh"
echo ""
echo "  2. Run with specific bag:"
echo "     ./run.sh /path/to/bag"
echo ""
echo "  3. Auto-select latest bag:"
echo "     ./run.sh --auto-latest"
echo ""
echo "See README.md for more information."
echo ""
