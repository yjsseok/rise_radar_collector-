# ROS2 Bag Player for SENSR Radar

🎬 GUI-based ROS2 bag player for visualizing SENSR radar point cloud data

## ✨ Features

- **Direct Bag Playback**: Play ROS2 bag files directly without conversion to PCD
- **Multi-Topic Support**: View multiple PointCloud2 topics simultaneously with checkboxes
- **Interactive GUI**: File/folder selection with native dialogs
- **Playback Controls**: Play, Pause, Stop, Loop, Speed control
- **Frame Navigation**: Slider, keyboard shortcuts (Space, ←/→, Home/End)
- **Point Cloud Visualization**: Open3D-based 3D viewer with intensity colormaps
- **Automatic Topic Detection**: Finds all PointCloud2 topics in bag files

## 📁 Directory Structure

```
bag_player/
├── rosbag_player.py              # Main entry point
├── requirements.txt              # Python dependencies
├── README.md                     # This file
├── config/
│   └── player_config.yaml        # Player configuration
└── src/
    ├── __init__.py
    ├── bag_reader.py             # ROS2 bag reader
    ├── pointcloud_converter.py   # PointCloud2 to Open3D converter
    ├── player_gui.py             # GUI player implementation
    └── utils.py                  # Utility functions
```

## 🚀 Installation

### Prerequisites

1. **ROS2 Humble** (or compatible version)
2. **Python 3.8+**
3. **WSL/Ubuntu** (for ROS2 support)

### Step 1: Install ROS2 Dependencies

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install required ROS2 packages
sudo apt update
sudo apt install -y \
    ros-humble-rosbag2-py \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-rosidl-runtime-py \
    python3-tk
```

### Step 2: Install Python Dependencies

```bash
cd radar_data/bag_player

# Install Python packages
pip install open3d numpy PyYAML
```

## 📖 Usage

### Method 1: GUI File Selection (Recommended)

Simply run without arguments to open a file selection dialog:

```bash
cd radar_data/bag_player
python rosbag_player.py
```

This will:
1. Open a folder selection dialog
2. Default to `radar_v3/output` if available
3. Let you browse and select a bag directory

### Method 2: Command Line Path

Provide the bag path directly:

```bash
python rosbag_player.py /path/to/bag/directory

# Example with radar_v3 output
python rosbag_player.py ../../radar_v3/output/sensr_data_20251114_103045
```

### Method 3: Auto-Select Latest

Automatically select the most recent bag file:

```bash
python rosbag_player.py --auto-latest

# Or specify search directory
python rosbag_player.py --auto-latest --search-dir ../../radar_v3/output
```

## 🎮 Controls

### GUI Controls

- **Topic Checkboxes**: Select which topics to visualize
  - Check/uncheck individual topics
  - "All" button: Select all topics
  - "None" button: Deselect all topics
- **Play/Pause/Stop Buttons**: Control playback
- **Loop Checkbox**: Enable continuous loop playback
- **Frame Slider**: Scrub through frames
- **Speed Slider**: Adjust playback speed (0.1x - 10x)

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play/Pause toggle |
| `←` | Previous frame |
| `→` | Next frame |
| `Home` | Jump to first frame |
| `End` | Jump to last frame |

### Mouse Controls (3D Viewer)

- **Left drag**: Rotate camera
- **Right drag**: Pan camera
- **Scroll**: Zoom in/out

## ⚙️ Configuration

Edit `config/player_config.yaml` to customize:

```yaml
player:
  default_fps: 10.0           # Default playback FPS
  default_speed: 1.0          # Default playback speed
  loop_playback: false        # Auto-loop on by default
  auto_play: false            # Start playing immediately

visualization:
  point_size: 2.0             # Point size in pixels
  background_color: [0.0, 0.0, 0.0, 1.0]  # Black background
  show_axes: true             # Show coordinate axes
  intensity_colormap: "viridis"  # Color map: viridis, jet, hot, cool

window:
  width: 1280                 # Window width
  height: 720                 # Window height
  title: "ROS2 Bag Player - SENSR Radar"

search:
  default_directory: "../../radar_v3/output"  # Default search path
  recursive_search: true      # Search recursively
  auto_select_latest: false   # Auto-select latest bag
```

## 📊 Features in Detail

### Multi-Topic Visualization

The player can display multiple PointCloud2 topics simultaneously:

1. All topics are listed in the collapsible "Topics" panel
2. Check the topics you want to visualize
3. Point clouds from selected topics are merged and displayed together
4. Each topic's closest message to the current timestamp is shown

### Intelligent Time Synchronization

When viewing multiple topics:
- Player shows messages closest to the current timestamp
- Handles topics with different message rates
- Maintains temporal alignment across topics

### Memory-Efficient Playback

- All messages are indexed in memory for fast access
- No intermediate file conversion required
- Direct bag reading with rosbag2-py

## 🔍 Troubleshooting

### "ROS2 packages not found"

**Solution**: Source ROS2 environment before running:

```bash
source /opt/ros/humble/setup.bash
python rosbag_player.py
```

Add to `~/.bashrc` for automatic sourcing:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### "Open3D not found"

**Solution**: Install Open3D:

```bash
pip install open3d
```

### "No PointCloud2 topics found"

**Problem**: Bag file doesn't contain PointCloud2 messages

**Solution**: Verify bag contents:

```bash
ros2 bag info /path/to/bag
```

### GUI not showing

**Solution**: Install tkinter:

```bash
sudo apt install python3-tk
```

### Empty/Black screen

**Possible causes**:
1. No topics selected (check topic checkboxes)
2. Empty point cloud messages
3. Points outside camera view (try zooming out)

## 📝 Example Workflow

### Playing radar_v3 Output

```bash
# 1. Navigate to bag_player directory
cd /home/user/rise_radar_collector-/radar_data/bag_player

# 2. Source ROS2 (if not in .bashrc)
source /opt/ros/humble/setup.bash

# 3. Run player with GUI selection
python rosbag_player.py

# 4. In the dialog, navigate to:
#    ../../radar_v3/output/sensr_data_YYYYMMDD_HHMMSS

# 5. The player will:
#    - List all PointCloud2 topics
#    - Show bag statistics
#    - Open GUI with first topic selected

# 6. Use checkboxes to select which topics to view
#    - /sensr/pointcloud
#    - /sensr/output_data
#    - etc.

# 7. Press Space to play, or use slider to scrub
```

## 🆚 Comparison with rostopcd.py + pcdplayer.py

| Feature | Old Method | New Bag Player |
|---------|-----------|----------------|
| Conversion | Required (bag → PCD) | ❌ Not needed |
| Disk Usage | Duplicates data as PCD files | ✅ Only reads bag |
| Speed | Slow (file I/O) | ✅ Fast (memory) |
| Multi-topic | One at a time | ✅ Multiple simultaneous |
| File Selection | Command line only | ✅ GUI dialog |
| Timestamps | Parsed from filename | ✅ Native bag metadata |

## 🎯 Future Enhancements

Potential features for future versions:

- [ ] Screenshot/video recording
- [ ] Point cloud filtering (distance, intensity)
- [ ] Multiple bag playback
- [ ] Export selected frames to PCD
- [ ] Real-time statistics display
- [ ] Custom camera presets

## 📄 License

Part of the SENSR Radar Collector project.

## 🤝 Contributing

If you find bugs or have feature requests, please create an issue in the main repository.

---

**Happy bag playing! 🎬**
