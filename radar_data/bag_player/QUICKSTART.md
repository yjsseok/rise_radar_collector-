# Quick Start Guide

🚀 Get started with ROS2 Bag Player in 3 steps!

## Step 1: Install

```bash
cd radar_data/bag_player
./install.sh
```

The install script will:
- ✅ Check ROS2 installation
- ✅ Install ROS2 dependencies (rosbag2-py, sensor-msgs, etc.)
- ✅ Install Python packages (Open3D, NumPy, PyYAML)
- ✅ Create run script

## Step 2: Run

### Option A: GUI File Picker (Easiest)

```bash
./run.sh
```

This opens a folder selection dialog. Navigate to a bag directory or folder containing bags.

### Option B: Command Line

```bash
# With specific bag path
./run.sh /path/to/bag/directory

# Auto-select latest bag
./run.sh --auto-latest
```

## Step 3: Play!

Once the player opens:

1. **Select Topics**: Check the topics you want to visualize in the "Topics" panel
2. **Press Space**: Start playback
3. **Use Slider**: Scrub through frames
4. **Adjust Speed**: Use speed slider for faster/slower playback

That's it! 🎉

## Example: Play radar_v3 Output

```bash
cd /home/user/rise_radar_collector-/radar_data/bag_player

# Run with GUI picker
./run.sh

# In dialog, select:
# ../../radar_v3/output/sensr_data_YYYYMMDD_HHMMSS

# Or directly:
./run.sh ../../radar_v3/output/sensr_data_20251114_103045
```

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play/Pause |
| `←` `→` | Previous/Next frame |
| `Home` `End` | First/Last frame |

## Troubleshooting

### "ROS2 environment not sourced"

```bash
source /opt/ros/humble/setup.bash
./run.sh
```

Or add to `~/.bashrc`:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

### "Open3D not found"

```bash
pip install open3d
```

### Need more help?

See [README.md](README.md) for detailed documentation.
