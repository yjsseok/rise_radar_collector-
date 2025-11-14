#!/usr/bin/env python3
"""
ROS2 Bag Player for SENSR Radar Data
Main entry point with GUI file/folder selection
"""

import sys
import logging
import argparse
from pathlib import Path
from typing import Optional

try:
    import open3d as o3d
    from open3d.visualization import gui
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    o3d = None
    gui = None

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.bag_reader import BagReader
from src.player_gui import BagPlayerGUI
from src.utils import (
    find_bag_files, find_latest_bag, select_folder_dialog,
    select_file_dialog, load_config, get_default_config,
    get_bag_info
)


def setup_logging(verbose: bool = False):
    """Setup logging configuration"""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )


def print_bag_info(bag_path: Path, reader: BagReader):
    """Print bag file information"""
    print("\n" + "=" * 70)
    print("📊 Bag Information")
    print("=" * 70)
    print(f"Path: {bag_path}")
    print(f"Name: {bag_path.name}")

    topics_info = reader.get_all_topics_info()

    if topics_info:
        print(f"\nTopics ({len(topics_info)}):")
        for topic, info in topics_info.items():
            print(f"  • {topic}")
            print(f"    - Type: {info['type']}")
            print(f"    - Messages: {info['count']}")
            if info['duration'] > 0:
                print(f"    - Duration: {info['duration']:.2f}s")

        total_messages = reader.get_message_count()
        print(f"\nTotal Messages: {total_messages}")
    else:
        print("\n⚠️  No PointCloud2 topics found in bag file!")

    print("=" * 70 + "\n")


def select_bag_interactive(default_dir: Optional[Path] = None) -> Optional[Path]:
    """
    Interactive bag selection with GUI

    Args:
        default_dir: Default directory to start from

    Returns:
        Selected bag path or None
    """
    print("\n🔍 Opening folder selection dialog...")
    print("Please select a bag directory or a folder containing bag files.")

    # Determine initial directory
    if default_dir and default_dir.exists():
        initial_dir = str(default_dir)
    else:
        # Try radar_v3/output
        radar_output = Path(__file__).parent.parent.parent / "radar_v3" / "output"
        if radar_output.exists():
            initial_dir = str(radar_output)
        else:
            initial_dir = str(Path.home())

    # Show selection dialog
    selected = select_file_dialog(
        initial_dir=initial_dir,
        title="Select ROS2 Bag Directory"
    )

    if selected is None:
        print("❌ No bag selected. Exiting.")
        return None

    print(f"✅ Selected: {selected}")
    return selected


def list_available_bags(directory: Path) -> Optional[Path]:
    """
    List available bags in directory and let user choose

    Args:
        directory: Directory to search

    Returns:
        Selected bag path or None
    """
    print(f"\n🔍 Searching for bag files in: {directory}")

    bags = find_bag_files(directory, recursive=True)

    if not bags:
        print("❌ No bag files found!")
        return None

    print(f"\n📁 Found {len(bags)} bag file(s):")
    for i, bag in enumerate(bags, 1):
        print(f"  {i}. {bag.name}")

    # Auto-select if only one bag
    if len(bags) == 1:
        print(f"\n✅ Auto-selected: {bags[0].name}")
        return bags[0]

    # Let user choose
    while True:
        try:
            choice = input(f"\nSelect bag (1-{len(bags)}) or 'q' to quit: ").strip()

            if choice.lower() == 'q':
                return None

            idx = int(choice) - 1
            if 0 <= idx < len(bags):
                return bags[idx]
            else:
                print(f"Invalid choice. Please enter 1-{len(bags)}")

        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\n\n❌ Cancelled.")
            return None


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="ROS2 Bag Player for SENSR Radar Point Clouds"
    )
    parser.add_argument(
        'bag_path',
        type=Path,
        nargs='?',
        help='Path to bag directory (optional, will show GUI selector if not provided)'
    )
    parser.add_argument(
        '--auto-latest',
        action='store_true',
        help='Automatically select the latest bag file'
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=Path(__file__).parent / 'config' / 'player_config.yaml',
        help='Path to config file'
    )
    parser.add_argument(
        '--search-dir',
        type=Path,
        help='Directory to search for bag files'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose logging'
    )

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)

    # Check dependencies
    if not OPEN3D_AVAILABLE:
        print("❌ Error: Open3D not found!")
        print("Install with: pip install open3d")
        return 1

    # Load config
    config = load_config(args.config) if args.config.exists() else get_default_config()

    # Determine bag path
    bag_path = None

    if args.bag_path:
        # Path provided via command line
        bag_path = args.bag_path.expanduser().resolve()

        if not bag_path.exists():
            print(f"❌ Error: Path not found: {bag_path}")
            return 1

        # Check if it's a valid bag
        if not (bag_path / 'metadata.yaml').exists():
            # Try to find bags inside
            bags = find_bag_files(bag_path, recursive=False)
            if bags:
                bag_path = bags[0]
            else:
                print(f"❌ Error: No valid bag found at: {bag_path}")
                return 1

    elif args.auto_latest:
        # Auto-select latest bag
        search_dir = args.search_dir or Path(config['search']['default_directory'])
        search_dir = (Path(__file__).parent / search_dir).resolve()

        if not search_dir.exists():
            print(f"❌ Error: Search directory not found: {search_dir}")
            return 1

        print(f"🔍 Searching for latest bag in: {search_dir}")
        bag_path = find_latest_bag(search_dir)

        if not bag_path:
            print(f"❌ No bag files found in: {search_dir}")
            return 1

        print(f"✅ Found latest bag: {bag_path.name}")

    else:
        # Interactive selection with GUI
        default_dir = None
        if args.search_dir:
            default_dir = args.search_dir.expanduser().resolve()
        else:
            default_search = config['search'].get('default_directory', '../radar_v3/output')
            default_dir = (Path(__file__).parent / default_search).resolve()

        bag_path = select_bag_interactive(default_dir)

        if not bag_path:
            return 1

    # Verify bag path
    if not bag_path or not bag_path.exists():
        print(f"❌ Error: Bag path not found: {bag_path}")
        return 1

    # Open bag
    print(f"\n📂 Opening bag: {bag_path.name}")

    reader = BagReader()
    if not reader.open(bag_path):
        print("❌ Failed to open bag file!")
        return 1

    # Print bag info
    print_bag_info(bag_path, reader)

    # Check if there are any messages
    if reader.get_message_count() == 0:
        print("❌ Error: No PointCloud2 messages found in bag!")
        reader.close()
        return 1

    # Start GUI
    try:
        print("🎬 Starting player...")
        print("\nKeyboard shortcuts:")
        print("  Space    - Play/Pause")
        print("  ←/→      - Previous/Next frame")
        print("  Home/End - First/Last frame")
        print("")

        # Initialize Open3D GUI app
        gui.Application.instance.initialize()

        # Create player
        player = BagPlayerGUI(reader, config)

        # Run
        gui.Application.instance.run()

        print("\n✅ Player closed.")

    except KeyboardInterrupt:
        print("\n\n⌨️  Interrupted by user.")
    except Exception as e:
        logger.error(f"Error running player: {e}", exc_info=True)
        return 1
    finally:
        reader.close()

    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⌨️  Interrupted by user.")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
