#!/usr/bin/env python3
"""GUI Player for ROS2 Bag Point Clouds"""

import threading
import time
import logging
from typing import Dict, List, Optional, Set
from pathlib import Path

try:
    import open3d as o3d
    from open3d.visualization import gui, rendering
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    o3d = None
    gui = None
    rendering = None

from .bag_reader import BagReader
from .pointcloud_converter import PointCloudConverter
from .utils import format_timestamp


class BagPlayerGUI:
    """Open3D GUI-based ROS2 Bag Player with multi-topic support"""

    def __init__(self, bag_reader: BagReader, config: Dict):
        """
        Initialize GUI Player

        Args:
            bag_reader: Opened BagReader instance
            config: Configuration dictionary
        """
        if not OPEN3D_AVAILABLE:
            raise ImportError("Open3D not found. Install with: pip install open3d")

        self.logger = logging.getLogger(__name__)
        self.bag_reader = bag_reader
        self.config = config

        # Player state
        self.current_index = 0
        self.playing = False
        self.loop = config['player'].get('loop_playback', False)
        self.speed = config['player'].get('default_speed', 1.0)
        self.fps = config['player'].get('default_fps', 10.0)

        # Topic selection state
        self.available_topics = bag_reader.get_topics()
        self.selected_topics: Set[str] = set(self.available_topics[:1])  # Start with first topic

        # Point cloud converter
        colormap = config['visualization'].get('intensity_colormap', 'viridis')
        self.converter = PointCloudConverter(colormap=colormap)

        # GUI state
        self.camera_prepared = False
        self.slider_guard = False

        # Threading
        self.stop_event = threading.Event()
        self.timer_thread = None

        # Initialize GUI
        self._init_gui()

        # Show first frame
        self._show_frame(update_slider=True)

    def _init_gui(self):
        """Initialize Open3D GUI"""
        vis_config = self.config['visualization']
        win_config = self.config['window']

        # Create application and window
        app = gui.Application.instance
        self.window = app.create_window(
            win_config.get('title', 'ROS2 Bag Player'),
            win_config.get('width', 1280),
            win_config.get('height', 720)
        )

        # Create 3D scene
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)

        bg_color = vis_config.get('background_color', [0, 0, 0, 1])
        self.scene.scene.set_background(bg_color)

        show_axes = vis_config.get('show_axes', True)
        self.scene.scene.show_axes(show_axes)

        # Material for point clouds
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.material.point_size = vis_config.get('point_size', 2.0)

        # Create UI panels
        self._create_control_panel()

        # Add widgets to window
        self.window.add_child(self.scene)
        self.window.add_child(self.panel)

        # Set layout callback
        self.window.set_on_layout(self._on_layout)

        # Set keyboard callback
        self.scene.set_on_key(self._on_key_event)

        # Set close callback
        self.window.set_on_close(self._on_close)

        # Start timer thread
        self.timer_thread = threading.Thread(target=self._timer_loop, daemon=True)
        self.timer_thread.start()

    def _create_control_panel(self):
        """Create control panel with all UI elements"""
        margin = gui.Margins(8, 8, 8, 8)
        spacing = 6

        self.panel = gui.Vert(spacing, margin)

        # Status label
        self.status_label = gui.Label("")
        self.panel.add_child(self.status_label)

        # Topic selection panel
        self._create_topic_panel()
        self.panel.add_child(self.topic_panel)

        # Playback slider
        total_messages = self.bag_reader.get_message_count()
        self.slider = gui.Slider(gui.Slider.INT)
        self.slider.set_limits(0, max(0, total_messages - 1))
        self.slider.set_on_value_changed(self._on_slider_changed)
        self.panel.add_child(self.slider)

        # Playback controls
        controls = gui.Horiz(8, margin)

        self.play_button = gui.Button("▶ Play")
        self.play_button.set_on_clicked(self._on_play_clicked)
        controls.add_child(self.play_button)

        self.pause_button = gui.Button("⏸ Pause")
        self.pause_button.set_on_clicked(self._on_pause_clicked)
        controls.add_child(self.pause_button)

        self.stop_button = gui.Button("⏹ Stop")
        self.stop_button.set_on_clicked(self._on_stop_clicked)
        controls.add_child(self.stop_button)

        self.loop_checkbox = gui.Checkbox("Loop")
        self.loop_checkbox.checked = self.loop
        self.loop_checkbox.set_on_checked(self._on_loop_changed)
        controls.add_child(self.loop_checkbox)

        self.panel.add_child(controls)

        # Speed control
        speed_panel = gui.Horiz(8, margin)
        speed_panel.add_child(gui.Label("Speed:"))

        self.speed_slider = gui.Slider(gui.Slider.DOUBLE)
        self.speed_slider.set_limits(0.1, 10.0)
        self.speed_slider.double_value = self.speed
        self.speed_slider.set_on_value_changed(self._on_speed_changed)
        speed_panel.add_child(self.speed_slider)

        self.speed_label = gui.Label(f"{self.speed:.1f}x")
        speed_panel.add_child(self.speed_label)

        self.panel.add_child(speed_panel)

    def _create_topic_panel(self):
        """Create topic selection panel with checkboxes"""
        self.topic_panel = gui.CollapsableVert("Topics", 6, gui.Margins(8, 4, 8, 4))
        self.topic_panel.set_is_open(True)

        self.topic_checkboxes = {}

        for topic in self.available_topics:
            checkbox = gui.Checkbox(topic)
            checkbox.checked = topic in self.selected_topics

            # Create closure to capture topic
            def make_callback(t):
                return lambda checked: self._on_topic_checkbox_changed(t, checked)

            checkbox.set_on_checked(make_callback(topic))

            self.topic_checkboxes[topic] = checkbox
            self.topic_panel.add_child(checkbox)

        # Add "Select All" / "Deselect All" buttons
        select_panel = gui.Horiz(4)

        select_all_btn = gui.Button("All")
        select_all_btn.set_on_clicked(self._on_select_all_topics)
        select_panel.add_child(select_all_btn)

        deselect_all_btn = gui.Button("None")
        deselect_all_btn.set_on_clicked(self._on_deselect_all_topics)
        select_panel.add_child(deselect_all_btn)

        self.topic_panel.add_child(select_panel)

    def _on_topic_checkbox_changed(self, topic: str, checked: bool):
        """Handle topic checkbox change"""
        if checked:
            self.selected_topics.add(topic)
        else:
            self.selected_topics.discard(topic)

        # Update visualization
        self._show_frame(update_slider=False)

    def _on_select_all_topics(self):
        """Select all topics"""
        self.selected_topics = set(self.available_topics)
        for topic, checkbox in self.topic_checkboxes.items():
            checkbox.checked = True
        self._show_frame(update_slider=False)

    def _on_deselect_all_topics(self):
        """Deselect all topics"""
        self.selected_topics.clear()
        for checkbox in self.topic_checkboxes.values():
            checkbox.checked = False
        self._show_frame(update_slider=False)

    def _show_frame(self, update_slider: bool = True):
        """Show current frame"""
        try:
            # Get current message
            current_topic = self.bag_reader.get_topic_at_index(self.current_index)
            timestamp = self.bag_reader.get_timestamp(self.current_index)

            # Collect all point clouds from selected topics at this timestamp
            clouds_to_show = []

            # Read messages from all selected topics
            for topic in self.selected_topics:
                # Find the closest message in this topic to current timestamp
                topic_msg = self._find_closest_message(topic, timestamp)
                if topic_msg:
                    cloud = self.converter.convert(topic_msg)
                    if not cloud.is_empty():
                        clouds_to_show.append(cloud)

            # Clear existing geometry
            self.scene.scene.clear_geometry()

            # Merge and show clouds
            if clouds_to_show:
                if len(clouds_to_show) == 1:
                    merged_cloud = clouds_to_show[0]
                else:
                    merged_cloud = self.converter.merge_point_clouds(clouds_to_show)

                self.scene.scene.add_geometry("pointcloud", merged_cloud, self.material)

                # Setup camera on first frame
                if not self.camera_prepared:
                    bounds = merged_cloud.get_axis_aligned_bounding_box()
                    if bounds.get_max_extent() > 0:
                        self.scene.setup_camera(60.0, bounds, bounds.get_center())
                        self.camera_prepared = True

                point_count = len(merged_cloud.points)
            else:
                point_count = 0

            # Update status
            total = self.bag_reader.get_message_count()
            status = f"Frame: {self.current_index + 1}/{total}"

            if timestamp:
                status += f" | Time: {format_timestamp(timestamp)}"

            status += f" | Points: {point_count:,}"

            if current_topic:
                status += f" | Topic: {current_topic}"

            topics_str = ", ".join(sorted(self.selected_topics)) if self.selected_topics else "None"
            status += f"\nSelected: {topics_str}"

            self.status_label.text = status

            # Update slider
            if update_slider and not self.slider_guard:
                self.slider_guard = True
                self.slider.int_value = self.current_index
                self.slider_guard = False

        except Exception as e:
            self.logger.error(f"Error showing frame: {e}")

    def _find_closest_message(self, topic: str, target_timestamp: Optional[int]) -> Optional[object]:
        """
        Find message in topic closest to target timestamp

        Args:
            topic: Topic name
            target_timestamp: Target timestamp in nanoseconds

        Returns:
            PointCloud2 message or None
        """
        if not target_timestamp:
            return None

        # Get all messages for this topic
        msg_count = self.bag_reader.get_message_count(topic)
        if msg_count == 0:
            return None

        # Binary search for closest timestamp
        best_idx = 0
        best_diff = abs(self.bag_reader.get_timestamp(0, topic) - target_timestamp)

        for idx in range(msg_count):
            ts = self.bag_reader.get_timestamp(idx, topic)
            if ts is None:
                continue

            diff = abs(ts - target_timestamp)
            if diff < best_diff:
                best_diff = diff
                best_idx = idx

        return self.bag_reader.read_message(best_idx, topic)

    def _on_slider_changed(self, value: float):
        """Handle slider value change"""
        if self.slider_guard:
            return

        self.playing = False
        self.current_index = max(0, min(int(round(value)), self.bag_reader.get_message_count() - 1))
        self._show_frame(update_slider=False)

    def _on_play_clicked(self):
        """Handle play button click"""
        self.playing = True

    def _on_pause_clicked(self):
        """Handle pause button click"""
        self.playing = False

    def _on_stop_clicked(self):
        """Handle stop button click"""
        self.playing = False
        self.current_index = 0
        self._show_frame(update_slider=True)

    def _on_loop_changed(self, checked: bool):
        """Handle loop checkbox change"""
        self.loop = checked

    def _on_speed_changed(self, value: float):
        """Handle speed slider change"""
        self.speed = value
        self.speed_label.text = f"{self.speed:.1f}x"

    def _timer_loop(self):
        """Timer loop for playback"""
        while not self.stop_event.wait(0.01):
            if self.playing:
                interval = (1.0 / self.fps) / self.speed
                time.sleep(interval)
                gui.Application.instance.post_to_main_thread(
                    self.window, self._advance_frame
                )

    def _advance_frame(self):
        """Advance to next frame"""
        if not self.playing:
            return

        self.current_index += 1
        total = self.bag_reader.get_message_count()

        if self.current_index >= total:
            if self.loop:
                self.current_index = 0
            else:
                self.current_index = total - 1
                self.playing = False

        self._show_frame(update_slider=True)

    def _on_key_event(self, event: gui.KeyEvent) -> gui.EventCallbackResult:
        """Handle keyboard events"""
        if event.type != gui.KeyEvent.DOWN:
            return gui.EventCallbackResult.IGNORED

        if event.key == gui.KeyName.SPACE:
            self.playing = not self.playing
            return gui.EventCallbackResult.HANDLED

        elif event.key == gui.KeyName.LEFT:
            self.playing = False
            self.current_index = max(0, self.current_index - 1)
            self._show_frame(update_slider=True)
            return gui.EventCallbackResult.HANDLED

        elif event.key == gui.KeyName.RIGHT:
            self.playing = False
            total = self.bag_reader.get_message_count()
            self.current_index = min(total - 1, self.current_index + 1)
            self._show_frame(update_slider=True)
            return gui.EventCallbackResult.HANDLED

        elif event.key == gui.KeyName.HOME:
            self.playing = False
            self.current_index = 0
            self._show_frame(update_slider=True)
            return gui.EventCallbackResult.HANDLED

        elif event.key == gui.KeyName.END:
            self.playing = False
            total = self.bag_reader.get_message_count()
            self.current_index = total - 1
            self._show_frame(update_slider=True)
            return gui.EventCallbackResult.HANDLED

        return gui.EventCallbackResult.IGNORED

    def _on_layout(self, layout_context: gui.LayoutContext):
        """Handle window layout"""
        content = self.window.content_rect
        panel_height = 220  # Increased for topic panel

        self.panel.frame = gui.Rect(
            content.x,
            content.get_bottom() - panel_height,
            content.width,
            panel_height
        )

        self.scene.frame = gui.Rect(
            content.x,
            content.y,
            content.width,
            content.height - panel_height
        )

    def _on_close(self) -> bool:
        """Handle window close"""
        self.playing = False
        self.stop_event.set()
        return True

    def run(self):
        """Run the GUI application"""
        gui.Application.instance.run()
