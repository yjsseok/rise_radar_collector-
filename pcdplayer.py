#!/usr/bin/env python3
"""Simple GUI player for sequential PCD files with play/pause and scrub bar."""

from __future__ import annotations

import argparse
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import open3d as o3d  # type: ignore
    from open3d.visualization import gui, rendering  # type: ignore
except ImportError as exc:  # pragma: no cover - guarded execution
    o3d = None  # type: ignore
    gui = None  # type: ignore
    rendering = None  # type: ignore
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class PCDPlayerApp:
    """Encapsulates the Open3D GUI for point cloud playback."""

    def __init__(self, files: List[Path], fps: float, loop: bool) -> None:
        self._files = files
        self._fps = max(fps, 0.1)
        self._loop = loop
        self._index = 0
        self._playing = False
        self._camera_prepared = False
        self._slider_guard = False
        self._cloud_cache: Dict[int, Any] = {}
        self._geometry_name = "pcd"

        app = gui.Application.instance
        self._window = app.create_window("PCD Player", 1280, 720)
        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self._window.renderer)
        self._scene.scene.set_background([0, 0, 0, 1])
        self._scene.scene.show_axes(True)
        self._scene.set_on_key(self._on_key_event)

        self._material = rendering.MaterialRecord()
        self._material.shader = "defaultUnlit"
        self._material.point_size = 2.0

        self._status_label = gui.Label("")

        self._play_button = gui.Button("Play")
        self._play_button.set_on_clicked(self._on_play_clicked)

        self._pause_button = gui.Button("Pause")
        self._pause_button.set_on_clicked(self._on_pause_clicked)

        self._rewind_button = gui.Button("Rewind")
        self._rewind_button.set_on_clicked(self._on_rewind_clicked)

        self._slider = gui.Slider(gui.Slider.INT)
        self._slider.set_limits(0, len(self._files) - 1)
        self._slider.set_on_value_changed(self._on_slider_value_changed)

        controls = gui.Horiz(8, gui.Margins(8, 8, 8, 8))
        controls.add_child(self._play_button)
        controls.add_child(self._pause_button)
        controls.add_child(self._rewind_button)

        panel = gui.Vert(6, gui.Margins(8, 8, 8, 8))
        panel.add_child(self._status_label)
        panel.add_child(self._slider)
        panel.add_child(controls)

        self._window.add_child(self._scene)
        self._window.add_child(panel)
        self._panel = panel

        self._window.set_on_layout(self._on_layout)

        self._stop_event = threading.Event()
        self._window.set_on_close(self._on_close)
        self._timer_thread = threading.Thread(target=self._timer_loop, daemon=True)
        self._timer_thread.start()

        self._show_frame(update_slider=True)

    def _load_cloud(self, index: int) -> Any:
        if index in self._cloud_cache:
            return self._cloud_cache[index]
        cloud = o3d.io.read_point_cloud(str(self._files[index]))
        self._cloud_cache[index] = cloud
        return cloud

    def _show_frame(self, update_slider: bool) -> None:
        cloud = self._load_cloud(self._index)
        description = f"Frame {self._index + 1}/{len(self._files)}"
        try:
            self._scene.scene.remove_geometry(self._geometry_name)
        except Exception:
            pass
        if cloud.is_empty():
            description += " | empty cloud"
        else:
            self._scene.scene.add_geometry(self._geometry_name, cloud, self._material)
            if not self._camera_prepared:
                bounds = cloud.get_axis_aligned_bounding_box()
                if bounds.get_max_extent() == 0:
                    bounds = o3d.geometry.AxisAlignedBoundingBox(
                        min_bound=(-1, -1, -1),
                        max_bound=(1, 1, 1),
                    )
                self._scene.setup_camera(60.0, bounds, bounds.get_center())
                self._camera_prepared = True
        timestamp = _timestamp_from_name(self._files[self._index])
        if timestamp is not None:
            description += f" | Timestamp(ns): {timestamp}"
        self._status_label.text = description

        if update_slider and not self._slider_guard:
            self._slider_guard = True
            self._slider.int_value = self._index
            self._slider_guard = False

    def _on_play_clicked(self) -> None:
        self._playing = True

    def _on_pause_clicked(self) -> None:
        self._playing = False

    def _on_rewind_clicked(self) -> None:
        self._playing = False
        self._index = 0
        self._show_frame(update_slider=True)

    def _on_slider_value_changed(self, value: float) -> None:
        if self._slider_guard:
            return
        self._playing = False
        self._index = max(0, min(int(round(value)), len(self._files) - 1))
        self._show_frame(update_slider=False)

    def _timer_loop(self) -> None:
        interval = 1.0 / self._fps
        while not self._stop_event.wait(interval):
            if self._playing:
                gui.Application.instance.post_to_main_thread(
                    self._window, self._advance_frame
                )

    def _advance_frame(self) -> None:
        if not self._playing:
            return
        self._index += 1
        if self._index >= len(self._files):
            if self._loop:
                self._index = 0
            else:
                self._index = len(self._files) - 1
                self._playing = False
                self._show_frame(update_slider=True)
                return
        self._show_frame(update_slider=True)

    def _on_close(self) -> bool:
        self._playing = False
        self._stop_event.set()
        return True

    def _on_key_event(self, event: gui.KeyEvent) -> gui.EventCallbackResult:
        if event.key == gui.KeyName.SPACE and event.type == gui.KeyEvent.DOWN:
            self._playing = not self._playing
            return gui.EventCallbackResult.HANDLED
        if event.key == gui.KeyName.LEFT and event.type == gui.KeyEvent.DOWN:
            self._playing = False
            self._index = max(0, self._index - 1)
            self._show_frame(update_slider=True)
            return gui.EventCallbackResult.HANDLED
        if event.key == gui.KeyName.RIGHT and event.type == gui.KeyEvent.DOWN:
            self._playing = False
            self._index = min(len(self._files) - 1, self._index + 1)
            self._show_frame(update_slider=True)
            return gui.EventCallbackResult.HANDLED
        return gui.EventCallbackResult.IGNORED

    def _on_layout(self, layout_context: gui.LayoutContext) -> None:
        content = self._window.content_rect
        panel_height = 110
        self._panel.frame = gui.Rect(
            content.x,
            content.get_bottom() - panel_height,
            content.width,
            panel_height,
        )
        self._scene.frame = gui.Rect(
            content.x,
            content.y,
            content.width,
            content.height - panel_height,
        )


def _ensure_open3d() -> None:
    if _IMPORT_ERROR is None:
        return
    raise RuntimeError(
        "Open3D? ???? ?????. `pip install open3d` ?? ? ?? ?????. "
        f"?? ??: {_IMPORT_ERROR}"
    )


def _collect_pcd_files(directory: Path, recursive: bool) -> List[Path]:
    iterator = directory.rglob("*.pcd") if recursive else directory.glob("*.pcd")
    return sorted(path for path in iterator if path.is_file())


def _timestamp_from_name(path: Path) -> Optional[int]:
    name = path.stem
    parts = name.rsplit("_", 1)
    if len(parts) != 2:
        return None
    try:
        return int(parts[1])
    except ValueError:
        return None


def _parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "PCD ?? ???? Open3D GUI? ???? ??/???? ? ??? ??? ?????."
        )
    )
    parser.add_argument("pcd_path", type=Path, help="??? PCD ??? ?? ?? ????")
    parser.add_argument("--fps", type=float, default=10.0, help="?? ??? ??? (?? 10)")
    parser.add_argument("--loop", action="store_true", help="??? ??? ?? ???? ??")
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="?? ?????? ?? PCD ??? ??",
    )
    return parser.parse_args()


def main() -> int:
    _ensure_open3d()
    args = _parse_arguments()
    pcd_root = args.pcd_path.expanduser().resolve()
    if not pcd_root.exists():
        raise FileNotFoundError(f"??? ?? ? ????: {pcd_root}")
    files = _collect_pcd_files(pcd_root, args.recursive)
    if not files:
        raise FileNotFoundError("PCD ??? ?? ?? ?? ?????.")

    app = gui.Application.instance
    app.initialize()
    _ = PCDPlayerApp(files, fps=args.fps, loop=args.loop)
    app.run()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
