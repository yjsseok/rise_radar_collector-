#!/usr/bin/env python3
"""PointCloud2 to Open3D converter"""

import logging
import numpy as np
from typing import List, Optional, Tuple

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    o3d = None

try:
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    PointCloud2 = None
    read_points = None


class PointCloudConverter:
    """Convert ROS2 PointCloud2 messages to Open3D PointCloud"""

    def __init__(self, colormap: str = 'viridis'):
        """
        Initialize converter

        Args:
            colormap: Colormap name for intensity visualization
        """
        if not OPEN3D_AVAILABLE:
            raise ImportError("Open3D not found. Install with: pip install open3d")

        if not ROS2_AVAILABLE:
            raise ImportError(
                "ROS2 sensor_msgs not found. Install with: "
                "pip install sensor-msgs sensor-msgs-py"
            )

        self.logger = logging.getLogger(__name__)
        self.colormap = colormap

    def convert(self, pc2_msg: PointCloud2) -> o3d.geometry.PointCloud:
        """
        Convert PointCloud2 message to Open3D PointCloud

        Args:
            pc2_msg: ROS2 PointCloud2 message

        Returns:
            Open3D PointCloud object
        """
        # Get available fields
        field_names = [field.name for field in pc2_msg.fields]

        # Read points
        points_list = list(read_points(
            pc2_msg,
            field_names=field_names,
            skip_nans=True
        ))

        # Create Open3D point cloud
        cloud = o3d.geometry.PointCloud()

        if not points_list:
            return cloud

        # Convert to numpy array
        points_array = np.array(points_list)

        # Extract XYZ coordinates
        xyz = None
        if 'x' in field_names and 'y' in field_names and 'z' in field_names:
            x_idx = field_names.index('x')
            y_idx = field_names.index('y')
            z_idx = field_names.index('z')
            xyz = points_array[:, [x_idx, y_idx, z_idx]]
        else:
            # Assume first 3 columns are XYZ
            xyz = points_array[:, :3]

        cloud.points = o3d.utility.Vector3dVector(xyz)

        # Apply colors based on intensity if available
        if 'intensity' in field_names:
            intensity_idx = field_names.index('intensity')
            intensity = points_array[:, intensity_idx]
            colors = self.apply_intensity_colormap(intensity)
            cloud.colors = o3d.utility.Vector3dVector(colors)
        else:
            # Default white color
            colors = np.ones((len(xyz), 3)) * 0.8
            cloud.colors = o3d.utility.Vector3dVector(colors)

        return cloud

    def apply_intensity_colormap(self, intensity: np.ndarray) -> np.ndarray:
        """
        Apply colormap to intensity values

        Args:
            intensity: Array of intensity values

        Returns:
            RGB color array (N x 3)
        """
        # Normalize intensity to [0, 1]
        if intensity.max() > intensity.min():
            normalized = (intensity - intensity.min()) / (intensity.max() - intensity.min())
        else:
            normalized = np.zeros_like(intensity)

        # Apply colormap
        if self.colormap == 'viridis':
            colors = self._viridis_colormap(normalized)
        elif self.colormap == 'jet':
            colors = self._jet_colormap(normalized)
        elif self.colormap == 'hot':
            colors = self._hot_colormap(normalized)
        elif self.colormap == 'cool':
            colors = self._cool_colormap(normalized)
        else:
            # Default: grayscale
            colors = np.column_stack([normalized, normalized, normalized])

        return colors

    def _viridis_colormap(self, values: np.ndarray) -> np.ndarray:
        """Viridis colormap approximation"""
        # Simple approximation of viridis colormap
        r = np.clip(0.267 + 0.975 * values, 0, 1)
        g = np.clip(0.005 + 0.902 * values, 0, 1)
        b = np.clip(0.329 + 0.528 * values, 0, 1)
        return np.column_stack([r, g, b])

    def _jet_colormap(self, values: np.ndarray) -> np.ndarray:
        """Jet colormap"""
        r = np.clip(1.5 - 4 * np.abs(values - 0.75), 0, 1)
        g = np.clip(1.5 - 4 * np.abs(values - 0.5), 0, 1)
        b = np.clip(1.5 - 4 * np.abs(values - 0.25), 0, 1)
        return np.column_stack([r, g, b])

    def _hot_colormap(self, values: np.ndarray) -> np.ndarray:
        """Hot colormap"""
        r = np.clip(3 * values, 0, 1)
        g = np.clip(3 * values - 1, 0, 1)
        b = np.clip(3 * values - 2, 0, 1)
        return np.column_stack([r, g, b])

    def _cool_colormap(self, values: np.ndarray) -> np.ndarray:
        """Cool colormap"""
        r = values
        g = 1 - values
        b = np.ones_like(values)
        return np.column_stack([r, g, b])

    def get_available_fields(self, pc2_msg: PointCloud2) -> List[str]:
        """
        Get list of available fields in PointCloud2 message

        Args:
            pc2_msg: ROS2 PointCloud2 message

        Returns:
            List of field names
        """
        return [field.name for field in pc2_msg.fields]

    def merge_point_clouds(self, clouds: List[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
        """
        Merge multiple point clouds into one

        Args:
            clouds: List of Open3D point clouds

        Returns:
            Merged point cloud
        """
        if not clouds:
            return o3d.geometry.PointCloud()

        if len(clouds) == 1:
            return clouds[0]

        merged = o3d.geometry.PointCloud()

        for cloud in clouds:
            merged += cloud

        return merged
