#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import glob
import os
import numpy as np


class PLYPublisher(Node):
    def __init__(self):
        super().__init__("ply_publisher")

        # Automatically find the latest .ply file in the SfM results directory
        results_dir = "/home/xinyue/ros2_ws/src/sfm/results"
        ply_files = sorted(glob.glob(os.path.join(results_dir, "*.ply")))
        if not ply_files:
            raise RuntimeError(f"No .ply files found in {results_dir}")
        self.ply_path = ply_files[-1]

        self.get_logger().info(f"Loading PLY from: {self.ply_path}")

        # Load the original point cloud from the PLY file
        points = self.load_ply(self.ply_path)
        points = np.array(points, dtype=float)

        # Translate the point cloud to appear in front of the robot
        # (aligned with the grasping pipeline workspace offset)
        offset = np.array([0.4, 0.0, 0.2])
        points = points + offset

        self.points = [tuple(p) for p in points]

        # Publish to /sfm/pointcloud (same topic name as the SfM-based pipeline)
        self.publisher_ = self.create_publisher(PointCloud2, "/sfm/pointcloud", 10)

        # Publish the point cloud at 2 Hz (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(f"Loaded {len(self.points)} points (after offset)")

    def load_ply(self, path):
        """Minimal ASCII PLY loader: returns a list of (x, y, z) tuples."""
        points = []
        with open(path, "r") as f:
            header = True
            for line in f:
                line = line.strip()
                if header:
                    # Skip header until we see 'end_header'
                    if line.startswith("end_header"):
                        header = False
                    continue
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 3:
                    continue
                x, y, z = map(float, parts[:3])
                points.append((x, y, z))
        return points

    def timer_callback(self):
        """Periodic timer callback: publish the point cloud message."""
        msg = self.create_pointcloud2(self.points)
        self.publisher_.publish(msg)

    def create_pointcloud2(self, points):
        """Convert a list of (x, y, z) into a sensor_msgs/PointCloud2 message."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        # RViz Fixed Frame should also be set to 'base_link'
        header.frame_id = "base_link"

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        buffer = []
        for x, y, z in points:
            buffer.append(struct.pack("fff", x, y, z))
        msg.data = b"".join(buffer)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PLYPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
