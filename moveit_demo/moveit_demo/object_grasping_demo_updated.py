#!/usr/bin/env python3
"""
Single-shot grasping demo integrating:
- SfM-based 3D reconstruction (sparse point cloud)
- Point cloud visualization via PointCloud2
- MoveIt2 motion planning using MoveItPy
- Joint-space and Cartesian-space motion primitives

Pipeline:
    1. Load pre-processed SfM point cloud + metadata
    2. Estimate object position (centroid)
    3. Publish point cloud once for RViz visualization
    4. Move robot to a pre-tuned joint-space "approach posture"
    5. Small pause for demonstration clarity
    6. Lift vertically above the object
    7. Move to a predefined placement location
    8. Return to the SRDF-defined "home" configuration

This script provides a clean, stable, and reproducible demo sequence suitable
for coursework submission.
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

import numpy as np
import json
import glob
import os
import struct
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint


class ObjectGraspingDemo(Node):
    def __init__(self):
        super().__init__("object_grasping_demo_updated")
        self.logger = get_logger("object_grasping_demo_updated")

        # Publisher for visualizing the SfM point cloud in RViz
        self.pc_pub = self.create_publisher(PointCloud2, "/sfm/pointcloud", 10)

        # Load the reconstructed sparse SfM point cloud + metadata
        self.load_processed_reconstruction()

        # Initialize MoveItPy interfaces
        self.ur5e = MoveItPy(node_name="object_grasping_demo_updated")
        self.arm = self.ur5e.get_planning_component("ur_manipulator")

    # ----------------------------------------------------------------------
    # Load SfM reconstruction output (.npy point cloud + metadata JSON)
    # ----------------------------------------------------------------------
    def load_processed_reconstruction(self):
        root = "/home/xinyue/ros2_ws/src/sfm/results"

        point_files = sorted(glob.glob(os.path.join(root, "processed_sfm_points_*.npy")))
        meta_files = sorted(glob.glob(os.path.join(root, "sfm_metadata_*.json")))

        if not point_files:
            raise RuntimeError("No SfM point cloud found (processed_sfm_points_*.npy)")
        if not meta_files:
            raise RuntimeError("No SfM metadata found (sfm_metadata_*.json)")

        latest_points = point_files[-1]
        latest_meta = meta_files[-1]

        self.logger.info(f"Loading point cloud: {latest_points}")
        self.logger.info(f"Loading metadata: {latest_meta}")

        self.points_3d = np.load(latest_points)

        with open(latest_meta, "r") as f:
            self.metadata = json.load(f)

        # Translate cloud so it appears in front of the robot base
        workspace_offset = np.array([0.4, 0.0, 0.2])
        self.points_3d_world = self.points_3d + workspace_offset

        # Estimate object position using point cloud centroid
        self.object_position = np.mean(self.points_3d_world, axis=0)

        # Print some basic cloud statistics for debugging / report
        self.logger.info(
            f"SfM points: {len(self.points_3d)}, centroid (base_link): {self.object_position}"
        )
        self.logger.info(
            "Cloud bounds | "
            f"x:[{self.points_3d_world[:,0].min():.3f}, {self.points_3d_world[:,0].max():.3f}], "
            f"y:[{self.points_3d_world[:,1].min():.3f}, {self.points_3d_world[:,1].max():.3f}], "
            f"z:[{self.points_3d_world[:,2].min():.3f}, {self.points_3d_world[:,2].max():.3f}]"
        )

    # ----------------------------------------------------------------------
    # Publish the sparse point cloud once (for RViz visualization)
    # ----------------------------------------------------------------------
    def publish_pointcloud_once(self):
        if self.points_3d_world is None or len(self.points_3d_world) == 0:
            self.logger.error("Point cloud empty: cannot publish /sfm/pointcloud")
            return

        points = self.points_3d_world  # (N x 3)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        buffer = [struct.pack("fff", float(p[0]), float(p[1]), float(p[2])) for p in points]
        data = b"".join(buffer)

        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = points.shape[0]
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12
        pc_msg.row_step = pc_msg.point_step * points.shape[0]
        pc_msg.is_dense = True
        pc_msg.data = data

        self.pc_pub.publish(pc_msg)
        self.logger.info(f"Published /sfm/pointcloud ({points.shape[0]} points)")

    # ----------------------------------------------------------------------
    # General-purpose MoveIt planning + execution helper
    # ----------------------------------------------------------------------
    def plan_and_execute(self, vel=0.2, accel=0.2) -> bool:
        """Plan and execute using OMPL RRTConnect with velocity/acceleration scaling."""
        robot = self.ur5e
        arm = self.arm

        # MoveItPy API differences across versions → keep a fallback constructor
        try:
            params = PlanRequestParameters(robot)
        except TypeError:
            params = PlanRequestParameters(robot, "ur_manipulator")

        params.planning_pipeline = "ompl"
        params.planner_id = "RRTConnectkConfigDefault"

        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = accel

        self.logger.info("Planning motion...")
        plan_result = arm.plan(params)

        if not plan_result or plan_result.trajectory is None:
            self.logger.error("Planning failed (empty plan_result)")
            return False

        traj = plan_result.trajectory
        self.logger.info("Executing trajectory...")
        robot.execute(traj, controllers=[])
        self.logger.info("Execution complete")
        return True

    # ----------------------------------------------------------------------
    # Stage 1: Move to a pre-defined joint-space "approach posture"
    # ----------------------------------------------------------------------
    def go_to_nice_joint_pose(self) -> bool:
        """
        Joint-space approach posture (degrees):
            shoulder_pan   -39°
            shoulder_lift  -98°
            elbow          144°
            wrist_1        -87°
            wrist_2         17°
            wrist_3        -93°
        """
        self.logger.info("Stage 1: Moving to RViz-tuned joint-space posture")

        robot_model = self.ur5e.get_robot_model()
        robot_state = RobotState(robot_model)

        joint_values = [
            -0.6806784083,   # -39°
            -1.7104226670,   # -98°
             2.5132741229,   # 144°
            -1.5184364492,   # -87°
             0.2967059728,   # 17°
            -1.6231562044,   # -93°
        ]

        group = "ur_manipulator"
        robot_state.set_joint_group_positions(group, joint_values)

        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=robot_model.get_joint_model_group(group),
        )

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        return self.plan_and_execute(vel=0.2, accel=0.2)

    # ----------------------------------------------------------------------
    # Stage 2: Lift vertically above the object
    # ----------------------------------------------------------------------
    def go_to_lift(self) -> bool:
        """Lift vertically to a safe height above the estimated object position."""
        self.logger.info("Stage 2: Vertical lift")

        self.arm.set_start_state_to_current_state()

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(self.object_position[0])
        pose.pose.position.y = float(self.object_position[1])
        pose.pose.position.z = float(self.object_position[2] + 0.25)
        pose.pose.orientation.w = 1.0  # simple downward orientation

        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
        return self.plan_and_execute(vel=0.2, accel=0.2)

    # ----------------------------------------------------------------------
    # Stage 3: Move sideways to a placement pose
    # ----------------------------------------------------------------------
    def go_to_place(self) -> bool:
        """Move to a predefined placement pose."""
        self.logger.info("Stage 3: Moving to placement pose")

        self.arm.set_start_state_to_current_state()

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.25
        pose.pose.position.z = 0.25
        pose.pose.orientation.w = 1.0

        # ✅ FIX: correct keyword is pose_stamped_msg
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
        return self.plan_and_execute(vel=0.2, accel=0.2)

    # ----------------------------------------------------------------------
    # Stage 4: Return to the standard SRDF "home" configuration
    # ----------------------------------------------------------------------
    def go_home(self) -> bool:
        """Return to the SRDF-defined 'home' joint configuration."""
        self.logger.info("Stage 4: Returning to home position")

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")

        return self.plan_and_execute(vel=0.2, accel=0.2)

    # ----------------------------------------------------------------------
    # Main pipeline
    # ----------------------------------------------------------------------
    def execute_complete_demo(self):
        self.logger.info("=== Starting full grasping demo ===")

        # Publish point cloud once for RViz visualization
        self.publish_pointcloud_once()

        # Stage 1: joint-space approach posture
        if not self.go_to_nice_joint_pose():
            self.logger.error("Failed: joint approach posture")
            return

        self.logger.info("Pausing for 2 seconds...")
        time.sleep(2.0)

        # Stage 2: vertical lift
        if not self.go_to_lift():
            self.logger.error("Failed: vertical lift")
            return

        # Stage 3: placement
        if not self.go_to_place():
            self.logger.error("Failed: placement motion")
            return

        # Stage 4: return home
        if not self.go_home():
            self.logger.error("Failed: return home")
            return

        self.logger.info("=== Demo completed successfully ===")

    def cleanup(self):
        try:
            del self.arm
            del self.ur5e
        except Exception:
            pass


def main():
    rclpy.init()
    node = ObjectGraspingDemo()
    try:
        node.execute_complete_demo()
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

