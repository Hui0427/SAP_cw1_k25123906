#!/usr/bin/env python3
"""
Updated Object Grasping Demo using Processed SfM Reconstruction
使用处理后的点云数据进行抓取演示
"""

import rclpy
import numpy as np
import time
import os
import json
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import PoseStamped
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint

class ObjectGraspingDemoUpdated(Node):
    def __init__(self):
        super().__init__('object_grasping_demo_updated')
        
        # Initialize MoveItPy
        self.robot = MoveItPy(node_name="object_grasping_demo_updated")
        self.arm = self.robot.get_planning_component("ur_manipulator")
        self.logger = get_logger("object_grasping_demo_updated")
        
        # Load processed reconstruction data
        self.load_processed_reconstruction()
        
        self.logger.info("Updated Object Grasping Demo initialized")
        
        # Start demonstration
        self.execute_complete_demo()
    
    def find_latest_processed_data(self):
        """找到最新的处理后的重建数据"""
        results_dir = "/home/xinyue/ros2_ws/src/sfm/results"
        
        # 查找处理后的点云文件
        processed_files = [f for f in os.listdir(results_dir) 
                          if f.startswith('processed_sfm_points_') and f.endswith('.npy')]
        
        if not processed_files:
            raise FileNotFoundError("No processed reconstruction files found")
        
        # 找到最新的文件
        latest_file = sorted(processed_files)[-1]
        latest_path = os.path.join(results_dir, latest_file)
        
        # 对应的元数据文件
        timestamp = latest_file.replace('processed_sfm_points_', '').replace('.npy', '')
        metadata_file = f"sfm_metadata_{timestamp}.json"
        metadata_path = os.path.join(results_dir, metadata_file)
        
        return latest_path, metadata_path
    
    def load_processed_reconstruction(self):
        """加载处理后的重建数据"""
        try:
            points_path, metadata_path = self.find_latest_processed_data()
            
            self.logger.info(f"Loading processed reconstruction: {os.path.basename(points_path)}")
            
            # 加载处理后的点云
            self.points_3d = np.load(points_path)
            
            # 加载元数据
            with open(metadata_path, 'r') as f:
                self.metadata = json.load(f)
            
            # 计算物体位置（点云中心）
            self.object_position = np.mean(self.points_3d, axis=0)
            
            # 调整到合适的机器人工作空间位置
            # UR5e的工作空间大致是：x[0.2, 0.6], y[-0.4, 0.4], z[0.1, 0.5]
            workspace_center = np.array([0.4, 0.0, 0.2])  # 机器人前方的中心位置
            self.object_position += workspace_center
            
            self.logger.info(f"成功加载处理后的重建数据:")
            self.logger.info(f"  • 点云点数: {len(self.points_3d)}")
            self.logger.info(f"  • 物体位置: {self.object_position}")
            self.logger.info(f"  • 原始重建: {self.metadata['num_points']} points, {self.metadata['num_poses']} poses")
            
        except Exception as e:
            self.logger.error(f"加载重建数据失败: {e}")
            self.logger.info("使用默认物体位置进行演示")
            # 使用默认位置
            self.object_position = np.array([0.4, 0.1, 0.15])
            self.points_3d = np.array([[0.4, 0.1, 0.15]])
            self.metadata = {"num_points": 1, "num_poses": 1, "timestamp": "default"}
    
    def execute_complete_demo(self):
        """执行完整的演示序列"""
        self.logger.info("🚀 开始完整的物体抓取演示")
        
        try:
            # 演示序列
            self.show_reconstruction_info()
            self.move_to_view_position()
            self.approach_object()
            self.grasp_object()
            self.lift_object()
            self.move_to_placement()
            self.release_object()
            self.return_to_home()
            
            self.logger.info("🎉 演示成功完成!")
            
        except Exception as e:
            self.logger.error(f"演示失败: {e}")
    
    def show_reconstruction_info(self):
        """显示重建信息"""
        self.logger.info("=" * 50)
        self.logger.info("📊 3D重建结果")
        self.logger.info("=" * 50)
        self.logger.info(f"• 重建时间: {self.metadata['timestamp']}")
        self.logger.info(f"• 3D点云点数: {self.metadata['num_points']} → {len(self.points_3d)} (处理后)")
        self.logger.info(f"• 相机位姿数量: {self.metadata['num_poses']}")
        self.logger.info(f"• 输入图像数量: {self.metadata['num_images']}")
        self.logger.info(f"• 物体抓取位置: [{self.object_position[0]:.3f}, {self.object_position[1]:.3f}, {self.object_position[2]:.3f}]")
        self.logger.info("=" * 50)
        time.sleep(3)
    
    def move_to_view_position(self):
        """移动到观察位置"""
        self.logger.info("1️⃣ 移动到观察位置")
        
        self.arm.set_start_state_to_current_state()
        
        # 使用关节空间规划到好的观察角度
        robot_model = self.robot.get_robot_model()
        robot_state = RobotState(robot_model)
        view_joints = [0.2, -1.0, 1.3, -1.3, -1.57, 0.0]  # 好的观察角度
        robot_state.set_joint_group_positions("ur_manipulator", view_joints)
        
        self.arm.set_goal_state(motion_plan_constraints=[
            self.construct_joint_constraint(robot_state)
        ])
        
        self.plan_and_execute(vel=0.2, accel=0.2)
        time.sleep(2)
    
    def approach_object(self):
        """接近物体"""
        self.logger.info("2️⃣ 接近物体")
        
        self.arm.set_start_state_to_current_state()
        
        approach_pose = self.create_pose(
            [self.object_position[0], self.object_position[1], self.object_position[2] + 0.1],  # 10cm上方
            [0.0, 0.707, 0.0, 0.707]  # 末端向下
        )
        
        self.arm.set_goal_state(pose_stamped_msg=approach_pose, pose_link="tool0")
        self.plan_and_execute(vel=0.15, accel=0.15)
        time.sleep(1)
    
    def grasp_object(self):
        """抓取物体"""
        self.logger.info("3️⃣ 执行抓取")
        
        self.arm.set_start_state_to_current_state()
        
        grasp_pose = self.create_pose(
            [self.object_position[0], self.object_position[1], self.object_position[2]],
            [0.0, 0.707, 0.0, 0.707]
        )
        
        self.arm.set_goal_state(pose_stamped_msg=grasp_pose, pose_link="tool0")
        self.plan_and_execute(vel=0.1, accel=0.1)
        
        # 模拟夹爪关闭
        self.logger.info("🤖 模拟夹爪关闭")
        time.sleep(2)
    
    def lift_object(self):
        """抬起物体"""
        self.logger.info("4️⃣ 抬起物体")
        
        self.arm.set_start_state_to_current_state()
        
        lift_pose = self.create_pose(
            [self.object_position[0], self.object_position[1], self.object_position[2] + 0.08],  # 抬起8cm
            [0.0, 0.707, 0.0, 0.707]
        )
        
        self.arm.set_goal_state(pose_stamped_msg=lift_pose, pose_link="tool0")
        self.plan_and_execute(vel=0.1, accel=0.1)
        time.sleep(1)
    
    def move_to_placement(self):
        """移动到放置位置"""
        self.logger.info("5️⃣ 移动到放置位置")
        
        self.arm.set_start_state_to_current_state()
        
        place_pose = self.create_pose(
            [0.35, -0.25, 0.12],  # 放置位置
            [0.0, 0.707, 0.0, 0.707]
        )
        
        self.arm.set_goal_state(pose_stamped_msg=place_pose, pose_link="tool0")
        self.plan_and_execute(vel=0.15, accel=0.15)
        time.sleep(1)
    
    def release_object(self):
        """释放物体"""
        self.logger.info("6️⃣ 释放物体")
        
        # 模拟夹爪打开
        self.logger.info("🤖 模拟夹爪打开")
        time.sleep(2)
    
    def return_to_home(self):
        """返回Home位置"""
        self.logger.info("7️⃣ 返回Home位置")
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")
        self.plan_and_execute(vel=0.2, accel=0.2)
        
        self.logger.info("✅ 所有动作完成!")
    
    def create_pose(self, position, orientation):
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        return pose
    
    def construct_joint_constraint(self, robot_state):
        """创建关节约束"""
        robot_model = self.robot.get_robot_model()
        return construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=robot_model.get_joint_model_group("ur_manipulator"),
        )
    
    def plan_and_execute(self, vel=None, accel=None):
        """规划和执行运动"""
        try:
            params = PlanRequestParameters(self.robot, "ur_manipulator")
        except TypeError:
            params = PlanRequestParameters(self.robot)
        
        params.planning_pipeline = "ompl"
        params.planner_id = "RRTConnectkConfigDefault"
        
        if vel is not None:
            params.max_velocity_scaling_factor = vel
        if accel is not None:
            params.max_acceleration_scaling_factor = accel

        self.logger.info("🔄 规划轨迹...")
        plan_result = self.arm.plan(params)

        if plan_result:
            self.logger.info("⚡ 执行规划...")
            robot_trajectory = plan_result.trajectory
            execution_status = self.robot.execute(robot_trajectory, controllers=[])
            self.logger.info(f"执行状态: {execution_status}")
            time.sleep(0.5)
            return True
        else:
            self.logger.error("❌ 规划失败!")
            return False

def main():
    rclpy.init()
    
    demo = ObjectGraspingDemoUpdated()
    
    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.logger.info("演示被用户中断")
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
