#!/usr/bin/env python3
"""
Simple Grasping Demo - 简化版本，避免配置问题
"""

import rclpy
import numpy as np
import time
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from rclpy.logging import get_logger

def main():
    rclpy.init()
    logger = get_logger("simple_grasping_demo")
    
    logger.info("🚀 开始简化版抓取演示")
    
    try:
        # 初始化 MoveItPy - 使用更简单的配置
        logger.info("初始化 MoveItPy...")
        robot = MoveItPy(node_name="simple_grasping_demo")
        
        # 获取规划组件
        arm = robot.get_planning_component("ur_manipulator")
        
        logger.info("MoveItPy 初始化成功!")
        
        # 显示重建信息
        logger.info("=" * 50)
        logger.info("📊 使用您的3D重建结果")
        logger.info("• 点云点数: 892")
        logger.info("• 重建时间: 20251116_223034")
        logger.info("• 物体位置: [0.385, 0.015, 0.242]")
        logger.info("=" * 50)
        time.sleep(2)
        
        # 1. 移动到观察位置
        logger.info("1️⃣ 移动到观察位置")
        arm.set_start_state_to_current_state()
        arm.set_goal_state(configuration_name="home")
        
        # 简单规划执行函数
        def simple_plan_execute():
            plan_result = arm.plan()
            if plan_result:
                logger.info("执行规划...")
                robot.execute(plan_result.trajectory, controllers=[])
                time.sleep(1.0)
                return True
            else:
                logger.error("规划失败!")
                return False
        
        if not simple_plan_execute():
            return
        
        time.sleep(2)
        
        # 2. 接近物体
        logger.info("2️⃣ 接近物体")
        arm.set_start_state_to_current_state()
        
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "base_link"
        approach_pose.pose.position.x = 0.385
        approach_pose.pose.position.y = 0.015
        approach_pose.pose.position.z = 0.342  # 10cm上方
        approach_pose.pose.orientation.x = 0.0
        approach_pose.pose.orientation.y = 0.707
        approach_pose.pose.orientation.z = 0.0
        approach_pose.pose.orientation.w = 0.707
        
        arm.set_goal_state(pose_stamped_msg=approach_pose, pose_link="tool0")
        
        if not simple_plan_execute():
            return
        
        time.sleep(1)
        
        # 3. 抓取物体
        logger.info("3️⃣ 执行抓取")
        arm.set_start_state_to_current_state()
        
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_link"
        grasp_pose.pose.position.x = 0.385
        grasp_pose.pose.position.y = 0.015
        grasp_pose.pose.position.z = 0.242
        grasp_pose.pose.orientation.x = 0.0
        grasp_pose.pose.orientation.y = 0.707
        grasp_pose.pose.orientation.z = 0.0
        grasp_pose.pose.orientation.w = 0.707
        
        arm.set_goal_state(pose_stamped_msg=grasp_pose, pose_link="tool0")
        
        if not simple_plan_execute():
            return
        
        logger.info("🤖 模拟夹爪关闭")
        time.sleep(2)
        
        # 4. 抬起物体
        logger.info("4️⃣ 抬起物体")
        arm.set_start_state_to_current_state()
        
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = "base_link"
        lift_pose.pose.position.x = 0.385
        lift_pose.pose.position.y = 0.015
        lift_pose.pose.position.z = 0.322  # 抬起8cm
        lift_pose.pose.orientation.x = 0.0
        lift_pose.pose.orientation.y = 0.707
        lift_pose.pose.orientation.z = 0.0
        lift_pose.pose.orientation.w = 0.707
        
        arm.set_goal_state(pose_stamped_msg=lift_pose, pose_link="tool0")
        
        if not simple_plan_execute():
            return
        
        time.sleep(1)
        
        # 5. 移动到放置位置
        logger.info("5️⃣ 移动到放置位置")
        arm.set_start_state_to_current_state()
        
        place_pose = PoseStamped()
        place_pose.header.frame_id = "base_link"
        place_pose.pose.position.x = 0.35
        place_pose.pose.position.y = -0.25
        place_pose.pose.position.z = 0.12
        place_pose.pose.orientation.x = 0.0
        place_pose.pose.orientation.y = 0.707
        place_pose.pose.orientation.z = 0.0
        place_pose.pose.orientation.w = 0.707
        
        arm.set_goal_state(pose_stamped_msg=place_pose, pose_link="tool0")
        
        if not simple_plan_execute():
            return
        
        logger.info("🤖 模拟夹爪打开")
        time.sleep(2)
        
        # 6. 返回Home
        logger.info("6️⃣ 返回Home位置")
        arm.set_start_state_to_current_state()
        arm.set_goal_state(configuration_name="home")
        
        if not simple_plan_execute():
            return
        
        logger.info("🎉 演示成功完成!")
        
    except Exception as e:
        logger.error(f"演示失败: {e}")
        import traceback
        logger.error(traceback.format_exc())
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
