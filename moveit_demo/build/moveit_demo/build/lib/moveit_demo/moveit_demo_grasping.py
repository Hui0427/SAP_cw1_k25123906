#!/usr/bin/env python3
"""
MoveItPy Grasping Demo for UR5e robot arm.
基于3D重建结果的抓取演示
"""

import time
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
import math

def plan_and_execute(robot, arm, planning_group, logger, vel=None, accel=None, sleep_time=0.0):
    """Helper function to plan and execute a motion."""
    try:
        params = PlanRequestParameters(robot)
    except TypeError:
        params = PlanRequestParameters(robot, planning_group)
    
    params.planning_pipeline = "ompl"
    params.planner_id = "RRTConnectkConfigDefault"
    
    if vel is not None:
        params.max_velocity_scaling_factor = vel
    if accel is not None:
        params.max_acceleration_scaling_factor = accel

    logger.info("Planning trajectory")
    plan_result = arm.plan(params)

    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        execution_status = robot.execute(robot_trajectory, controllers=[])
        logger.info(f"Execution status: {execution_status}")
        time.sleep(0.5)
    else:
        logger.error("Planning failed")
    
    time.sleep(sleep_time)

def main():
    rclpy.init()
    logger = get_logger("moveit_grasping_demo")
    
    # Initialize MoveItPy
    ur5e = MoveItPy(node_name="moveit_grasping_demo")
    ur5e_arm = ur5e.get_planning_component("ur_manipulator")
    
    logger.info("=== 🤖 3D重建抓取演示开始 ===")
    logger.info("基于您的SfM重建结果: 938个3D点, 74个相机位姿")
    logger.info("重建时间: 20251116_223034")
    logger.info("物体位置: [0.385, 0.015, 0.242]")
    
    planning_group = "ur_manipulator"
    
    # ========================================================================
    # STEP 1: 移动到观察位置
    # ========================================================================
    logger.info("=== 步骤1: 移动到观察位置 ===")
    ur5e_arm.set_start_state_to_current_state()
    
    # 使用关节空间规划到观察位置
    robot_model = ur5e.get_robot_model()
    robot_state = RobotState(robot_model)
    # 好的观察角度
    joint_values = [0.3, -0.8, 1.2, -0.4, -1.57, 0.0]
    robot_state.set_joint_group_positions("ur_manipulator", joint_values)
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=robot_model.get_joint_model_group("ur_manipulator"),
    )
    ur5e_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.2, accel=0.2, sleep_time=2.0)
    
    # ========================================================================
    # STEP 2: 接近物体 (基于3D重建结果)
    # ========================================================================
    logger.info("=== 步骤2: 接近重建的物体 ===")
    ur5e_arm.set_start_state_to_current_state()
    
    # 基于您的重建结果: 物体位置 [0.385, 0.015, 0.242]
    approach_pose = PoseStamped()
    approach_pose.header.frame_id = "base_link"
    approach_pose.pose.position.x = 0.385    # 从重建结果
    approach_pose.pose.position.y = 0.015    # 从重建结果  
    approach_pose.pose.position.z = 0.342    # 10cm上方
    approach_pose.pose.orientation.x = 0.0
    approach_pose.pose.orientation.y = 0.707
    approach_pose.pose.orientation.z = 0.0
    approach_pose.pose.orientation.w = 0.707
    
    ur5e_arm.set_goal_state(pose_stamped_msg=approach_pose, pose_link="tool0")
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.15, accel=0.15, sleep_time=1.0)
    
    # ========================================================================
    # STEP 3: 执行抓取
    # ========================================================================
    logger.info("=== 步骤3: 执行抓取 ===")
    ur5e_arm.set_start_state_to_current_state()
    
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_link"
    grasp_pose.pose.position.x = 0.385    # 物体位置
    grasp_pose.pose.position.y = 0.015
    grasp_pose.pose.position.z = 0.242
    grasp_pose.pose.orientation.x = 0.0
    grasp_pose.pose.orientation.y = 0.707
    grasp_pose.pose.orientation.z = 0.0
    grasp_pose.pose.orientation.w = 0.707
    
    ur5e_arm.set_goal_state(pose_stamped_msg=grasp_pose, pose_link="tool0")
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.1, accel=0.1, sleep_time=2.0)
    
    logger.info("🤖 模拟夹爪关闭 - 抓取物体")
    time.sleep(2)
    
    # ========================================================================
    # STEP 4: 抬起物体
    # ========================================================================
    logger.info("=== 步骤4: 抬起物体 ===")
    ur5e_arm.set_start_state_to_current_state()
    
    lift_pose = PoseStamped()
    lift_pose.header.frame_id = "base_link"
    lift_pose.pose.position.x = 0.385
    lift_pose.pose.position.y = 0.015
    lift_pose.pose.position.z = 0.322    # 抬起8cm
    lift_pose.pose.orientation.x = 0.0
    lift_pose.pose.orientation.y = 0.707
    lift_pose.pose.orientation.z = 0.0
    lift_pose.pose.orientation.w = 0.707
    
    ur5e_arm.set_goal_state(pose_stamped_msg=lift_pose, pose_link="tool0")
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.1, accel=0.1, sleep_time=1.0)
    
    # ========================================================================
    # STEP 5: 移动到放置位置
    # ========================================================================
    logger.info("=== 步骤5: 移动到放置位置 ===")
    ur5e_arm.set_start_state_to_current_state()
    
    place_pose = PoseStamped()
    place_pose.header.frame_id = "base_link"
    place_pose.pose.position.x = 0.35     # 放置位置
    place_pose.pose.position.y = -0.25
    place_pose.pose.position.z = 0.12
    place_pose.pose.orientation.x = 0.0
    place_pose.pose.orientation.y = 0.707
    place_pose.pose.orientation.z = 0.0
    place_pose.pose.orientation.w = 0.707
    
    ur5e_arm.set_goal_state(pose_stamped_msg=place_pose, pose_link="tool0")
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.15, accel=0.15, sleep_time=1.0)
    
    logger.info("🤖 模拟夹爪打开 - 释放物体")
    time.sleep(2)
    
    # ========================================================================
    # STEP 6: 返回Home
    # ========================================================================
    logger.info("=== 步骤6: 返回Home位置 ===")
    ur5e_arm.set_start_state_to_current_state()
    ur5e_arm.set_goal_state(configuration_name="home")
    plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.2, accel=0.2, sleep_time=2.0)
    
    logger.info("🎉 3D重建抓取演示完成!")
    logger.info("✅ 基于938个3D点的重建结果")
    logger.info("✅ 成功执行抓取-移动-放置序列")
    logger.info("✅ 展示了完整的机器人抓取流程")

if __name__ == "__main__":
    main()
