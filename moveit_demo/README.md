# MoveIt Motion Planning Tutorial

**7CCEMSAP - Sensing and Perception Module**

This tutorial provides a hands-on introduction to robot motion planning using MoveIt2 and MoveItPy in ROS2, with practical examples for controlling a UR5e robot arm.

> **📋 Complete Solutions Provided**: This repository contains complete working solutions for MoveIt motion planning. Use these as reference implementations while learning robotic manipulation concepts.

## Learning Objectives

This tutorial explains how to use MoveIt2 and MoveItPy to control a robot arm in ROS2. You will learn to:

- **Launch MoveIt** with the UR5e robot configuration
- **Plan robot motions** using joint space and Cartesian space planning
- **Execute trajectories** to move the robot arm
- **Use MoveItPy** Python API for programmatic robot control

**Additional Learning Goals:**
- Understand the difference between **joint space** and **Cartesian space** planning
- Use **predefined robot states** from SRDF files
- Plan to **specific end-effector poses** using inverse kinematics
- Chain multiple movements together
- Work with **real robot hardware** or simulation

## Prerequisites

- A working Linux environment with ROS2 Jazzy installed
- Python 3.6+ installed
- MoveIt2 packages installed
- UR robot description packages (`ur_description`, `ur_moveit_config`)
- Basic familiarity with ROS2 and Python

## What is MoveIt?

MoveIt is a motion planning framework for robotic manipulation. It provides:

- **Motion planning**: Generate collision-free trajectories
- **Inverse kinematics**: Compute joint angles from end-effector poses
- **Collision detection**: Avoid obstacles and self-collisions
- **Trajectory execution**: Execute planned motions safely
- **Python API (MoveItPy)**: Programmatic control of robot motion

MoveIt is widely used for:
- Pick-and-place operations
- Manipulation tasks
- Path planning in cluttered environments
- Research and development in robotics

## Step-by-Step Tutorial

### Step 1: Prerequisites and System Setup

**Install Required Packages:**
```bash
sudo apt-get install \
  ros-jazzy-desktop \
  ros-jazzy-moveit \
  ros-jazzy-moveit-py \
  ros-jazzy-ur-description \
  ros-jazzy-ur-moveit-config \
  ros-jazzy-ur-robot-driver
```

**Source ROS2:**
```bash
source /opt/ros/jazzy/setup.bash
```

### Step 2: Set Up Your ROS2 Workspace

**Create and build workspace:**
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build --symlink-install
source install/local_setup.bash
```

**Clone this repository:**
```bash
cd ~/ros2_ws/src
git clone https://github.kcl.ac.uk/7CCEMSAP/moveit_demo.git moveit_demo
```

**Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select moveit_demo
source install/setup.bash
```

**Verify your workspace structure:**
After setup, your workspace should look like this:
```
~/ros2_ws/
├── build/          # Build artifacts (generated)
├── install/        # Installed packages (generated)
├── log/           # Build logs (generated)
└── src/           # Source packages
    └── moveit_demo/  # This tutorial package
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── moveit_demo/  # Python package directory
        │   ├── __init__.py
        │   └── moveit_demo.py
        ├── config/  # MoveIt configuration files
        └── launch/  # Launch files
            └── moveit_demo.launch.py
```

**About the Configuration Files:**

The `config/` directory contains MoveIt configuration files that define how the robot planning system works:

- **`moveit_py.yaml`**: Main MoveItPy configuration file that sets up planning pipelines, planning scene monitoring, and default planning parameters. This is loaded by the launch file to configure MoveItPy.
- **`ur5e.urdf.xacro`** and **`ur5e.srdf.xacro`**: Robot description files that define the robot's physical structure (URDF) and semantic information like planning groups and predefined states (SRDF).
- **Planner configuration files** (`ompl_planning.yaml`, `chomp_planning.yaml`, `pilz_industrial_motion_planner_planning.yaml`): Define settings for different motion planning algorithms (OMPL, CHOMP, and Pilz Industrial Motion Planner).

**File Locations:**

- **In your workspace**: `~/ros2_ws/src/moveit_demo/config/` (source files) or `~/ros2_ws/install/moveit_demo/share/moveit_demo/config/` (installed files)
- **System-wide MoveIt configs**: `/opt/ros/jazzy/share/ur_moveit_config/config/` (planner configurations from `ur_moveit_config` package)
- **System-wide URDF files**: `/opt/ros/jazzy/share/ur_description/urdf/` (robot description files from `ur_description` package)
- **System-wide SRDF files**: `/opt/ros/jazzy/share/ur_moveit_config/srdf/` (semantic robot description files from `ur_moveit_config` package)

These files are automatically loaded by the launch file (`moveit_demo.launch.py`) when you run the demo. You typically don't need to modify them unless you want to customize planning behavior or add new robot configurations.

### Step 3: Launch the Robot Driver

**Option 1: Using Mock Hardware (Recommended for VMs)**
```bash
# In terminal 1: Launch robot driver with mock hardware
source /opt/ros/jazzy/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_mock_hardware:=true robot_ip:=192.168.56.101
```

This launches:
- Robot description publisher (`/robot_description`)
- Joint state publisher (`/joint_states`)
- Controller manager for trajectory execution
- Mock hardware (no actual robot needed)

<!-- **Option 2: Using Real Hardware**
```bash
# In terminal 1: Launch robot driver with real hardware
source /opt/ros/jazzy/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=<YOUR_ROBOT_IP>
``` -->

**Verify the robot is running:**
```bash
# In a new terminal, check topics
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E 'joint|robot_description'
```

You should see:
- `/joint_states` - Current joint positions
- `/robot_description` - Robot URDF

### Step 4: Launch MoveIt with RViz and Practice Interactive Control

**Launch MoveIt with RViz for interactive robot control:**
```bash
# In terminal 2: Launch MoveIt with RViz
source /opt/ros/jazzy/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

This opens RViz with the MoveIt Motion Planning plugin, which provides an interactive interface for controlling the robot.

**Practice using the MoveIt Motion Planner in RViz:**

1. **Locate the Motion Planning panel** (usually on the left side of RViz)
   - You should see a panel titled "Motion Planning" or "MoveIt Motion Planning"

2. **Set the planning group**
   - In the Motion Planning panel, ensure **Planning Group** is set to `ur_manipulator`

3. **Move the robot interactively:**
   - You'll see an **interactive marker** (colored arrows and rings) attached to the robot's end-effector
   - **Drag the arrows** to move the end-effector in X, Y, or Z directions
   - **Drag the rings** to rotate the end-effector around different axes
   - The robot model will show a **ghost/orange preview** of where the robot will move

4. **Plan a trajectory:**
   - After positioning the interactive marker where you want the robot to go, click the **"Plan"** button
   - MoveIt will compute a collision-free trajectory
   - You'll see an **animated preview** of the planned motion (usually in green/orange)

5. **Execute the planned motion:**
   - Once planning succeeds, click the **"Execute"** button
   - The robot will move to the target pose
   - Watch the robot move in real-time

6. **Try different control methods:**
   - **Cartesian control**: Use the interactive marker to position the end-effector
   - **Joint control**: Use the "Joints" tab in the Motion Planning panel to set individual joint angles
   - **Predefined states**: Use the "States" tab to select predefined configurations like "home" or "up"

**Why this is important:**
This interactive practice helps you understand:
- How MoveIt plans collision-free trajectories
- The difference between planning and execution
- How to control robots using both Cartesian (end-effector) and joint space methods
- Visual feedback of planned motions before execution

### Step 5: Control the Robot Using a Python Script

Now that you've practiced controlling the robot interactively in RViz, let's learn how to control it programmatically using Python scripts with MoveItPy.

**What you need running:**
- **Terminal 1**: Robot driver (from Step 3) - should still be running
- **Terminal 2**: (Close this) - MoveIt with RViz from Step 4 is not needed for the Python script
- **Terminal 3**: (New) - We'll run the Python script here

**Important**: The Python script creates its own MoveIt instance, so you should close Terminal 2 (MoveIt/RViz) before running the script. The script will work independently and you can optionally open RViz separately to visualize the motion if desired.

**Make sure the robot driver is running** (from Step 3):
```bash
# Terminal 1: Robot driver should still be running
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_mock_hardware:=true robot_ip:=192.168.56.101
```

**Run the MoveItPy demo script:**
```bash
# Run the demo (new terminal)
cd ~/ros2_ws
source install/setup.bash
ros2 launch moveit_demo moveit_demo.launch.py
```

**What the script does:**
The demo script (`moveit_demo.py`) performs three examples in sequence:

1. **Example 1: Joint Space Planning** - Moves robot to a custom joint configuration (arm forward and extended)
2. **Example 2: Cartesian Space Planning** - Moves end-effector to a specific pose (position and orientation)
3. **Example 3: Joint Space Planning** - Returns robot to "home" state

**Expected output:**
You should see:
- Planning and execution messages in the terminal
- Robot arm moving in RViz (if you have it open from Step 4)
- "Demo completed successfully!" message

**Watch the robot move:**
- If RViz is open from Step 4, you'll see the robot execute all three movements
- The terminal will show detailed planning and execution status for each example

### Step 6: Understanding the Code

> **📁 Code Location**: The complete working code has been provided for you! You can find the main MoveItPy implementation in:
> ```bash
> ~/ros2_ws/src/moveit_demo/moveit_demo/moveit_demo.py
> ```
> This file contains the complete solution that you can examine, run, and modify.

**The MoveItPy demo does the following:**

1. **Initializes MoveItPy** with robot configuration
2. **Plans trajectories** using joint space or Cartesian space planning
3. **Executes trajectories** to move the robot arm
4. **Chains movements** by using current state as start state

**Key concepts:**
- **Joint Space Planning**: Direct control of joint angles
- **Cartesian Space Planning**: Control of end-effector position/orientation
- **Predefined States**: Named robot configurations from SRDF files
- **State Chaining**: Using current state as start for next movement

### Step 7: Code Breakdown

> **💡 Tip**: Open the code file in your editor to follow along:
> ```bash
> code ~/ros2_ws/src/moveit_demo/moveit_demo/moveit_demo.py
> # or
> nano ~/ros2_ws/src/moveit_demo/moveit_demo/moveit_demo.py
> ```

Let's examine the Python code in detail:

#### **Imports and Setup (Lines 1-14)**
```python
import time
import rclpy
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
```
- **ROS2 imports**: `rclpy` for ROS2 functionality, `get_logger` for logging
- **MoveItPy**: `MoveItPy` for robot motion planning, `PlanRequestParameters` for planning configuration
- **Messages**: `PoseStamped` for Cartesian pose goals
- **MoveIt core**: `RobotState` and `construct_joint_constraint` for joint space planning

#### **Helper Function: plan_and_execute (Lines 17-52)**
```python
def plan_and_execute(robot, arm, planning_group, logger, vel=None, accel=None, sleep_time=0.0):
    """Helper function to plan and execute a motion."""
    params = PlanRequestParameters(robot, planning_group)
    params.planning_pipeline = "ompl"
    params.planner_id = "RRTConnectkConfigDefault"
    
    if vel is not None:
        params.max_velocity_scaling_factor = vel
    if accel is not None:
        params.max_acceleration_scaling_factor = accel

    plan_result = arm.plan(params)
    
    if plan_result:
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
```
- **Planning parameters**: Configures planner (OMPL RRTConnect), velocity, and acceleration limits
- **Planning**: Calls `arm.plan()` to generate trajectory
- **Execution**: Calls `robot.execute()` to move the robot
- **Safety**: Velocity/acceleration scaling (0.3 = 30% of maximum) for safe operation

#### **MoveItPy Initialization (Lines 55-60)**
```python
ur5e = MoveItPy(node_name="moveit_demo")
ur5e_arm = ur5e.get_planning_component("ur_manipulator")
```
- **MoveItPy instance**: Creates the main MoveIt interface
- **Planning component**: Gets the planning interface for the "ur_manipulator" group
- **Parameters**: Loaded automatically from launch file configuration

#### **Example 1: Joint Space Planning (Lines 66-94)**
```python
robot_model = ur5e.get_robot_model()
robot_state = RobotState(robot_model)
joint_values = [0.0, -0.6, 1.0, -0.4, 0.0, 0.0]
robot_state.set_joint_group_positions("ur_manipulator", joint_values)
joint_constraint = construct_joint_constraint(
    robot_state=robot_state,
    joint_model_group=robot_model.get_joint_model_group("ur_manipulator"),
)
ur5e_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.3, accel=0.3, sleep_time=2.0)
```
- **Start state**: Sets start to current robot position
- **Goal state**: Custom joint angles [0.0, -0.6, 1.0, -0.4, 0.0, 0.0] that position arm forward and extended
- **Planning**: Computes trajectory in joint space
- **Type**: **JOINT CONTROL** - Direct joint angle specification using RobotState

#### **Example 2: Cartesian Space Planning (Lines 111-126)**
```python
pose_goal = PoseStamped()
pose_goal.header.frame_id = "base_link"
pose_goal.pose.position.x = 0.4
pose_goal.pose.position.y = 0.1
pose_goal.pose.position.z = 0.35
pose_goal.pose.orientation.x = 0.0
pose_goal.pose.orientation.y = 0.707
pose_goal.pose.orientation.z = 0.0
pose_goal.pose.orientation.w = 0.707

ur5e_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")
```
- **Pose goal**: Specifies end-effector position (x, y, z) and orientation (quaternion)
- **Position**: 40cm forward, 10cm to the side, 35cm up (reachable from Example 1 configuration)
- **Orientation**: End-effector pointing down (standard orientation: 0, 0.707, 0, 0.707)
- **Inverse kinematics**: MoveIt computes joint angles to achieve this pose
- **Planning**: Generates trajectory that moves end-effector to target pose
- **Type**: **CARTESIAN POSITION** - End-effector pose specification

#### **Example 3: Return to Home (Lines 114-128)**
```python
ur5e_arm.set_start_state_to_current_state()
ur5e_arm.set_goal_state(configuration_name="home")
plan_and_execute(ur5e, ur5e_arm, planning_group, logger, vel=0.2, accel=0.2, sleep_time=2.0)
```
- **State chaining**: Uses current state (from previous movement) as start
- **Home position**: Returns to safe "home" configuration
- **Type**: **JOINT CONTROL** - Predefined state from SRDF

### Step 8: Visualize the Results

**Option 1: Using RViz (if you launched MoveIt separately)**
If you launched `ur_moveit.launch.py` in Step 4, you should see the robot moving in RViz.

**Option 2: Open RViz manually**
```bash
# In a new terminal
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

**Configure RViz:**
1. Add "RobotModel" display
   - Set **Robot Description** to: `robot_description`
2. Add "Motion Planning" plugin (optional)
   - Set **Planning Scene Topic** to: `/monitored_planning_scene`
3. Set **Fixed Frame** to: `base_link`

You should see the robot model moving as the demo executes.

### Step 9: Understanding Joint Space vs Cartesian Space Planning

#### **Joint Space Planning (Examples 1 & 3)**
- **Input**: Direct joint angles (e.g., `[0, -1.57, 0, -1.57, 0, 0]`)
- **Planning**: Trajectory computed directly in joint space
- **Control**: Joint positions are directly controlled
- **Pros**: 
  - Fast and predictable
  - Always works if joint limits are respected
  - No inverse kinematics needed
- **Cons**:
  - End-effector path is not controlled
  - Hard to know where end-effector will be
  - Not intuitive for task-oriented planning

**Use when:**
- You know the exact joint configuration you want
- You need fast, predictable motion
- You're moving between predefined poses
- End-effector position doesn't matter

#### **Cartesian Space Planning (Example 2)**
- **Input**: End-effector position and orientation
- **Planning**: Inverse kinematics computed first, then trajectory in joint space
- **Control**: End-effector pose is controlled
- **Pros**:
  - Intuitive for task-oriented planning
  - End-effector path can be controlled
  - Natural for pick-and-place, reaching tasks
- **Cons**:
  - Requires IK solver (may fail if pose unreachable)
  - Slower (IK computation overhead)
  - Joint paths may be unpredictable

**Use when:**
- You need the end-effector at a specific location
- You're doing pick-and-place operations
- You need to reach a specific point in space
- Task-oriented planning (e.g., "move to this object")

### Step 10: Exercises to Try

**Basic Exercises:**
1. **Modify joint values** - Change the predefined state joint angles
2. **Change target pose** - Modify the Cartesian goal position/orientation
3. **Adjust velocity/acceleration** - Experiment with different scaling factors
4. **Add more movements** - Chain additional movements together

**Intermediate Exercises:**
1. **Create custom joint configurations** - Use `RobotState` to set custom joint angles
2. **Plan to multiple poses** - Create a sequence of Cartesian goals
3. **Add error handling** - Check if planning succeeds before execution
4. **Use different planners** - Try OMPL or CHOMP planners

**Advanced Exercises:**
1. **Implement waypoint planning** - Plan through multiple intermediate poses
2. **Add collision objects** - Plan around obstacles
3. **Create a pick-and-place sequence** - Combine multiple planning types
4. **Add trajectory visualization** - Display planned paths in RViz

## Summary

This tutorial has guided you through using MoveIt2 and MoveItPy for robot motion planning. You've learned:

- **MoveIt setup**: Launching robot drivers and MoveIt configuration
- **Joint space planning**: Direct control of joint angles
- **Cartesian space planning**: Control of end-effector pose
- **MoveItPy API**: Programmatic robot control in Python
- **State chaining**: Linking multiple movements together

### Next Steps

**Continue Learning:**
- Experiment with different planning pipelines (OMPL, CHOMP, Pilz)
- Try planning with collision objects and obstacles
- Implement more complex manipulation sequences
- Explore advanced MoveIt features (constraints, waypoints)

**Advanced Projects:**
- Implement pick-and-place operations
- Create manipulation sequences for specific tasks
- Add sensor feedback for reactive planning
- Build complete manipulation pipelines

## Troubleshooting

### Common Issues

1. **"Package not found" error**
   - Ensure you've built the package: `colcon build --packages-select moveit_demo`
   - Source the workspace: `source install/setup.bash`
   - Check you're in the correct directory: `cd ~/ros2_ws`

2. **"Unable to configure planning scene monitor" error**
   - Make sure robot driver is running: `ros2 launch ur_robot_driver ur_control.launch.py ...`
   - Check `/robot_description` topic: `ros2 topic echo /robot_description --once`
   - Verify `/joint_states` is publishing: `ros2 topic echo /joint_states --once`

3. **"No IK solution" error**
   - The target pose is unreachable from current state
   - Try a different pose or move to a different state first
   - Check pose is within robot workspace

4. **Robot doesn't move in RViz**
   - Ensure robot driver is running and publishing joint states
   - Check RViz is subscribed to correct topics
   - Verify `/joint_states` topic is active: `ros2 topic list | grep joint`

5. **Planning fails**
   - Check start state is valid (not in collision)
   - Verify goal state is reachable
   - Try increasing planning time: `params.planning_time = 5.0`
   - Check for self-collisions in the planned path

6. **"Action client not connected" error**
   - Robot driver may not be running
   - Controller may not be active
   - Check controller status: `ros2 control list_controllers`

### ROS2 Commands Reference

```bash
# Build your package
colcon build --packages-select moveit_demo

# Source the workspace
source install/setup.bash

# Run the demo
ros2 launch moveit_demo moveit_demo.launch.py

# Check topics
ros2 topic list
ros2 topic echo /joint_states

# Check nodes
ros2 node list

# Check parameters
ros2 param list /moveit_demo
```

## Further Reading

- **MoveIt Documentation**: https://moveit.picknik.ai/
- **MoveItPy API**: https://moveit.picknik.ai/main/api/python_api/html/
- **UR Robot Documentation**: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- **ROS2 Motion Planning**: https://docs.ros.org/en/jazzy/Tutorials.html
