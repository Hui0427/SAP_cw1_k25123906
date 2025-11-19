import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xinyue/ros2_ws/src/moveit_demo/install/moveit_demo'
