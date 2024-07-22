import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rafael-victus/ROS2-Humble-for-Beginner/02-TUTORIALS/002-DEVELOPMENT/ros2_ws/install/py_srvcli'
