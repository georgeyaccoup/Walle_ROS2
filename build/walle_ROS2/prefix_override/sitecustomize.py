import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/george/new_ws/src/walle_ROS2/install/walle_ROS2'
