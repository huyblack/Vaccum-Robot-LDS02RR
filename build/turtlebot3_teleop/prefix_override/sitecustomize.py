import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/huy/nav2_ws/Vaccum-Robot-LDS02RR/install/turtlebot3_teleop'
