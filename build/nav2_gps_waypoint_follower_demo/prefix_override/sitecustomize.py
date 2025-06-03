import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/Shared/src/TurtleRRT-Connect/MIAPR_project/install/nav2_gps_waypoint_follower_demo'
