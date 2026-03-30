import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emilio/turtlebot3_ws/src/warehouse_navigator/install/warehouse_navigator'
