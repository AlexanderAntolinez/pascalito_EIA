import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pascalito/ros2_ws_pascalv1/src/motor_control_pkg/install/motor_control_pkg'
