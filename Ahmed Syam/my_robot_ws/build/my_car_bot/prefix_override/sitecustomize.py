import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/a-qassem/01DBC69DA6BBEB20/FCIS/ROB/task-for-everyone/Ahmed Syam/my_robot_ws/install/my_car_bot'
