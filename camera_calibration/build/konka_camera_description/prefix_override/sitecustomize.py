import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/konka/data/code/camera_calibration/install/konka_camera_description'
