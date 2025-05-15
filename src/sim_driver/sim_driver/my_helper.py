import numpy as np
from omni.isaac.debug_draw import _debug_draw

def quaternion_to_euler(w, x, y, z):
    # 计算roll (x轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # 计算pitch (y轴旋转)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # pitch接近 ±90度时的处理，避免超出范围
    else:
        pitch = np.arcsin(sinp)

    # 计算yaw (z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# # 绘制末端规划路线
# draw_tj  = _debug_draw.acquire_debug_draw_interface()
# def draw_traj(ee_pos_seq):
#     if ee_pos_seq == None or len(ee_pos_seq) == 0:
#         return
#     # Standard Library
#     b,h = ee_pos_seq.shape
#     point_list = []
#     for i in range(b):
#         point_list += [(ee_pos_seq[i,0],ee_pos_seq[i,1],ee_pos_seq[i,2])]
#     colors = [(0,0,1,1) for _ in range(b)]
#     sizes = [10.0 for _ in range(b)]
#     draw_tj.draw_points(point_list, colors, sizes)