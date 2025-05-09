# 0: 速度模式; 1: 位置模式
int32 mode

geometry_msgs/Twist vel

geometry_msgs/Pose pose

# 加速时间, 单位: 秒
std_msgs/Float32 acc_time

# 减速时间, 单位: 秒
std_msgs/Float32 reduce_time

std_msgs/Float32 yaw
