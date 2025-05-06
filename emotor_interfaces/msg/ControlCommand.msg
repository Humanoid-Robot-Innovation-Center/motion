uint16 motor_id      # 电机ID
uint8 control_mode   # 控制模式：0 = 位置控制, 1 = 速度控制, 2 = 力矩控制

# 以下字段根据控制模式不同含义各异
float32 position     # 位置值（用于位置控制）
float32 speed        # 速度值（用于速度控制）
int16 torque         # 力矩值（用于力矩控制）
uint16 current       # 电流值（通用参数）