# konka1_motion
konka1机器人运动控制仿真模块（Version1.2）

仿真平台对工程文件作出的修改（*代表实际运行时需要注意修改的部分）：
1. * 相机到base link的转换矩阵由订阅获取改为存储在代码段中
2. * 轮式机器人抬升夹爪等待时间较长，仿真中缩短了这个时间
3. motor_interface新增DebugData.msg,用于可视化debug
4. 小脑模块新增debug数据发布节点:(self.DebugPublisher = self.create_publisher(DebugData, '/brain/ee_pos', 10))
5. * 小脑模块新增Debug发布函数：def pub_debug(self)，并在pickup和release函数中调用
6. gongga_interface新增float32[] ee_pos_x/y/z，在运动规划服务的反馈中新增末端轨迹（世界坐标下）
7. 小脑模块的generate_motion_plan函数中记录了末端轨迹


仿真平台修改：
1. 新增末端轨迹可视化 (V1.1)
2. 新增感知数据可视化 (V1.2)
3. 电机响应不再共用同一个回调函数（V1.2）
4. 将电机回调缓存区由10增加到100,避免丢失数据（V1.2）
5. 新增感知数据筛选，以提高仿真速度，减少占用（V1.2）


工程文件修改：
1. 将底盘修改为相对运动模式（V1.1）
2. 将规划插值时间间隔改为 5 Hz，避免丢步（V1.2）
3. 将curobo库改到motion_generator功能包中，编译时复制到安装目录下，setup_env.bash文件也修改了curobo指定路径（V1.2）
4. 为了避免碰撞，pickup和release功能将先抬升，后规划，是添加碰撞体素之前的过渡方法（V1.2）
5. motion_generator子模块中，通过代码获取urdf路径，不再需要移植后在yml文件中进行修改了（V1.2）

其他：
1. 新增setup_env.bash文件用于指定isaacsim和curobo的路径（V1.1）
2. 新增start_module.sh脚本，用于启动整个运动控制模块 （V1.2）
