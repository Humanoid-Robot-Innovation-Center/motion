from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_controller',
            executable='lichuan_driver_node',
            name='lichuan_driver_node',
            output='screen',
            parameters=[
                {
                    'wheel_base': 0.305,
                    'wheel_radius': 0.05,

                    'device_port': '/dev/stepper_motor',
                    'baud': 115200,
                    'data_bit': 8,
                    'stop_bit': 1,

                    'base_slave_id': 21,
                    'base_reduction_ratio': 0.0942,

                    'lift_slave_id': 23,
                    'lift_reduction_ratio': 0.0200072,

                    'arm_slave_id': 24,
                    'arm_reduction_ratio': 0.0308348
                }
            ]
        ),
        Node(
            package='motor_controller',
            executable='feetech_driver_node',
            name='feetech_driver_node',
            output='screen',
            parameters=[
                {
                    'wrist_port': '/dev/wrist',
                    'head_port': '/dev/head',
                    'baud': 1000000,
                    'id_wrist_roll': 13,
                    'id_wrist_pitch': 12,
                    'id_wrist_yaw': 11,
                    'id_gripper_pull': 14,
                    'id_head_pitch': 16,
                    'id_head_yaw': 15,
                    'id_camera_pitch': 18,
                    'id_camera_yaw': 17,
                }
            ]
        ),
        Node(
            package='motor_controller',
            executable='joint_state_publisher_node',
            name='joint_state_publisher_node',
            output='screen',
            parameters=[
                {
                    'stepper_device_port': '/dev/stepper_motor',
                    'wrist_device_port': '/dev/wrist',
                    'head_device_port': '/dev/head',
                    'id_wrist_roll': 13,
                    'id_wrist_pitch': 12,
                    'id_wrist_yaw': 11,
                    'id_gripper_pull': 14,
                    'id_head_pitch': 16,
                    'id_head_yaw': 15,
                    'id_camera_pitch': 18,
                    'id_camera_yaw': 17,
                }
            ]
        )
    ])
