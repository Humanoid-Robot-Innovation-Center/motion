from motor_interface.msg import VelAndPose
from sensor_msgs.msg import JointState as JS
from typing import Dict
import numpy as np


def create_default_motor_msg():
    # todo: align to new motor API
    motor_msg = VelAndPose()
    motor_msg.mode = 1
    motor_msg.vel.linear.x = 0.0
    motor_msg.vel.linear.y = 0.0
    motor_msg.vel.linear.z = 0.0
    motor_msg.vel.angular.x = 0.0
    motor_msg.vel.angular.y = 0.0
    motor_msg.vel.angular.z = 0.0
    motor_msg.pose.position.x = 0.0
    motor_msg.pose.position.y = 0.0
    motor_msg.pose.position.z = 0.0
    motor_msg.pose.orientation.x = 0.0
    motor_msg.pose.orientation.y = 0.0
    motor_msg.pose.orientation.z = 0.0
    motor_msg.pose.orientation.w = 1.0
    return motor_msg


def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = float(cr * cp * cy + sr * sp * sy)
    x = float(sr * cp * cy - cr * sp * sy)
    y = float(cr * sp * cy + sr * cp * sy)
    z = float(cr * cp * sy - sr * sp * cy)

    return w, x, y, z


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


def convert_trajectory_message(trajectory_msg: JS) -> Dict:
    """
    used to convert msg sent by motion generator
    which is a JointState with a large number of
    position and velocity along the time

    """

    joint_positions = []
    joint_velocities = []
    joint_trajectory = {
        "joint_names": None,
        "position_trajectory": None,
        "velocity_trajectory": None
    }

    joint_names = trajectory_msg.name
    joint_trajectory_positions = trajectory_msg.position
    joint_trajectory_velocities = trajectory_msg.velocity

    if len(joint_names) == 0:
        return joint_trajectory
    
    for i in range(0, len(joint_trajectory_positions), len(joint_names)):
        joint_state_position = []
        joint_state_velocity = []
        for j in range(len(joint_names)):
            joint_state_position.append(joint_trajectory_positions[i + j])
            joint_state_velocity.append(joint_trajectory_velocities[i + j])
        joint_positions.append(joint_state_position)
        joint_velocities.append(joint_state_velocity)

    joint_trajectory["joint_names"] = joint_names
    joint_trajectory["position_trajectory"] = joint_positions
    joint_trajectory["velocity_trajectory"] = joint_velocities

    return joint_trajectory
