import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d as tfs
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.linalg import logm, expm


def remove_outliers(H_cr_list):
    """对输入的变换矩阵进行异常值筛选."""
    translations = np.array([H_cr[:3, 3] for H_cr in H_cr_list])
    mean = np.mean(translations, axis=0)
    std_dev = np.std(translations, axis=0)

    filtered = [H_cr for H_cr in H_cr_list if np.all(np.abs(H_cr[:3, 3] - mean) < 2 * std_dev)]
    return filtered


def karcher_mean(quaternions, max_iterations=100, tolerance=1e-6):
    """计算四元数的 Karcher mean."""
    current_mean = quaternions[0]
    for _ in range(max_iterations):
        # 转换当前平均四元数为旋转矩阵
        rotation_matrix = tfs.quaternions.quat2mat(current_mean)
        sum_log = np.zeros((3, 3))  # 累加旋转矩阵的对数映射

        for q in quaternions:
            q_matrix = tfs.quaternions.quat2mat(q)
            delta_matrix = np.dot(rotation_matrix.T, q_matrix)
            log_map = logm(delta_matrix)  # 使用 scipy.linalg.logm 计算对数映射
            sum_log += log_map

        # 计算更新后的旋转矩阵
        delta = expm(sum_log / len(quaternions))  # 使用 scipy.linalg.expm 计算矩阵指数映射
        new_rotation_matrix = np.dot(rotation_matrix, delta)

        # 检查收敛
        if np.linalg.norm(delta - np.eye(3)) < tolerance:
            break

        # 更新当前平均旋转矩阵对应的四元数
        current_mean = tfs.quaternions.mat2quat(new_rotation_matrix)

    return current_mean


def fit_transformations(H_cr_list):
    """使用最小二乘法拟合相机到机器人坐标系的变换矩阵."""
    filtered_H_cr = remove_outliers(H_cr_list)
    translations = np.array([H_cr[:3, 3] for H_cr in filtered_H_cr])
    rotations = np.array([tfs.quaternions.mat2quat(H_cr[:3, :3]) for H_cr in filtered_H_cr])

    # 平均平移和旋转
    mean_translation = np.mean(translations, axis=0)
    mean_rotation = karcher_mean(rotations)

    # 构造平均变换矩阵
    mean_H_cr = np.eye(4)
    mean_H_cr[:3, :3] = tfs.quaternions.quat2mat(mean_rotation)
    mean_H_cr[:3, 3] = mean_translation
    return mean_H_cr


class CameraToBaseNode(Node):
    def __init__(self):
        super().__init__('camera_to_base_transform')

        # Define parameters for marker ids and their transformations
        self.declare_parameter('marker_ids', [1,2,3,4])
        self.declare_parameter('transform_count', 100)

        self.marker_ids = self.get_parameter('marker_ids').get_parameter_value().integer_array_value
        self.transform_count = self.get_parameter('transform_count').value

        # 预定义的从base到每个marker的变换矩阵
        self.marker_to_base_transforms = {
            1: np.array([[0, 1, 0, -0.0126],
                         [-1, 0, 0, -0.082], 
                         [0, 0, 1, 0.650],
                         [0, 0, 0, 1]]),  # Replace with actual transformation
            2: np.array([[0, 1, 0, 0.0974],
                         [-1, 0, 0, -0.082],
                         [0, 0, 1, 0.650],
                         [0, 0, 0, 1]]),  # Replace with actual transformation
            3: np.array([[0, 1, 0, 0.1564],
                         [-1, 0, 0, 0.026],
                         [0, 0, 1, 0.554],
                         [0, 0, 0, 1]]) , # Replace with actual transformation
            4: np.array([[0, 1, 0, 0.2664],
                         [-1, 0, 0, 0.026],
                         [0, 0, 1, 0.554],
                         [0, 0, 0, 1]])  # Replace with actual transformation
        }

        self.H_cr_list = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        for marker_id in self.marker_ids:
            try:
                # 获取当前marker的从base到marker的预定义变换
                marker_to_base = self.marker_to_base_transforms.get(marker_id, np.eye(4))

                # 查询相机到marker的变换
                trans = self.tf_buffer.lookup_transform( f'calibration_board_{marker_id}','camera_color_optical_frame', rclpy.time.Time())

                # 将ROS 2变换转换为4x4矩阵
                camera_to_marker = np.eye(4)
                translation = [trans.transform.translation.x, trans.transform.translation.y,
                               trans.transform.translation.z]
                rotation = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y,
                            trans.transform.rotation.z]
                camera_to_marker[:3, :3] = tfs.quaternions.quat2mat(rotation)
                camera_to_marker[:3, 3] = translation

                # 计算相机到base的变换并存储在H_cr_list中
                H_cr = np.dot(marker_to_base, camera_to_marker)
                print(H_cr)
                self.H_cr_list.append(H_cr)

            except Exception as e:
                self.get_logger().info(f"Failed to get transform for marker {marker_id}: {str(e)}")
                continue  # 继续查询下一个marker

        if len(self.H_cr_list) >= self.transform_count:
            mean_H_cr = fit_transformations(self.H_cr_list)
            self.publish_transform(mean_H_cr)
            self.H_cr_list = []  # 重置列表

    def publish_transform(self, mean_H_cr):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = 'camera_color_frame'

        # Extract rotation and translation
        quat = tfs.quaternions.mat2quat(mean_H_cr[:3, :3])
        msg.transform.translation.x = mean_H_cr[0, 3]
        msg.transform.translation.y = mean_H_cr[1, 3]
        msg.transform.translation.z = mean_H_cr[2, 3]
        msg.transform.rotation.w = quat[0]
        msg.transform.rotation.x = quat[1]
        msg.transform.rotation.y = quat[2]
        msg.transform.rotation.z = quat[3]

        self.tf_broadcaster.sendTransform(msg)
        self.get_logger().info("Published camera-to-base transformation")


def main(args=None):
    rclpy.init(args=args)
    node = CameraToBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()