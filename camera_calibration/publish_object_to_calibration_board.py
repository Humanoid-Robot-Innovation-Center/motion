import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf_transformations
import time

class TFPublisher(Node):

    def __init__(self):
        super().__init__('tf_publisher')
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        # 创建TF缓存和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)
        self.query_timer = self.create_timer(1.0, self.query_transform)

    def broadcast_tf(self):
        # 创建一个TransformStamped消息
        t = TransformStamped()

        # 设置时间戳
        t.header.stamp = self.get_clock().now().to_msg()

        # 设置父坐标系和子坐标系的名称
        t.header.frame_id = "object"
        t.child_frame_id = "ground_link"

        # 平移 (重合，平移为零)
        t.transform.translation.x = -0.6
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.045

        # 旋转 (重合，单位四元数)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # 通过TF广播器发送消息
        self.tf_broadcaster.sendTransform(t)

    def query_transform(self):
        try:
            # 查询 link_head_pan 相对于 base_link 的变换
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'link_head_pan', now)
            
            # 打印变换结果
            self.get_logger().info(f"Translation: {trans.transform.translation}")
            self.get_logger().info(f"Rotation: {trans.transform.rotation}")
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

