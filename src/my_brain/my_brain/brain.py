import rclpy
from typing import Optional
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import json
import time

class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        self.cmdPublisher = self.create_publisher(String, '/brain/commands', 10)
        # 心跳发布
        self.hbPublisher  = self.create_publisher(String, '/brain/heartbeat', 10)
        self.create_timer(0.5, self.timer_callback)
        
        self.status_subscriber = self.create_subscription(String, '/brain/feedback', self.task_feedback_callback, 10)
        self.keyboard_listener = keyboard.Listener(on_press=self.key_callback)
        self.keyboard_listener.start()

    def timer_callback(self):
    # 心跳
        msg = String()
        msg.data = 'heartbeat'
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.hbPublisher.publish(msg)
    
    def task_feedback_callback(self, msg):
    # 任务状态
        data = json.loads(msg.data)
        self.taskStatus = data.get("status")
        if self.taskStatus == "failed":
            self.get_logger().info('任务失败: "%s"' % data.get("reason"))
    
    def key_callback(self, event):
        if event == keyboard.Key.page_up:
            self.send_pickup_command()
        
        if event == keyboard.Key.page_down:
            self.send_release_command()

    def send_pickup_command(self):
        msg = String()
        cmd = {
            "cmd_type":"pickup",
            "obj_type":"bottle",
            "image_rgb_url":"/home/ubuntu/codetest/konka_sim-V1.1-2025-5-9/install/my_brain/share/my_brain/resource/rgb.png",
            "image_depth_url":"/home/ubuntu/codetest/konka_sim-V1.1-2025-5-9/install/my_brain/share/my_brain/resource/depth.png",
            "detection_2d":{
                "left_top":[388, 231],
                "right_bottom":[435, 350]
            }
        }
        msg.data = json.dumps(cmd)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.cmdPublisher.publish(msg)

        # while not self.taskStatus == "succeeded":
        #     time.sleep(0.5)
        
        # msg.data = {
        #     "cmd_type":"pickup",
        #     "obj_type":"bottle",
        #     "image_rgb_url":"rgb.jpg",
        #     "image_depth_url":"depth.png",
        #     "point_2d":[45,560]
        # }
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.cmdPublisher.publish(msg)

    def send_release_command(self):
        msg = String()
        cmd = {
            "cmd_type":"release",
            "obj_type":"bottle",
            "image_rgb_url":"/home/ubuntu/codetest/konka_sim-V1.1-2025-5-9/install/my_brain/share/my_brain/resource/rgb.png",
            "image_depth_url":"/home/ubuntu/codetest/konka_sim-V1.1-2025-5-9/install/my_brain/share/my_brain/resource/depth.png",
            "point_2d":[411, 290]
            }
        msg.data = json.dumps(cmd)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.cmdPublisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    brainNode = Brain()
    try:
        rclpy.spin(brainNode)
    except KeyboardInterrupt:
        brainNode.keyboard_listener.stop()
    finally:
        print("用户终止程序")
        brainNode.destroy_node()

if __name__ == '__main__':
    main()
