#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNavNode(Node):
    def __init__(self):
        super().__init__('my_nav_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2Hz
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello from Python node {self.count}"
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
