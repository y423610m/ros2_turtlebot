#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from my_nav_interfaces.srv import PlanPath
import random

from planner_utils import create_path_zigzag, create_path_linear, create_path_a

class Planner(Node):
    def __init__(self):
        super().__init__("planner")

        self.path = None

        self.publisher_ = self.create_publisher(Path, 'path', 10)

        self.srv = self.create_service(
            PlanPath,
            'plan_path',  # service name (navigator calls this)
            self.handle_plan_path
        )

        timer_period = 1.0  # every 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Path planner started.")

    def handle_plan_path(self, request, response):
        self.get_logger().info(f"Got plan request {request=} {response=}")

        # path = create_path_zigzag(request.start, request.goal)
        path = create_path_linear(request.start, request.goal, self.get_logger())

        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        response.path = path

        self.path = path

        return response

    def timer_callback(self):
        if self.path:
            self.publisher_.publish(self.path)
            # self.get_logger().info("Published Path with %d poses" % len(self.path.poses))

def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()