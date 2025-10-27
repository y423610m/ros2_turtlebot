#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from my_nav_interfaces.srv import PlanPath
import random

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
        start = request.start
        goal = request.goal

        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        # Create a simple line path (x=0→4)
        for i in range(5):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(i)
            pose.pose.position.y = 1.0 * (i%2)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        if random.randint(0,1) % 2 == 0:
            path.poses.reverse()
            print("reverse")
        else:
            print("true")
        path.poses.append(goal)

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