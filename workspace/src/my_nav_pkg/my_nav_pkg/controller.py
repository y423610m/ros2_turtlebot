#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Twist
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from my_nav_interfaces.srv import FollowPath
from rclpy.executors import MultiThreadedExecutor

import math
import time

from threading import Thread

class Controller(Node):
    x = None
    y = None
    rz = None

    path = None
    next_path_index = 0

    def __init__(self):
        super().__init__("planner")

        self.follow_sub = self.create_service(
            FollowPath,
            'follow_path',  # service name (navigator calls this)
            self.handle_follow_path
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.spin_once)
        # self.timer = self.create_timer(0.5, self.update_pose)

        self.command_queue = []
        self.prev_path_index = -1
        self.next_path_index = 0

        self.Ki = 1.0

    def spin_once(self):
        self.update_pose()

        self.move_once()

        if self.x is None:
            return
        if self.path is None:
            return

    def handle_follow_path(self, request, response):
        self.command_queue.append((request, response))
        self.get_logger().info(f"follow path {request=}")
        return response

    def move_once(self):
        if len(self.command_queue) == 0:
            return
        request, response = self.command_queue[0]

        self.path = request.path
        # self.next_path_index = 0

        while self.x is None:
            time.sleep(0.1)

        close_threshold = 0.1
        best_distance = 1e9
        best_index = self.next_path_index
        path_index = best_index
        pose = self.path.poses[path_index].pose
        distance = ( (pose.position.x-self.x)**2 + (pose.position.y-self.y)**2 ) ** 0.5
        if distance < close_threshold:
            best_index = min(path_index+1, len(self.path.poses)-1)

        # for path_index in range(len(self.path.poses)-1, -1, -1):
        #     pose = self.path.poses[path_index].pose
        #     distance = ( (pose.position.x-self.x)**2 + (pose.position.y-self.y)**2 ) ** 0.5
        #     if distance < close_threshold:
        #         best_index = min(path_index+1, len(self.path.poses)-1)
        #         break
        #     if best_distance > distance:
        #         best_distance = distance
        #         best_index = path_index

        if best_index is None:
            response.message = '1'
            self.get_logger().info(f"follow path end {response.message=}")
            return
        self.next_path_index = max(self.next_path_index, best_index)

        if self.prev_path_index != self.next_path_index:
            self.get_logger().info(f"Moving to path index {self.next_path_index}")
            self.prev_path_index = self.next_path_index

        self.get_logger().info(f"{self.next_path_index=} {len(self.path.poses)=}")
        is_reached = self.move(self.path.poses[self.next_path_index].pose)
        if is_reached and self.next_path_index +1  >= len(self.path.poses):
            # response.goal = PoseStamped()
            response.message = 'reach'
            self.get_logger().info(f"follow path end {response.message=}")

            self.command_queue.pop(0)
            self.path = None
            self.next_path_index = 0


    def update_pose(self):
        try:
            # map→base_linkの変換を取得
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            # trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            # self.get_logger().info(f"{trans=}")
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y
            q = trans.transform.rotation
            (_, _, self.rz) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            # self.get_logger().info(f"x={self.x:.2f}, y={self.y:.2f}, rz={self.rz:.2f} rad")
        except Exception as e:
            self.get_logger().warn(str(e))

    def move(self, pose):
        twist = Twist()

        x = pose.position.x
        y = pose.position.y
        (_, _, rz) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        distance = ( (pose.position.x-self.x)**2 + (pose.position.y-self.y)**2 ) ** 0.5

        is_reached = False

        dx = x-self.x
        dy = y-self.y
        norms = math.sqrt(dx**2 + dy**2)
        dot = math.cos(self.rz) * dx + math.sin(self.rz) * dy
        cos_angle = dot / norms

        is_directed = cos_angle > 0.95
        cross = math.cos(self.rz) * dy - math.sin(self.rz) * dx

        linear_speed = abs(distance * 3.0)
        linear_speed = max(0.1, min(linear_speed, 0.5))
        rotation_speed = abs(3.0 * (1.0-abs(cos_angle)))
        rotation_speed = max(0.0, min(rotation_speed, 0.4))

        close_threshold = 0.1
        if distance < close_threshold:
            if abs(rz-self.rz) * 180 / 3.1415 < 5.0:
            # if abs(rz-self.rz) < 5 * 3.14 / 180.0:
                is_reached = True
                self.Ki = 1.0
            else:
                # rotation_speed = 0.2
                self.Ki *= 1.01
                rotation_speed *= self.Ki
                if cross > 0.0:
                    twist.angular.z =  rotation_speed
                else:
                    twist.angular.z = -rotation_speed
        else:
            # A*B = |A| x |B| x cos(angleAB)
            # is_directed = abs(rz-self.rz) < 5 * 3.14 / 180.0


            # is_directed = (math.cos(self.rz)*() + math.sin(self.rz)*(y-self.y)) / (abs(x-self.x)*abs(y-self.y)) > 0.99  # and math.cos(self.rz)*(x-self.x)/abs(x-self.x) > 0
            # self.get_logger().info(f"{cos_angle=}")
            # self.get_logger().info(f"{x=} {self.x=} {y=} {self.y=} {rz=} {self.rz=} {cross=} {cos_angle=}")
            # self.get_logger().info(f"{(math.cos(self.rz)*(x-self.x) + math.sin(self.rz)*(y-self.y)) / (abs(x-self.x)*abs(y-self.y))=}")

            # rotate
            if is_directed:
                twist.linear.x = linear_speed
                rotation_speed *= 1.5
            if cross > 0.0:
                twist.angular.z =  rotation_speed
            else:
                twist.angular.z = -rotation_speed

        self.get_logger().info(f"{abs(rz-self.rz) * 180 / 3.1415=} {distance=} {rotation_speed=}")
        # self.get_logger().info(f"{distance=} {rz=} {self.rz=}")
        # self.get_logger().info(f"{pose=}")
        # self.get_logger().info(f"{twist=}") 
        self.cmd_pub.publish(twist)
        return is_reached

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()