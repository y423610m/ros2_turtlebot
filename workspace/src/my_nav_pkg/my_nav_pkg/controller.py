#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Twist
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

import math

class Controller(Node):
    x = None
    y = None
    rz = None

    path = None
    next_index = 0

    def __init__(self):
        super().__init__("planner")

        self.scan_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.spin_once)
        # self.timer = self.create_timer(0.5, self.get_pose)


    def path_callback(self, path:Path):
        self.path = path

    def get_pose(self):
        try:
            # map→base_linkの変換を取得
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y
            q = trans.transform.rotation
            (_, _, self.rz) = euler_from_quaternion([q.x, q.y, q.z, q.w])

            self.get_logger().info(f"x={self.x:.2f}, y={self.y:.2f}, yaw={self.rz:.2f} rad")
        except Exception as e:
            self.get_logger().warn(str(e))

    def spin_once(self):
        self.get_pose()

        if self.x is None:
            return
        if self.path is None:
            return


        # find next waypoint
        if len(self.path.poses) == 0:
            return
        close_threshold = 0.1
        best_distance = 1e9
        best_index = None
        for path_index in range(len(self.path.poses)-1, -1, -1):
            pose = self.path.poses[path_index].pose
            distance = ( (pose.position.x-self.x)**2 + (pose.position.y-self.y)**2 ) ** 0.5
            if distance < close_threshold:
                best_index = min(path_index+1, len(self.path.poses)-1)
                break
            if best_distance > distance:
                best_distance = distance
                best_index = path_index

        if best_index is None:
            return
        self.next_index = max(self.next_index, best_index)

        self.get_logger().info(f"{self.next_index=} {len(self.path.poses)=}")
        self.move(self.path.poses[self.next_index].pose)

    def move(self, pose):
        twist = Twist()

        x = pose.position.x
        y = pose.position.y
        (_, _, rz) = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        distance = ( (pose.position.x-self.x)**2 + (pose.position.y-self.y)**2 ) ** 0.5

        close_threshold = 0.1
        if distance < close_threshold:
            if abs(rz-self.rz) < 5 * 3.14 / 180.0:
            # if abs(rz-self.rz) < 5 * 3.14 / 180.0:
                pass
            else:
                twist.angular.z = 0.1
        else:
            # A*B = |A| x |B| x cos(angleAB)
            # is_directed = abs(rz-self.rz) < 5 * 3.14 / 180.0
            dx = x-self.x
            dy = y-self.y
            norms = math.sqrt(dx**2 + dy**2)

            dot = math.cos(self.rz) * dx + math.sin(self.rz) * dy
            cos_angle = dot / norms
            is_directed = cos_angle > 0.9

            cross = math.cos(self.rz) * dy - math.sin(self.rz) * dx

            # is_directed = (math.cos(self.rz)*() + math.sin(self.rz)*(y-self.y)) / (abs(x-self.x)*abs(y-self.y)) > 0.99  # and math.cos(self.rz)*(x-self.x)/abs(x-self.x) > 0
            self.get_logger().info(f"{x=} {self.x=} {y=} {self.y=} {rz=} {self.rz=} {cross=}")
            # self.get_logger().info(f"{(math.cos(self.rz)*(x-self.x) + math.sin(self.rz)*(y-self.y)) / (abs(x-self.x)*abs(y-self.y))=}")

            # rotate
            if is_directed:
                twist.linear.x = 0.1
            if cross > 0.0:
                twist.angular.z = 1.0 * (1.1-abs(cos_angle))
            else:
                twist.angular.z = -1.0 * (1.1-abs(cos_angle))

        self.get_logger().info(f"{distance=} {rz=} {self.rz=}")
        self.get_logger().info(f"{pose=}")
        self.get_logger().info(f"{twist=}")
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()