#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
from tf_transformations import euler_from_quaternion
from my_nav_interfaces.srv import PlanPath, FollowPath
from rclpy.callback_groups import ReentrantCallbackGroup

class BehaviorTreeNavigator(Node):
    def __init__(self):
        super().__init__('bt_navigator')
        # Create client to request a path from the planner
        self.plan_path_client = self.create_client(
            PlanPath,
            'plan_path',
            callback_group=ReentrantCallbackGroup()
        )
        self.follow_path_client = self.create_client(
            FollowPath,
            'follow_path',
            callback_group=ReentrantCallbackGroup()
        )

        self.x = None
        self.y = None
        self.rz = None
        self.get_logger().info('Waiting for initial pose ...')
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Wait until planner service is available
        while not self.plan_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for planner service...')

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.rz) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.get_logger().info('Initial pose was set')
        

    def goal_pose_callback(self, msg: PoseStamped):
        if self.x is None:
            self.get_logger().info('Ignoring goal pose because initial pose is not set yet')
            return

        # start and goal poses
        start = PoseStamped()
        start.header.frame_id = 'odom'
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.orientation.w = 1.0

        # goal = PoseStamped()
        # goal.header.frame_id = 'odom'
        # goal.pose.position.x = 5.0
        # goal.pose.position.y = 3.0
        # goal.pose.orientation.w = 1.0
        goal = msg

        # Send service request
        self.get_logger().info('Sending path planning request...')
        req = PlanPath.Request()
        req.start = start
        req.goal = goal

        self.plan_path_future = self.plan_path_client.call_async(req)
        self.plan_path_future.add_done_callback(self.on_plan_path_response)

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Got goal pose {msg=}")
        pose = msg.po

    def on_plan_path_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'Path received with {len(res.path.poses)} points.')

            request = FollowPath.Request()
            request.path = res.path
            self.follow_path_future = self.follow_path_client.call_async(request)
            self.follow_path_future.add_done_callback(self.on_follow_path_response)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def on_follow_path_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'follow path returned {res=}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()