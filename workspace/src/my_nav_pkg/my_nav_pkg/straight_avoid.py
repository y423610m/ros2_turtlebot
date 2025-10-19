import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class MyTurtleBotBase(Node):
    def __init__(self, name='turtle_bot_base'):
        super().__init__(name)

        # Subscribe to /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publish velocity commands to /cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Movement parameters
        self.forward_speed = 0.2       # m/s
        self.min_distance = 0.5        # m (stop if obstacle is closer than this)
        self.angle_range_deg = 30      # front ±30°
        self.center_index = 0

        self.get_logger().info('Obstacle avoider node started.')

    def scan_callback(self, msg: LaserScan):
        # Convert front ±angle_range_deg to indices
        total_points = len(msg.ranges)
        angle_increment = msg.angle_increment
        mid_index = total_points // 2
        half_range = int(math.radians(self.angle_range_deg) / angle_increment)

        # def is_in_range(index, center_index, total_points, angle_range_dig):
            # return min(abs(center_index-index), abs(total_points-1-))

        # Front ranges (±angle_range_deg)
        range_index = int(self.angle_range_deg/180*total_points)
        front_ranges = msg.ranges[-range_index:] + msg.ranges[0:range_index]
        self.get_logger().info(f"{total_points=} {range_index=}")
        # front_ranges = msg.ranges[mid_index - half_range : mid_index + half_range]

        # Filter out invalid (inf) readings
        valid_ranges = [r for r in front_ranges if not math.isinf(r)]

        # Determine if there's an obstacle
        min_front_distance = min(valid_ranges) if valid_ranges else float('inf')

        twist = Twist()
        self.get_logger().info(f'{min_front_distance=}')
        if min_front_distance > self.min_distance:
            # No obstacle → move forward
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
        else:
            # Obstacle too close → stop
            twist.linear.x = 0.0
            twist.angular.z = 0.3

        self.cmd_pub.publish(twist)

class ObstacleAvoider(MyTurtleBotBase):
    def __init__(self):
        super().__init__('obstacle_avoider')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
