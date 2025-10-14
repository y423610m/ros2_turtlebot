#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class WaffleSpawner(Node):
    def __init__(self):
        super().__init__('spawn_entity')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            # rclpy.spin_once(self)
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Ready to spawn waffle!')

def main(args=None):
    rclpy.init(args=args)
    node = WaffleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()