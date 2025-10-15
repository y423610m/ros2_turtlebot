#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os

class WaffleSpawner(Node):
    def __init__(self):
        super().__init__('waffle_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            # rclpy.spin_once(self)
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Ready to spawn waffle!')

        self.req = SpawnEntity.Request()

        # Load waffle URDF/SDF
        model_path = os.path.join(
            os.getenv('TURTLEBOT3_MODEL_PATH', '/usr/share/turtlebot3_gazebo/models'),
            'turtlebot3_waffle',
            'model.sdf'
        )
        with open(model_path, 'r') as f:
            sdf_xml = f.read()

        self.req.name = 'turtlebot3_waffle'
        self.req.xml = sdf_xml
        self.req.robot_namespace = ''
        self.req.reference_frame = 'world'
        self.req.initial_pose.position.x = 0.0
        self.req.initial_pose.position.y = 0.0
        self.req.initial_pose.position.z = 0.01

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.done_cb)

def main(args=None):
    rclpy.init(args=args)
    node = WaffleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()