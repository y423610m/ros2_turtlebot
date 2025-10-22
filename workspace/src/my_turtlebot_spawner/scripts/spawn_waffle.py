#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import os
import time

class WaffleSpawner(Node):
    def __init__(self):
        super().__init__('waffle_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            # rclpy.spin_once(self)
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Ready to spawn waffle!')

        self._spawn_robot()
        # self._spawn_4walls()
        # self._spawn_obs_walls()
        # self._spawn_obs_cylinder()
        # self._spawn_obs_box()

    def _spawn_robot(self):
        try:
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
            # self.future.add_done_callback(self.done_cb)
        except Exception as e:
            self.get_logger().info(f"Failed to spawn robot. {e=}")
        time.sleep(1)

    def _spawn_4walls(self):
        self.get_logger().info('Spawning 4walls')
        try:
            self.req = SpawnEntity.Request()

            sdf_path = '/workspace/src/my_turtlebot_spawner/objects/4walls.sdf'
            with open(sdf_path, 'r') as f:
                sdf_xml = f.read()
            self.get_logger().info(f'{sdf_xml=}')
            
            for iBox in range(1):

                # Create request
                req = SpawnEntity.Request()
                req.name = 'walls'
                req.xml = sdf_xml
                req.robot_namespace = ''
                req.reference_frame = 'world'

                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                req.initial_pose = pose

                # Send request asynchronously
                future = self.cli.call_async(req)
                # rclpy.spin_until_future_complete(self, future)

                # self.future = self.cli.call_async(self.req)
                # self.future.add_done_callback(self.done_cb)

        except Exception as e:
            self.get_logger().warning(f"Failed to spawn walls {e=}")
        time.sleep(1)

    def _spawn_obs_walls(self):
        self.get_logger().info('Spawning walls')
        try:
            self.req = SpawnEntity.Request()

            sdf_path = '/workspace/src/my_turtlebot_spawner/objects/long_wall.sdf'
            with open(sdf_path, 'r') as f:
                sdf_xml = f.read()
            self.get_logger().info(f'{sdf_xml=}')

            # Create request
            req = SpawnEntity.Request()
            req.xml = sdf_xml
            req.robot_namespace = ''
            req.reference_frame = 'world'

            for idx, (x,y,rz) in enumerate([ 
                (-1., 1., 0.5), (1., -1., 0.5),
                (-3., 1., 0.5), (3., -1., 0.5),
                (-1., -3., 0.5), (1., 3., 0.5),
                (-3., -3., 0.5), (3., 3., 0.5),
             ]):
                req.name = f"obs_wall_{idx}"
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0
                req.initial_pose = pose
                future = self.cli.call_async(req)

        except Exception as e:
            self.get_logger().warning(f"Failed to spawn walls {e=}")
        time.sleep(1)

    def _spawn_obs_cylinder(self):
        self.get_logger().info('Spawning cylinder')
        try:
            self.req = SpawnEntity.Request()

            sdf_path = '/workspace/src/my_turtlebot_spawner/objects/cylinder.sdf'
            with open(sdf_path, 'r') as f:
                sdf_xml = f.read()
            self.get_logger().info(f'{sdf_xml=}')

            # Create request
            req = SpawnEntity.Request()
            req.xml = sdf_xml
            req.robot_namespace = ''
            req.reference_frame = 'world'

            for idx, (x,y) in enumerate([ 
                (1.5, 1.0),
                (3.5, 0.),
                (3.5, 2.0),
             ]):
                req.name = f"obs_cylinder_{idx}"
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0
                req.initial_pose = pose
                future = self.cli.call_async(req)

        except Exception as e:
            self.get_logger().warning(f"Failed to spawn walls {e=}")
        time.sleep(1)

    def _spawn_obs_box(self):
        self.get_logger().info('Spawning box')
        try:
            self.req = SpawnEntity.Request()

            sdf_path = '/workspace/src/my_turtlebot_spawner/objects/box.sdf'
            with open(sdf_path, 'r') as f:
                sdf_xml = f.read()
            self.get_logger().info(f'{sdf_xml=}')

            # Create request
            req = SpawnEntity.Request()
            req.xml = sdf_xml
            req.robot_namespace = ''
            req.reference_frame = 'world'

            for idx, (x,y) in enumerate([ 
                (2.0, 0.5),
                (3.5, 0.),
                (3.5, 1.5),
             ]):
                req.name = f"obs_box_{idx}"
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0
                req.initial_pose = pose
                future = self.cli.call_async(req)

        except Exception as e:
            self.get_logger().warning(f"Failed to spawn walls {e=}")
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = WaffleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()