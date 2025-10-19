from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Gazebo server（libgazebo_ros_factory.so で ROS2 サービス有効化）
    # gazebo_server = Node(
    #     package='gazebo_ros',
    #     executable='gzserver',
    #     output='screen',
    #     arguments=['--verbose', '-s', 'libgazebo_ros_factory.so', '/usr/share/gazebo-11/worlds/empty.world']
    # )

    # # Gazebo GUI
    # gazebo_gui = Node(
    #     package='gazebo_ros',
    #     executable='gzclient',
    #     output='screen'
    # )
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', '/usr/share/gazebo-11/worlds/empty.world'],
        output='screen'
    )

    # Gazebo GUI
    gazebo_gui = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    spawn_python = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='my_turtlebot_spawner',
                executable='spawn_waffle.py',
                output='screen'
            )
        ]
    )

    # spawn_cpp = TimerAction(
    #     period=3.0,
    #     actions=[
    #         Node(
    #             package='my_turtlebot_spawner',
    #             executable='spawn_waffle_cpp',
    #             output='screen'
    #         )
    #     ]
    # )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # Load the proper URDF based on TURTLEBOT3_MODEL
            'robot_description': open(
                f'/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_{os.environ["TURTLEBOT3_MODEL"]}.urdf'
            ).read()
        }]
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_gui,
        spawn_python,
        robot_state_publisher_node,
        # spawn_cpp
    ])
