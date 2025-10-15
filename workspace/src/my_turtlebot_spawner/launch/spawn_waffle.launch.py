from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import TimerAction

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

    return LaunchDescription([
        gazebo_server,
        gazebo_gui,
        spawn_python,
        # spawn_cpp
    ])
