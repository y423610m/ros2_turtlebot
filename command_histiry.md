
# build docker
docker compose up -d --build

# 
xhost +

docker exec -it ros2_nav2 bash
docker exec -it $(docker ps | sed -n '2p' | awk '{print $1}') bash

export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL_PATH=/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  # This takes a few minutes

# control with rviz 
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False


# aaa
<!-- gazebo --verbose /usr/share/gazebo-11/worlds/empty.world -->
<!-- ros2 run my_turtlebot_spawner spawn_waffle -->
<!-- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py -->

ros2 launch my_turtlebot_spawner spawn_waffle.launch.py


# rqt graph

rqt_graph

