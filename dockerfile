FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    vim \
    less

RUN apt update && apt install -y \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rqt-graph \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-teleop \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL_PATH=/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models" >> /root/.bashrc


WORKDIR /workspace

CMD ["bash"]