cmake_minimum_required(VERSION 3.5)
project(my_turtlebot_spawner)

# C++14 をデフォルトに
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# -----------------
# C++ ノード
# -----------------
# add_executable(spawn_waffle_cpp src/spawn_waffle_cpp.cpp)
# ament_target_dependencies(spawn_waffle_cpp
#   rclcpp
#   gazebo_msgs
#   gazebo_ros
# )

# install(TARGETS spawn_waffle_cpp
#   DESTINATION lib/${PROJECT_NAME}
# )

# -----------------
# Python ノード
# -----------------
install(
  PROGRAMS scripts/spawn_waffle.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------
# Launch ファイル
# -----------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# パッケージを ament で登録
ament_package()
