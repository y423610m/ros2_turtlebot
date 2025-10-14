cmake_minimum_required(VERSION 3.5)
project(my_nav_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# --- C++ node ---
add_executable(my_nav_node_cpp src/my_nav_node.cpp)
ament_target_dependencies(my_nav_node_cpp rclcpp std_msgs)

install(TARGETS
  my_nav_node_cpp
  DESTINATION lib/${PROJECT_NAME})

# --- Python node ---
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/my_nav_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
