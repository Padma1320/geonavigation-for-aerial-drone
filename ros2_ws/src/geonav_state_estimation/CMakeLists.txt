cmake_minimum_required(VERSION 3.8)
project(geonav_state_estimation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ekf_node
  src/ekf_node.cpp
)

ament_target_dependencies(
  ekf_node
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
)

target_link_libraries(
  ekf_node
  Eigen3::Eigen
)

install(
  TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
