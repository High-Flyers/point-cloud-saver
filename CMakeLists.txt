cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_saver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  std_msgs
  tf
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node src/node.cpp)

add_dependencies(node ${catkin_EXPORTED_TARGETS})

target_link_libraries(node ${catkin_LIBRARIES})