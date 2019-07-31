cmake_minimum_required(VERSION 2.8.3)
project(waypoint_editor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  interactive_markers
)

catkin_package(
  INCLUDE_DIRS include

)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_editor_node src/waypoint_editor_node.cpp src/waypoint_editor.cpp)
target_link_libraries(waypoint_editor_node
   ${catkin_LIBRARIES}
)

