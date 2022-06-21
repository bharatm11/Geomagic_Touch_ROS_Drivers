cmake_minimum_required(VERSION 2.8.3)
project(bilateral)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs 
  omni_msgs 
  std_msgs 
  tf
)

catkin_package(
CATKIN_DEPENDS 
  geometry_msgs 
  omni_msgs 
  robot_state_publisher 
  std_msgs 
  tf
  xacro
)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(slave src/slave.cpp)
target_link_libraries(slave rt ncurses ${catkin_LIBRARIES})
add_dependencies(slave omni_msgs_gencpp)

