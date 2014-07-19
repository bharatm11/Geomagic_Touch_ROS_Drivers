cmake_minimum_required(VERSION 2.8.3)
project(omni_common)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs 
  omni_msgs 
  sensor_msgs 
  std_msgs 
  tf
)

catkin_package(
CATKIN_DEPENDS 
  geometry_msgs 
  omni_description 
  omni_msgs 
  robot_state_publisher 
  sensor_msgs 
  std_msgs 
  tf
  xacro
)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h HL/hl.h HDU/hdu.h)

if(NOT OPENHAPTICS_INCLUDE_DIR)
  set(OPENHAPTICS_MESSAGE "OPENHAPTICS was not found. If you do not have it you will not be able to use the Phantom Omni.")
  message(STATUS "${OPENHAPTICS_MESSAGE}") 
  return()
endif(NOT OPENHAPTICS_INCLUDE_DIR)
  
add_executable(omni src/omni.cpp)
target_link_libraries(omni HD HDU rt ncurses ${catkin_LIBRARIES})
add_dependencies(omni omni_msgs_gencpp)

add_executable(omni_state src/omni_state.cpp)
target_link_libraries(omni_state HD HDU rt ncurses ${catkin_LIBRARIES})
add_dependencies(omni_state omni_msgs_gencpp)

