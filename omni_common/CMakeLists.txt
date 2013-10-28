cmake_minimum_required(VERSION 2.8.3)
project(omni_common)

find_package(catkin REQUIRED COMPONENTS roscpp omni_msgs std_msgs geometry_msgs rosconsole tf sensor_msgs)

catkin_package(DEPENDS roscpp omni_msgs std_msgs geometry_msgs rosconsole tf sensor_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

find_path(OPENHAPTICS_INCLUDE_DIR	NAMES	HD/hd.h HL/hl.h HDU/hdu.h)
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

