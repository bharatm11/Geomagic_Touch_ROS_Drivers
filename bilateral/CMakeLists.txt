cmake_minimum_required(VERSION 3.0.2)
project(bilateral)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  omni_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs omni_msgs roscpp std_msgs tf
)

add_executable(${PROJECT_NAME}_master src/master.cpp)
target_include_directories(${PROJECT_NAME}_master SYSTEM PRIVATE include ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME}_master
  PRIVATE
    -Werror
    -Wall
    -Wextra
    -Wshadow
    -Wno-long-long
    -pedantic)
target_link_libraries(${PROJECT_NAME}_master rt ncurses ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_master ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_slave src/slave.cpp)
target_include_directories(${PROJECT_NAME}_slave SYSTEM PRIVATE include ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME}_slave
  PRIVATE
    -Werror
    -Wall
    -Wextra
    -Wshadow
    -Wno-long-long
    -pedantic)
target_link_libraries(${PROJECT_NAME}_slave rt ncurses ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_slave ${catkin_EXPORTED_TARGETS})
