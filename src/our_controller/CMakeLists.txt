cmake_minimum_required(VERSION 3.0.2)
project(our_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(Gap_Follow_Controller src/gap_follow.cpp)
target_link_libraries(Gap_Follow_Controller ${catkin_LIBRARIES})
add_dependencies(Gap_Follow_Controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})
add_dependencies(keyboard_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(Multiplexer src/mux.cpp)
target_link_libraries(Multiplexer ${catkin_LIBRARIES})
add_dependencies(Multiplexer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})