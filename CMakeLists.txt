cmake_minimum_required(VERSION 2.8.3)
project(lucy_base_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()


## cmd_vel_to_ackermann
add_executable(base_controller_transform src/cmd_vel_to_ackermann.cpp)
target_link_libraries(base_controller_transform ${catkin_LIBRARIES})
add_dependencies(base_controller_transform beginner_tutorials_generate_messages_cpp)

## motor_control_debugger
add_executable(debugger src/motor_control_debugger.cpp)
target_link_libraries(debugger ${catkin_LIBRARIES})
add_dependencies(debugger beginner_tutorials_generate_messages_cpp)