cmake_minimum_required(VERSION 2.8.3)
project(lucy_base_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()


add_executable(lucy_bc src/cmd_vel_to_steering_and_speed.cpp)
target_link_libraries(lucy_bc ${catkin_LIBRARIES})
add_dependencies(lucy_bc beginner_tutorials_generate_messages_cpp)