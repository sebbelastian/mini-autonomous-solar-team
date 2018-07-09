cmake_minimum_required(VERSION 2.8.3)
project(rangefinder)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build sensor and reader
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sensor src/sensor.cpp)
target_link_libraries(sensor ${catkin_LIBRARIES})
add_dependencies(sensor rangefinder_generate_messages_cpp)

add_executable(reader src/reader.cpp)
target_link_libraries(reader ${catkin_LIBRARIES})
add_dependencies(reader rangefinder_generate_messages_cpp)
