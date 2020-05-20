cmake_minimum_required(VERSION 2.8.3)
project(passthrough)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talk src/talk.cpp)
target_link_libraries(talk ${catkin_LIBRARIES})
add_dependencies(talk beginner_tutorials_generate_messages_cpp)

add_executable(listen src/listen.cpp)
target_link_libraries(listen ${catkin_LIBRARIES})
add_dependencies(listen beginner_tutorials_generate_messages_cpp)

add_executable(listen2 src/listen2.cpp)
target_link_libraries(listen2 ${catkin_LIBRARIES})
add_dependencies(listen2 beginner_tutorials_generate_messages_cpp)
