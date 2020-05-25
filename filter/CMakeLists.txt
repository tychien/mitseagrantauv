cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(passthrough)

find_package(PCL 1.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS pcl_ros rospy roscpp std_msgs)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})



link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (passthrough passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})
