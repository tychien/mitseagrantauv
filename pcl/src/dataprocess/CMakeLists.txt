cmake_minimum_required(VERSION 2.8.3)
project(dataprocess)

find_package(catkin REQUIRED COMPONENTS  visualization_msgs geometry_msgs pcl_conversions pcl_ros roscpp rospy std_msgs sensor_msgs genmsg)
find_package(PCL 1.3 REQUIRED)
## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

add_message_files(FILES ClusterArray.msg)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

set(msg_files 
   "msg/ClusterArray.msg"
)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include
  LIBRARIES
  #perception_crop
# CATKIN_DEPENDS roscpp rospy
# DEPENDS system_lib
    )

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(talk src/talk.cpp)
target_link_libraries(talk ${catkin_LIBRARIES})
add_dependencies(talk beginner_tutorials_generate_messages_cpp)

add_executable(listen src/listen.cpp)
target_link_libraries(listen ${catkin_LIBRARIES})
add_dependencies(listen beginner_tutorials_generate_messages_cpp)

add_executable(listen2 src/listen2.cpp)
target_link_libraries(listen2 ${catkin_LIBRARIES})
add_dependencies(listen2 beginner_tutorials_generate_messages_cpp)

add_executable(filter src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})

#add_dependencies(filter beginner_tutorials_generate_messages_cpp )

add_executable(downsampling src/downsampling.cpp)
target_link_libraries(downsampling ${catkin_LIBRARIES})

add_executable(remove_outliers src/remove_outliers.cpp)
target_link_libraries(remove_outliers ${catkin_LIBRARIES})

add_executable(segmentation src/segmentation.cpp)
target_link_libraries(segmentation ${catkin_LIBRARIES})

add_executable(cluster src/cluster.cpp)
target_link_libraries(cluster ${catkin_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(cluster ${catkin_EXPORTED_TARGETS})
endif()


#add_executable (cluster_extraction src/cluster_extraction.cpp)
#target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

#add_executable(perception_point_cloud_demo src/point_cloud_demo.cpp)
#set_target_properties(perception_point_cloud_demo PROPERTIES OUTPUT_NAME point_cloud_demo PREFIX "")
#add_dependencies(perception_point_cloud_demo ${${passthrough}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(perception_point_cloud_demo
#  perception_crop
#  ${catkin_LIBRARIES}
#)


#add_library(perception_crop src/crop.cpp)
#add_dependencies(perception_crop ${${passthrough}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(perception_crop ${catkin_LIBRARIES})
