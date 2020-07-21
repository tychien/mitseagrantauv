#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pose_pub_client;


int main(int argc, char **argv){

    ros::init(argc, argv, "radar_pose_client");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseArray>("/adaptive_clustering/poses", 1, CallBack);
    ros::NodeHandle private_nh("~");

}
