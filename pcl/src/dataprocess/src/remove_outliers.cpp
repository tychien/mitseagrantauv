#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher outremoved_pub;

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

    std::cout << std:: endl;
    std::cout << "before:" << pcl_pc_in->width * pcl_pc_in->height << "points" << std::endl;

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(pcl_pc_in);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius(2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    outrem.filter(*pcl_pc_out);

    sensor_msgs::PointCloud2 ros_pc2_out;
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    outremoved_pub.publish(ros_pc2_out);
    std::cout << "after: " << pcl_pc_out->width * pcl_pc_out->height << "points" << std::endl;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "removing");
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/downsampling/downsampled_cloud",1,PointCloudCallBack);

    ros::NodeHandle private_nh("~");
    outremoved_pub = private_nh.advertise<sensor_msgs::PointCloud2>("outremoved_cloud",100); 
    ros::spin();
    return 0;
}
