#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher downsampled_pub;

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

    std::cout << std::endl;

    std::cout << "before:" << pcl_pc_in->width * pcl_pc_in->height << "points" << std::endl;


    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(pcl_pc_in);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    sor.filter(*pcl_pc_out);

    sensor_msgs::PointCloud2 ros_pc2_out;
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    downsampled_pub.publish(ros_pc2_out);
    
    std::cout << "after: " << pcl_pc_out->width * pcl_pc_out->height << "points" << std::endl;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "downsampling");
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/passthrough/filtered_cloud",1,PointCloudCallBack);
    
    ros::NodeHandle private_nh("~");
    downsampled_pub = private_nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud",100); 
    ros::spin();
    return 0;
}
