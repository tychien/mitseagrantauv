#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher filtered_pub;


void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
     /*** Remove ground and ceiling ***/
    
    //pcl::IndicesPtr pc_indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZI> ptz;
    ptz.setInputCloud(pcl_pc_in);
    ptz.setFilterFieldName("z");
    ptz.setFilterLimits(-2.4,0.5);
    //ptz.filter(*pc_indices);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    ptz.filter(*pcl_pc_out);

    sensor_msgs::PointCloud2 ros_pc2_out;
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    filtered_pub.publish(ros_pc2_out);
    for(std::size_t i=0; i<pcl_pc_out->points.size(); ++i){
        std::cout << "  " << pcl_pc_out->points[i].z<< std::endl;
    }


}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "passthrough");
    ros::NodeHandle nh;
    //ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, PointCloudCallBack);
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/broadband_radar/channel_1/pointcloud", 1, PointCloudCallBack); 
    ros::NodeHandle private_nh("~");
    filtered_pub = private_nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud",100);
    ros::spin();
    return 0;
}
