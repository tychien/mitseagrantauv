#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "dataprocess/ClusterArray.h"

ros::Publisher cluster_pub;

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in){

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

    std::cout << std:: endl;
    std::cout << "before:" << pcl_pc_in->width * pcl_pc_in->height << "points" << std::endl;

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
    //pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) pcl_pc_in->points.size ();
    while (pcl_pc_in->points.size () > 0.3 * nr_points){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (pcl_pc_in);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud (pcl_pc_in);
        extract.setIndices (inliers);
        extract.setNegative (false);
        
        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter (*pcl_pc_out);
        *pcl_pc_in = *pcl_pc_out; 
    }
        
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>>clusters;
    
    
        
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud (pcl_pc_in);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (pcl_pc_in);
        ec.extract (cluster_indices);
    
        int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (pcl_pc_in->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    /*
    sensor_msgs::PointCloud2 ros_pc2_out;
    pcl::toROSMsg(*cloud_cluster, ros_pc2_out);
    cluster_pub.publish(ros_pc2_out);
    */
    j++;
  }
    dataprocess::ClusterArray cluster_array;
    for(int i=0; i<clusters.size(); i++){
        if(cluster_pub.getNumSubscribers() >0){
            sensor_msgs::PointCloud2 ros_pc2_out;
            pcl::toROSMsg(*clusters[i], ros_pc2_out);
            cluster_array.clusters.push_back(ros_pc2_out);
        }
    }
    std::cout << "size=" << cluster_array.clusters.size() << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/removing/outremoved_cloud", 1, PointCloudCallBack);

    ros::NodeHandle private_nh("~");
    cluster_pub = private_nh.advertise<sensor_msgs::PointCloud2>("clusters",100);
    ros::spin();
    return 0;

}
