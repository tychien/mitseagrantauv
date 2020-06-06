#include <pcl/ModelCoefficients.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointType;

class SubscribeAndPublish{
    public:
        SubscribeAndPublish(){
            pub = n.advertise<PointCloud>("clustered",1);
            sub = n.subscribe("downsampled", 1, &SubscribeAndPublish::callback, this);
        }
        
        void callback(const PointCloud input){
            //.... do something with the input and generate the output...
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            cloud->header.frame_id="clustered";
            cloud->width = input.width;
            cloud->height= input.height;
            cloud->points.resize(cloud->width * cloud->height);
            cloud->points = input.points;  
            //std::cout << cloud->height <<" "<< cloud->width<<" "; 
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud (cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance (0.02); // 2cm
            ec.setMinClusterSize (100);
            ec.setMaxClusterSize (25000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud);
            ec.extract (cluster_indices);
//a           
            //pub.publish("hi"); 
            int j = 0;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                std::cout << " j= "<< j;
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points.push_back (cloud->points[*pit]); //*
                    //std::cout << " hi ";
                }
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
    
                pub.publish(cloud_cluster);
                j++; 
                //std::cout << "clustered = "<< cloud_cluster->height <<" "<< cloud_cluster->width << " ";
                //std::cout << "clustered= "<< cloud_cluster->height <<" ";
            }

        }
    private:
        ros::NodeHandle n; 
        ros::Publisher pub;
        ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "clustering");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
