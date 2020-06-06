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
            pub = n.advertise<PointCloud>("segmentation_created",1);
            sub = n.subscribe("downsampled", 1, &SubscribeAndPublish::callback, this);
        }
        
        void callback(const PointCloud input){
            //.... do something with the input and generate the output...
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>), cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            
            cloud->header.frame_id="segmentation";
            cloud->width = input.width;
            cloud->height= input.height;
            cloud->points.resize(cloud->width * cloud->height);
            cloud->points = input.points;  
            
//            std::cout<< cloud->height <<" "<< cloud->width <<" ";            
            /*
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            cloud->header.frame_id ="filter"; 
            cloud->width = input.width;
            cloud->height = input.height;
            cloud->points.resize (cloud->width * cloud->height);
            cloud->points = input.points;
            
            std::cout << input.width<< " ";
            */


            /*  
            // Create the filtering object
             
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, 1.0);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*filtered_cloud);            


            pub.publish(filtered_cloud);
        
            */
            
            /*
            // Create the downsampling object 
            pcl::VoxelGrid<pcl::PointXYZI> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.1f,0.1f,0.1f);
            sor.filter(*filtered_cloud);

            pub.publish(filtered_cloud); 
            */

            // Create the segmentation object for the planar model and set all the parameters
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

            int i=0, nr_points = (int) cloud->points.size ();
            
            while (cloud->points.size () > 0.3 * nr_points)
            {
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud (cloud);
                seg.segment (*inliers, *coefficients);
                if (inliers->indices.size () == 0)
                {
                    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                    break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZI> extract;
                extract.setInputCloud (cloud);
                extract.setIndices (inliers);
                extract.setNegative (false);

                // Get the points associated with the planar surface
                extract.filter (*cloud_plane);

                // Remove the planar inliers, extract the rest
                extract.setNegative (true);
                extract.filter (*cloud_f);
                *cloud = *cloud_f;
            }
            pub.publish(cloud); 

            std::cout<< "segmentation_created=" << cloud->height <<" "<< cloud->width <<" ";            
          //  pub.publish(cloud_cluster);
        }

    private:
        ros::NodeHandle n; 
        ros::Publisher pub;
        ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "segmentation_creating");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
