#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointType;

class SubscribeAndPublish{
    public:
        SubscribeAndPublish(){
            pub = n.advertise<PointCloud>("downsampled",1);
            sub = n.subscribe("filtered", 1, &SubscribeAndPublish::callback, this);
        }
        
        void callback(const PointCloud input){
            //.... do something with the input and generate the output...
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            
            cloud->header.frame_id="downsampled";
            cloud->width = input.width;
            cloud->height= input.height;
            cloud->points.resize(cloud->width * cloud->height);
            cloud->points = input.points;  
            std::cout <<cloud->height<<" "<< cloud->width <<" ";
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
            
            // Create the downsampling object 
            pcl::VoxelGrid<pcl::PointXYZI> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.1f,0.1f,0.1f);
            sor.filter(*filtered_cloud);

            pub.publish(filtered_cloud); 
        
        
        }

    private:
        ros::NodeHandle n; 
        ros::Publisher pub;
        ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "downsampling");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
