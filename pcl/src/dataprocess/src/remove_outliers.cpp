#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointType;

class SubscribeAndPublish{
    public:
        SubscribeAndPublish(){
            pub = n.advertise<PointCloud>("removed",1);
            sub = n.subscribe("downsampled", 1, &SubscribeAndPublish::callback, this);
        }
        
        void callback(const PointCloud input){
            //.... do something with the input and generate the output...
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            
            cloud->header.frame_id="removed";
            cloud->width = input.width;
            cloud->height= input.height;
            cloud->points.resize(cloud->width * cloud->height);
            cloud->points = input.points;  
            std::cout <<cloud->height<<" "<< cloud->width <<" ";

            pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
            // build the filter
            outrem.setInputCloud(cloud);
            outrem.setRadiusSearch(0.8);
            outrem.setMinNeighborsInRadius (2);
            // apply filter
            outrem.filter (*filtered_cloud);
            

            pub.publish(filtered_cloud); 
        
        
        }

    private:
        ros::NodeHandle n; 
        ros::Publisher pub;
        ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "removing");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
