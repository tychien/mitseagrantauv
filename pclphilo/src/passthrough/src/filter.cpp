#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
class SubscribeAndPublish{
    public:
        SubscribeAndPublish(){
            pub = n.advertise<PointCloud>("filtered",1);
            sub = n.subscribe("velodyne_points", 1, &SubscribeAndPublish::callback, this);
        }
        
        void callback(const PointCloud input){
            //.... do something with the input and generate the output...
        //    PointCloud output;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            cloud->width = input.width;
            cloud->height = input.height;
            cloud->points.resize (cloud->width * cloud->height);
            cloud->points = input.points;
            
            std::cout << input.width<< " ";
              
          //  output = input;
            // Create the filtering object
             
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, 1.0);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*filtered_cloud);            


            pub.publish(filtered_cloud);
        }

    private:
        ros::NodeHandle n; 
        ros::Publisher pub;
        ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "subscribe_and_publish");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
