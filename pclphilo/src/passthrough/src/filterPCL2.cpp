#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef sensor_msgs::PointCloud2 cloud_msg;
class SubscribeAndPublish{
    public:
        SubscribeAndPublish(){
            pub = n.advertise<cloud_msg>("filtered",1);
            sub = n.subscribe("velodyne_points", 1, &SubscribeAndPublish::callback, this);
        }
        
        void callback(const cloud_msg input){
            //.... do something with the input and generate the output...
        //    PointCloud output;
            cloud_msg::Ptr cloud (new cloud_msg);
            //sensor_msgs::PointCloud2<pcl::PointXYZRGB>::Ptr cloud (new )
            cloud_msg::Ptr filtered_cloud (new cloud_msg);
            cloud->width = input.width;
            cloud->height = input.height;
            cloud->data.resize (cloud->width * cloud->height);
            cloud->data = input.data;
            
            std::cout << input.width<< " ";
              
          //  output = input;
            // Create the filtering object
             
            cloud_msg pass;
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
