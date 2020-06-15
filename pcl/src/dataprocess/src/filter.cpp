#include "ros/ros.h"
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointType;

class SubscribeAndPublish{
    public:
        SubscribeAndPublish(){
            pub = n.advertise<PointCloud>("filtered",1);
            sub = n.subscribe("velodyne_points", 1, &SubscribeAndPublish::callback, this);
        }
        
        void callback(const PointCloud input){
            //.... do something with the input and generate the output...
        //    PointCloud output;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            cloud->header.frame_id ="filter"; 
            cloud->width = input.width;
            cloud->height = input.height;
            cloud->points.resize (cloud->width * cloud->height);
            cloud->points = input.points;
            
            std::cout << input.height << " " << input.width<< " ";
              
          //  output = input;
            // Create the filtering object
             
            pcl::PassThrough<pcl::PointXYZI> passz;
            passz.setInputCloud (cloud);
            passz.setFilterFieldName ("z");
            passz.setFilterLimits (-2.4, 0.5);
            //pass.setFilterLimitsNegative (true);
            passz.filter (*filtered_cloud);            


            pcl::PassThrough<pcl::PointXYZI> passx;
            passx.setInputCloud (cloud);
            passx.setFilterFieldName ("x");
            passx.setFilterLimits (-80, 80);
            passx.filter(*filtered_cloud);

            pcl::PassThrough<pcl::PointXYZI> passy;
            passy.setInputCloud (cloud);
            passy.setFilterFieldName("y");
            passy.setFilterLimits(-80,80);
            passy.filter(*filtered_cloud);

            pub.publish(filtered_cloud);

                std::cout << " " << filtered_cloud->points[1].z << std::endl;


        }

    private:
        ros::NodeHandle n; 
        ros::Publisher pub;
        ros::Subscriber sub;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "filtering");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
