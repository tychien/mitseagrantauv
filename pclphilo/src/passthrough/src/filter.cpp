#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
//void chatterCallback(const std_msgs::String::ConstPtr& msg)
//{
//    ROS_INFO("I heard: [%s]", msg->data.c_str());
//}

//void chatterCallback(const sensor_msgs::PointCloud2& pclmsg);
void callback(const sensor_msgs::PointCloud2ConstPtr&);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("velodyne_points",1,callback);
    ros::spin();

    return 0;
}
