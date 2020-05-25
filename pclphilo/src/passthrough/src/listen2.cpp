#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::ConstPtr a;
void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
      printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<PointCloud>("velodyne_points", 1, callback);
    
  ros::NodeHandle pn;
  ros::Publisher pub = pn.advertise<PointCloud> ("points2",1);
  /*
  ros::Rate loop_rate(4);
  while (pn.ok())
  {
    //pcl_conversions::toPCL(ros::Time::now(), a->header.stamp);
    pub.publish (a);
    ros::spinOnce();
    loop_rate.sleep();  
  }
*/
  ros::spin();

  return 0;
}
