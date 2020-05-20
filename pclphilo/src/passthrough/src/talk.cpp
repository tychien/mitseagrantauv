#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter1", 1000);
  ros::Publisher chatter_pub2= n.advertise<std_msgs::String>("chatter2", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg1;
    std_msgs::String msg2;
    std::stringstream ss1;
    ss1 << "1 hello world " << count;
    msg1.data = ss1.str();
    std::stringstream ss2;
    ss2 << "2 hello world " << count;
    msg2.data = ss2.str();
    ROS_INFO("%s", msg1.data.c_str());
    ROS_INFO("%s", msg2.data.c_str());
    chatter_pub.publish(msg1);
    chatter_pub2.publish(msg2);
    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }


  return 0;
}
