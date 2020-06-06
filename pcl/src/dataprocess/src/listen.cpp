#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener1"); 
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter1", 1000, chatterCallback);
  ros::NodeHandle m;
  ros::Subscriber sub2= m.subscribe("chatter2", 1000, chatterCallback);
  ros::NodeHandle o; 
  ros::Publisher chatter_pub = o.advertise<std_msgs::String>("chatter3",1000);
  ros::Rate loop_rate(10);
  int count =0;
  while(ros::ok()){
  
  std_msgs::String check;
  std::stringstream ss;
  ss<< "Second"<< count;
  check.data = ss.str();
  ROS_INFO("%s",check.data.c_str());
  chatter_pub.publish(check);
  loop_rate.sleep(); 
  ros::spin();
  count ++;
  }
  return 0;
}
