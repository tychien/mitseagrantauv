#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_raw");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter1", 1000);
  ros::Publisher chatter_pub2= n.advertise<std_msgs::String>("chatter2", 1000);
  ros::NodeHandle m;
  ros::Publisher chatter_pub3= m.advertise<std_msgs::String>("velodyne_pt",1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg1;
    std_msgs::String msg2;
    std_msgs::String msg3;
    std::stringstream ss1;
    ss1 << "1 hello world " << count;
    msg1.data = ss1.str();
    std::stringstream ss2;
    ss2 << "2 hello world " << count;
    msg2.data = ss2.str();
    std::stringstream ss3;
    
    ss3 << "header: \n seq: 204 \n stamp: \n secs: 1572379242 \n nsecs: 794358000 \n frame_id: \"velodyne\" \n height: 1width: 7439 \n fields: \n- \n name: \"x\" \n offset: 0 \n datatype: 7 \n count: 1 \n -  \n name: \"y\" \n offset: 4 \n datatype: 7 \n count: 1 \n - \n name: \"z\" \n offset: 8 \n datatype: 7 \n count: 1 \n -  \n name: \"intensity\" \n offset: 16 \n datatype: 7 \n count: 1 \n - \n name: \"ring\" \n offset: 20 \n datatype: 4 \n count: 1 \n is_bigendian: False \n point_step: 32 \n row_step: 238048";






    msg3.data = ss3.str();
    ROS_INFO("%s", msg1.data.c_str());
    ROS_INFO("%s", msg2.data.c_str());
    ROS_INFO("%s", msg3.data.c_str());
    chatter_pub.publish(msg1);
    chatter_pub2.publish(msg2);
    chatter_pub3.publish(msg3);
    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }


  return 0;
}
