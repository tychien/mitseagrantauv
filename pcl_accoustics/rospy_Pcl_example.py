import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

def talker():
    rospy.init_node('point_polygon')
    pubPcl = rospy.Publisher("/m_pointcloud_topic", PointCloud)
    rate = rospy.Rate(10)
    listPoints = []
    for i in range(50):
        x = -13*i*i-4*i-1
        y = -4*i*i+3*i+2
        listPoints.append(Point32(i, 0, 0.001*y))
        listPoints.append(Point32(i, 0, 0.001*x)) 
    #listPoints = [Point32(1.0, 1.0, 1.5),Point32(2.0, 1.0, 2.5),Point32(3.0, 1.0, 3.5),Point32(4.0, 1.0, 4.5)]
    while not rospy.is_shutdown():
        rospy.loginfo("pcl_publish_example")
        m_pointcloud = PointCloud()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        m_pointcloud.header = header
        m_pointcloud.points = listPoints 
        #m_pointcloud.points.append(Point32(1.0, 3.0, 3.5))
        pubPcl.publish(m_pointcloud)
        rate.sleep() 


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
