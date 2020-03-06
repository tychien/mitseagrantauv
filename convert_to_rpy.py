#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

pub_euler = rospy.Publisher('euler', Vector3, queue_size=10)
pub_euler_degrees = rospy.Publisher('euler_degrees', Vector3, queue_size=10)
pub_heading = rospy.Publisher('heading', Float32, queue_size=10)

def imu_cb(message):
  q = message.orientation
  euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[:3]
  degrees = [180/3.14*x for x in euler]
 # heading = 90-degrees[2]
  heading = 180-degrees[2]
  pub_euler.publish(euler[0], euler[1], euler[2])
  pub_euler_degrees.publish(degrees[0], degrees[1], degrees[2])
  pub_heading.publish(heading)


def log_pos():
  rospy.init_node('convert_to_rpy')
  rate = rospy.Rate(10)
  rospy.Subscriber('/imu', Imu, imu_cb)
  while not rospy.is_shutdown():
    rospy.spin()


try:
  log_pos()
except rospy.ROSInterruptException:
  pass
