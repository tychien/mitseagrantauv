#!/usr/bin/env python

import sys
import rospy
from nmea_msgs.msg import Sentence
import socket

HOST = "localhost"
#HOST = '192.168.1.151'
#HOST = '10.189.94.39'
PORT = 2000

try:
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.connect((HOST, PORT))
except:
  print("ERROR Connecting: %s" , sys.exc_info())
  exit()
    

def sendNMEACallback(message):
  nmea_message = message.sentence
  if len(nmea_message) > 0:
    if nmea_message[-1] != '\n':
      nmea_message += '\n'
    try:
      sock.sendall(nmea_message)
      #print("Sent: %s" % nmea_message)
    except:
      print("ERROR sending %s" % sys.exc_info()[0])

def nmea_client():
  rospy.init_node('nmea_client')
  rate = rospy.Rate(100)
  pub = rospy.Publisher('nmea_received', Sentence, queue_size=10)
  rospy.Subscriber('nmea_to_send', Sentence, sendNMEACallback)
  while not rospy.is_shutdown():
    raw_message = sock.recv(1024)
    for s in str(raw_message).split("\n"):
      if s == "":
        continue
      nmea_sentence = Sentence()
      # not ideal, should be time of ingestion on frontseat but we are not using this for timing
      nmea_sentence.header.stamp = rospy.Time.now()
      # check this
      nmea_sentence.header.frame_id = "gps_wamv_link"
      nmea_sentence.sentence = s.strip()
      pub.publish(nmea_sentence)
    
    
   
try:
  nmea_client()
except rospy.ROSInterruptException:
  pass
  
