#!/usr/bin/env python

# This node subscribes to a topic published by rviz which gives a pose estimate for a robot
# and corrects its frame_id so that it will properly work in a multi-robot system.
# Specifically, it changes the frame_id from "map" to "/map" 

import sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
  msg.header.frame_id = "/map"
  pub.publish(msg)
  print msg


sub_topic = "/" + sys.argv[1] + "/initialpose/patch"
pub_topic = "/" + sys.argv[1] + "/initialpose/"

rospy.init_node("pose_estimate_patcher")
sub = rospy.Subscriber(sub_topic, PoseWithCovarianceStamped, callback)
pub = rospy.Publisher(pub_topic, PoseWithCovarianceStamped, queue_size=1)
rospy.spin()
