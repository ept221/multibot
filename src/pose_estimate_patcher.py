#!/usr/bin/env python

# This node subscribes to a topic published by rviz which gives a pose estimate for a robot
# and corrects its frame_id so that it will properly work in a multi-robot system.
# In a regular single robot system, rviz publishes pose estimates to /initialpose. In 
# the multibot system, rviz is configured to publish into specific namespaces, eg.
# /robot1/initialpose/patch and /robot2/initialpose/patch. This node lives inside one of the
# robot namespaces, and subscribes to initialpose/patch. It then corrects the message
# published by rviz by changing the frame_id from "map" to "/map" and then publishes this modified
# message to initialpose/ where the bot's navigation stack is waiting for it.

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
  msg.header.frame_id = "/map"
  pub.publish(msg)
  print msg

sub_topic = "initialpose/patch"
pub_topic = "initialpose/"

rospy.init_node("pose_estimate_patcher")
sub = rospy.Subscriber(sub_topic, PoseWithCovarianceStamped, callback)
pub = rospy.Publisher(pub_topic, PoseWithCovarianceStamped, queue_size=1)
rospy.spin()
