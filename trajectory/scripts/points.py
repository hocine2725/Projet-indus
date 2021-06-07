#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size=20)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100


### ce programme permet de publier sous forme dun markerarray qui est un type de message rviz les points quon veut atteindre ca aide visuellement a savoir si cest bien place 

## on remplit manutellement la position des points 


while not rospy.is_shutdown():

   marker = Marker()
   marker.header.frame_id = "/new QR"
   marker.type = marker.SPHERE
   marker.action = marker.ADD
   marker.scale.x = 0.03
   marker.scale.y = 0.03
   marker.scale.z = 0.03
   marker.color.a = 1.0
   marker.color.r = 1.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = 0.0
   marker.pose.position.y = -0.05
   marker.pose.position.z = 0.1

   marker1 = Marker()
   marker1.header.frame_id = "/new QR"
   marker1.type = marker.SPHERE
   marker1.action = marker.ADD
   marker1.scale.x = 0.03
   marker1.scale.y = 0.03
   marker1.scale.z = 0.03
   marker1.color.a = 1.0
   marker1.color.r = 1.0
   marker1.color.g = 1.0
   marker1.color.b = 0.0
   marker1.pose.orientation.w = 1.0
   marker1.pose.position.x = 0.0
   marker1.pose.position.y = -0.4
   marker1.pose.position.z = 0.1



   markerArray.markers.append(marker)
   publisher.publish(markerArray)
   rospy.sleep(0.5)
   markerArray.markers.append(marker1)

   # Publish the MarkerArray
   publisher.publish(markerArray)

   rospy.sleep(0.5)
