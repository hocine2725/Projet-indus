#!/usr/bin/env python

from __future__ import print_function

from service_node.srv import add,addResponse
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs



def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


# fonction call back 
def handle(req):
   #on peut choisir les reperes dans lesquels on veut faire la transformation
   a=transform_pose(req.a, "new QR", "world")
   print("calcule")
   print(a)
   return a

def traitement_server():
   rospy.init_node('traitement_server')
   # on cree le service
   s = rospy.Service('transformation', add, handle)
   print("Ready to calculate.")
   # tourne à l'infini
   rospy.spin()

if __name__ == "__main__":
   traitement_server()
