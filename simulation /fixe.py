#! /usr/bin/env python

from tf import TransformBroadcaster
import rospy
import numpy as np
from rospy import Time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import tf
import math





def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]







b = TransformBroadcaster()
def call(pose):
    print(pose)
    translation = (pose.position.x, pose.position.y, pose.position.z)
    orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    #rate = rospy.Rate(0.01)  # 5hz
    b.sendTransform(translation, orientation, Time.now(), 'QR', '/camera') # camera

    #rate.sleep()


## on choisit la pose du aruco manuellement 

def main():
    rospy.init_node('my_broadcaster2')
    b = TransformBroadcaster()

    r = rospy.Rate(0.01)
    pose = Pose()
    pose.position.x=0.20
    pose.position.y=0.20
    pose.position.z=0.20

    pose.orientation.x=0.0

    pose.orientation.y=0.0
    pose.orientation.z=0.0
    pose.orientation.w=1.0

    translation = (pose.position.x, pose.position.y, pose.position.z)
    orientation=(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    orientation=euler_to_quaternion(0*(math.pi/180),-45*(math.pi/180),0*(math.pi/180))
    r = rospy.Rate(1)
    print(orientation)
    while not rospy.is_shutdown():
        b.sendTransform(translation, orientation, Time.now(), 'QR', '/world')
	print("en marche")
        r.sleep()




if __name__ == '__main__':
    main()

