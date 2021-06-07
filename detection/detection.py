"""



Aruco:
                A y
                |
                |
                |tag center
                X---------> x

CAMERA:


                X--------> x
                | frame center
                |
                |
                V y


"""

######################################################################

# importation des modules 

import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

from numpy import loadtxt, zeros, ones, array, linspace, logspace

from std_msgs.msg import String

import moveit_msgs.msg
from geometry_msgs.msg import Pose
import tf
import tf2_ros
import geometry_msgs.msg

import tf2_ros
import tf2_geometry_msgs 
from rospy import Time
from tf import TransformBroadcaster
###########################################################################


##################################################
#on choisit l'ID et la taille du aruco
id_to_find  = 72
marker_size  = 10#- [cm]
##############################################""



""" 
fonctions utiles 
"""
########################################################################################
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])



# euler to quat
def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]


# cette fonction sert a faire la transformation du QR au repere de base
def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame

    pose_stamped.child_frame_id=to_frame
    
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
####################################################################################################################


rospy.init_node('image_listener')




# Initialisation de variables utiles
##########################################################""
final = geometry_msgs.msg.TransformStamped()
final.header.stamp = rospy.Time.now()
final.header.frame_id = "QR"
final.child_frame_id = "world"

final.transform.translation.x = 0.0
final.transform.translation.y = 0.0
final.transform.translation.z = 0.0

final.transform.rotation.x = 0.0
final.transform.rotation.y = 0.0
final.transform.rotation.z = 0.0
final.transform.rotation.w = 1.0


broadcaster = tf2_ros.StaticTransformBroadcaster()
static_transformStamped = geometry_msgs.msg.TransformStamped()
static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = "camera"
static_transformStamped.child_frame_id = "QR"

static_transformStamped.transform.translation.x = 0
static_transformStamped.transform.translation.y = 0
static_transformStamped.transform.translation.z = 0

static_transformStamped.transform.rotation.x = 0
static_transformStamped.transform.rotation.y = 0
static_transformStamped.transform.rotation.z = 0
static_transformStamped.transform.rotation.w = 1.0



tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

my_pose = Pose()
my_pose.position.x = 0.0
my_pose.position.y = 0.0
my_pose.position.z = 0.0
my_pose.orientation.x = 0.0
my_pose.orientation.y = 0.0
my_pose.orientation.z = 0.0
my_pose.orientation.w = 1.0
pose_stamped = tf2_geometry_msgs.PoseStamped()
pose_stamped.pose = my_pose
pose_stamped.header.frame_id = "QR"

###############################################


## cette fonction sert a savoir si le aruco est bien detecté , si sa pose est correcte avant que le aruco s'enregistre
def move(a,b,c,qx,qy,qz,qw,pt,p1,p2,p3,p4,p5,p6,p7):

    pub = rospy.Publisher('/hello', Pose, queue_size=100)
    pose = Pose()
    pose.position.x = a/100
    pose.position.y = b/100
    pose.position.z = c/100
    pose.orientation.x=qx
    pose.orientation.y=qy
    pose.orientation.z=qz
    pose.orientation.w=qw
    pub.publish(pose)


    static_transformStamped.transform.translation.x = pose.position.x
    static_transformStamped.transform.translation.y = pose.position.y
    static_transformStamped.transform.translation.z = pose.position.z


    static_transformStamped.transform.rotation.x = qx
    static_transformStamped.transform.rotation.y = qy
    static_transformStamped.transform.rotation.z = qz
    static_transformStamped.transform.rotation.w = qw
    broadcaster.sendTransform(static_transformStamped)

    
    if(pt>15):
        transform = tf_buffer.lookup_transform("world",
                                    "QR", #source frame
                                    rospy.Time(0), #get the tf at first available time
                                    rospy.Duration(1.0))

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        print(pose_transformed.pose)


        final.transform.translation.x = pose_transformed.pose.position.x
        
        final.transform.translation.y = pose_transformed.pose.position.y
        final.transform.translation.z = pose_transformed.pose.position.z

        final.transform.rotation.x = pose_transformed.pose.orientation.x
        final.transform.rotation.y = pose_transformed.pose.orientation.y
        final.transform.rotation.z = pose_transformed.pose.orientation.z
        final.transform.rotation.w = pose_transformed.pose.orientation.w

        p1=final.transform.rotation.x
        p2=final.transform.rotation.y
        p3=final.transform.rotation.z
        p4=final.transform.rotation.w
        p5=pose_transformed.pose.position.x
        p6=pose_transformed.pose.position.y
        p7=pose_transformed.pose.position.z
   
    return [p1,p2,p3,p4,p5,p6,p7]
        


###########################################################################




## cette fonction permet de publier la trasformation entre le aruco et le world(qui est un repere fixe)
## precedent le aruco dependait de la caméra donc il bouge avec le repere camera 
def move2(liste):
 
    b = TransformBroadcaster()

    r = rospy.Rate(0.01)

    translation = [liste[4],liste[5],liste[6]]
    
    orientation=[liste[0],liste[1],liste[2],liste[3]]
    r = rospy.Rate(1)
    
    b.sendTransform(translation, orientation, Time.now(), 'new QR', '/world')

   



    

def main():
    #--- Get the camera calibration path
    calib_path  = ""
    camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
    camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')
    print(camera_matrix)
    print(camera_distortion)

    #--- 180 deg rotation matrix around the x axis
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    #--- Define the aruco dictionary
    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters  = aruco.DetectorParameters_create()


    #--- Capture the videocamera (this may also be a video or a picture)
    cap = cv2.VideoCapture(1)
    #-- Set the camera size as the one it was calibrated with
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN

    # variables utiles 
    pt=0

    p1=0.0
    p2=0.0
    p3=0.0
    p4=1.0
    p5=0.0
    p6=0.0
    p7=0.0
    allow=0
    while True:

        #-- Read the camera frame
        ret, frame = cap.read()

        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters,
                                cameraMatrix=camera_matrix, distCoeff=camera_distortion)
        
        pt=pt+1
        

        if ids is not None and ids[0] == id_to_find:

            #-- ret = [rvec, tvec, ?]
            #-- array of rotation and position of each marker in camera frame
            #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            #-- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip *R_tc)

            #-- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


            k, m, p = rotationMatrixToEulerAngles(R_tc)

            [qx, qy, qz, qw]=euler_to_quaternion(p,-m,k)
            
            print(pt)
            a=tvec[0]
            b=tvec[1]
            c=tvec[2]


            # pt sert de compteur ,en l'augementant on laisse plus de temps a la detection avant denregistrer les valeurs 
            if(pt<75):
                liste=move(a, b, c,qx,qy,qz,qw,pt,p1,p2,p3,p4,p5,p6,p7)
            
            else :
                allow=1
        if allow==1 :
            print("QR OK")
            move2(liste)
            cv2.putText(frame, "QR OK ", (0, 350), font, 20, (255,0, 0), 2, cv2.LINE_AA)





        #--- Display the frame
        cv2.imshow('frame', frame)


        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break


if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print 'Interrupted'
        sys.exit(0)
