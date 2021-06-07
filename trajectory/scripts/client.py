#!/usr/bin/env python
from __future__ import print_function
from service_node.srv import *
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from time import time

import tf
import math

from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs

####################################################
def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True
#################################################################





# fonction qui appelle le service en envoyant la pose du point qu'on veut transformer
def transformation_client(my_pose):
  rospy.wait_for_service('transformation')
  try:
    transformation = rospy.ServiceProxy('transformation', add)
    resp1 = transformation(my_pose)
    return resp1.sum
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)


###############################################################################

## fonctions utiles
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return (roll_x*180/math.pi), (pitch_y*180/math.pi), (yaw_z*180/math.pi) # in radians


 
#####################################################################################################
class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

 
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)


    robot = moveit_commander.RobotCommander()


    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    planning_frame = group.get_planning_frame()
    print ("============ Reference frame: %s" % planning_frame)


    eef_link = group.get_end_effector_link()
    print ("============ End effector: %s" % eef_link)


    group_names = robot.get_group_names()
    print ("============ Robot Groups:", robot.get_group_names())


    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    group.set_goal_tolerance(0.001)
    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.01)

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names




  ## définition du premier point à atteindre 
  def plan_cartesian_path1(self, scale=1):
  
    group = self.group


    ## on remplit les coordonnées du point 1
    print("Requesting")
    my_pose1 = Pose()
    my_pose1.position.x = 0.0
    my_pose1.position.y = -0.1
    my_pose1.position.z = 0.09
    my_pose1.orientation.x = 0.0
    my_pose1.orientation.y = 0.0
    my_pose1.orientation.z = 0.0
    my_pose1.orientation.w = 1.0
    print("reponse")
    # on appelle le service , on recupere la reponse dans b 
    b=transformation_client(my_pose1)


    # on remplit le tableau 
    # on peut eventuellement mettre plusieurs points a la fois
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x=b.position.x
    wpose.position.y=b.position.y
    wpose.position.z=b.position.z

    waypoints.append(copy.deepcopy(wpose))





    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction


  ## définition du deuxieme point à atteindre 
  def plan_cartesian_path2(self, scale=1):
  
    group = self.group



    print("Requesting")
    my_pose = Pose()
    my_pose.position.x = 0.0
    my_pose.position.y = -0.35
    my_pose.position.z = 0.09
    my_pose.orientation.x = 0.0
    my_pose.orientation.y = 0.0
    my_pose.orientation.z = 0.0
    my_pose.orientation.w = 1.0
    print("reponse")
    a=transformation_client(my_pose)

    waypoints = []
    

    wpose = group.get_current_pose().pose

    wpose.position.x=a.position.x
    wpose.position.y=a.position.y
    wpose.position.z=a.position.z

    waypoints.append(copy.deepcopy(wpose))


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold


    return plan, fraction


  # dernier point pour la decoupe 
  def plan_cartesian_path3(self, scale=1):
    
    group = self.group


    print("Requesting")
    self.group = group
    pose2 = Pose()
    pose2.position.x = 0.0
    pose2.position.y = -0.35
    pose2.position.z = 0.13
    pose2.orientation.x = 0.0
    pose2.orientation.y = 0.0
    pose2.orientation.z = 0.0
    pose2.orientation.w = 1.0
    print("reponse")
    c=transformation_client(pose2)


    waypoints = []
    
    wpose = group.get_current_pose().pose

    wpose.position.x=c.position.x
    wpose.position.y=c.position.y
    wpose.position.z=c.position.z

    ###############################################
    # on recupere l'orientation actuelle du robot
    h=[wpose.orientation.x,wpose.orientation.y,wpose.orientation.z,wpose.orientation.w]
    # on met en Euler
    [r,g,b]=euler_from_quaternion(h[0],h[1],h[2],h[3])
    #####
    # on met l'angle quon veut 
    g=g+10


    ################
    # on se remet en quat
    [h[0],h[1],h[2],h[3]]=h=tf.transformations.quaternion_from_euler(r*(math.pi/180),g*(math.pi/180),b*(math.pi/180))
    wpose.orientation.x = h[0]
    wpose.orientation.y = h[1]
    wpose.orientation.z = h[2]
    wpose.orientation.w = h[3]
    ################################################
    waypoints.append(copy.deepcopy(wpose))


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction
  




  # envoie un message a rviz pour display la traj 
  def display_trajectory(self, plan):
 
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)



  # cette fonction permet dexecuter la traj apres le display 
  def execute_plan(self, plan):
    group = self.group


    group.execute(plan, wait=True)
    print(group.get_current_pose().pose)




  # fonction utile 
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()


      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False


  # cette fonction sert à rajouter un plan on peut choisir sa pose par rapport au aruco , on peut choisir ses dimensions ,ca sert de securite pour moveit lors du calcul de traj 
  def add_box(self, timeout=4):
   
    box_name = self.box_name
    scene = self.scene

    
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "new QR"
    box_pose.pose.position.x=0.0
    box_pose.pose.position.y=0.0
    box_pose.pose.position.z=-0.03

    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0

    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.4, 0.7, 0.001))
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)



def main():

  #rospy.init_node('EnvironnementROSHocine', anonymous=False)
  publisherPython = rospy.Publisher('signauxToArduino', String, queue_size=20)
  try:
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print ("============ Press `Enter` to add box")
    raw_input()
    tutorial.add_box()


    print("============ Press `Enter` to plan and display a Cartesian path ...")
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path1()

    print ("============ Press `Enter` to go to the first point ...")
    raw_input()
    
    print ("publish pour l'arduino")
    tutorial.execute_plan(cartesian_plan)

    print("============ Press `Enter` to display the trajectory ...")
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path2()
    print("============ Press `Enter` pour commencer la depose ...")
    raw_input()
    # on enovie le message pour activer les moteurs de l'arduino 
    publisherPython.publish("depose")
    cartesian_plan, fraction = tutorial.plan_cartesian_path2()
    tutorial.execute_plan(cartesian_plan)
    # on stop l'asservissement 
    publisherPython.publish("stop")

    print("============ Press `Enter` to decoupe ...")
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path3()
    tutorial.execute_plan(cartesian_plan)
    # une fois qu'on est a la bonne hauteur et orientation , on envoie le message à l'arduino pour la decoupe 
    print("decoupe")
    publisherPython.publish("decoupe")

    print("============ Done!")
  except rospy.ROSInterruptException:
    print( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
