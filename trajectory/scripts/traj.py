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

def add_two_ints_client(my_pose):
  rospy.wait_for_service('add_two_ints')
  try:
    add_two_ints = rospy.ServiceProxy('add_two_ints', add)
    resp1 = add_two_ints(my_pose)
    return resp1.sum
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

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
 

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print ("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print ("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    group.set_goal_tolerance(0.001)
    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.01)

    #group.allow_replanning(20)
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def plan_cartesian_path1(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##



    print("Requesting2")
    my_pose1 = Pose()
    my_pose1.position.x = 0.05
    my_pose1.position.y = -0.1
    my_pose1.position.z = 0.02
    my_pose1.orientation.x = 0.0
    my_pose1.orientation.y = 0.0
    my_pose1.orientation.z = 0.0
    my_pose1.orientation.w = 1.0
    print("reponse2")
    b=add_two_ints_client(my_pose1)



    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x=b.position.x
    wpose.position.y=b.position.y
    wpose.position.z=b.position.z





    waypoints.append(copy.deepcopy(wpose))

   


    #print(waypoints)











    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:


    return plan, fraction



  
  def plan_cartesian_path3(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##


    print("Requesting1")
    my_pose = Pose()
    my_pose.position.x = 0.05
    my_pose.position.y = -0.3
    my_pose.position.z = 0.02
    my_pose.orientation.x = 0.0
    my_pose.orientation.y = 0.0
    my_pose.orientation.z = 0.0
    my_pose.orientation.w = 1.0
    print("reponse1")
    a=add_two_ints_client(my_pose)


    print("Requesting2")
    my_pose1 = Pose()
    my_pose1.position.x = 0.05
    my_pose1.position.y = -0.1
    my_pose1.position.z = 0.02
    my_pose1.orientation.x = 0.0
    my_pose1.orientation.y = 0.0
    my_pose1.orientation.z = 0.0
    my_pose1.orientation.w = 1.0
    print("reponse2")
    b=add_two_ints_client(my_pose1)


    print("Requesting3")
    self.group = group
    pose2 = Pose()
    pose2.position.x = 0.05
    pose2.position.y = -0.3
    pose2.position.z = 0.1
    pose2.orientation.x = 0.0
    pose2.orientation.y = 0.0
    pose2.orientation.z = 0.0
    pose2.orientation.w = 1.0
    print("reponse2")
    c=add_two_ints_client(pose2)














    waypoints = []
    



    wpose = group.get_current_pose().pose

    wpose.position.x=c.position.x
    wpose.position.y=c.position.y
    wpose.position.z=c.position.z

    #rajouteeee
    ###############################################
    h=[wpose.orientation.x,wpose.orientation.y,wpose.orientation.z,wpose.orientation.w]
   # [r,g,b]=tf.transformations.euler_from_quaternion(h[0],h[1],h[2],h[3])
    [r,g,b]=euler_from_quaternion(h[0],h[1],h[2],h[3])
    g=g+23
    [h[0],h[1],h[2],h[3]]=h=tf.transformations.quaternion_from_euler(r*(math.pi/180),g*(math.pi/180),b*(math.pi/180))
    wpose.orientation.x = h[0]
    wpose.orientation.y = h[1]
    wpose.orientation.z = h[2]
    wpose.orientation.w = h[3]
    ################################################
    waypoints.append(copy.deepcopy(wpose))
    #print(waypoints)











    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:


    return plan, fraction
  def plan_cartesian_path2(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##


    print("Requesting1")
    my_pose = Pose()
    my_pose.position.x = 0.05
    my_pose.position.y = -0.3
    my_pose.position.z = 0.02
    my_pose.orientation.x = 0.0
    my_pose.orientation.y = 0.0
    my_pose.orientation.z = 0.0
    my_pose.orientation.w = 1.0
    print("reponse1")
    a=add_two_ints_client(my_pose)


    print("Requesting2")
    my_pose1 = Pose()
    my_pose1.position.x = 0.05
    my_pose1.position.y = -0.1
    my_pose1.position.z = 0.02
    my_pose1.orientation.x = 0.0
    my_pose1.orientation.y = 0.0
    my_pose1.orientation.z = 0.0
    my_pose1.orientation.w = 1.0
    print("reponse2")
    b=add_two_ints_client(my_pose1)


    print("Requesting3")
    self.group = group
    pose2 = Pose()
    pose2.position.x = 0.05
    pose2.position.y = -0.3
    pose2.position.z = 0.02
    pose2.orientation.x = 0.0
    pose2.orientation.y = 0.0
    pose2.orientation.z = 0.0
    pose2.orientation.w = 1.0
    print("reponse2")
    c=add_two_ints_client(pose2)


















    waypoints = []
    



    wpose = group.get_current_pose().pose

    wpose.position.x=c.position.x
    wpose.position.y=c.position.y
    wpose.position.z=c.position.z

    #rajouteeee
    ###############################################
    h=[wpose.orientation.x,wpose.orientation.y,wpose.orientation.z,wpose.orientation.w]
   # [r,g,b]=tf.transformations.euler_from_quaternion(h[0],h[1],h[2],h[3])
    [r,g,b]=euler_from_quaternion(h[0],h[1],h[2],h[3])
    g=g-13
    [h[0],h[1],h[2],h[3]]=h=tf.transformations.quaternion_from_euler(r*(math.pi/180),g*(math.pi/180),b*(math.pi/180))
    wpose.orientation.x = h[0]
    wpose.orientation.y = h[1]
    wpose.orientation.z = h[2]
    wpose.orientation.w = h[3]
    ################################################
    waypoints.append(copy.deepcopy(wpose))
    #print(waypoints)

    #print(waypoints)











    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:


    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)
    print(group.get_current_pose().pose)
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "new QR"
    box_pose.pose.position.x=0.0
    box_pose.pose.position.y=0.0
    #box_pose.pose.position.z=-0.05
    box_pose.pose.position.z=-0.09

    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0

    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.4, 0.7, 0.001))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
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

    print ("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print ("============ Press `Enter` to execute a saved path ...")
    raw_input()
    
    print ("publish pour l'arduino")
    tutorial.execute_plan(cartesian_plan)
    print("============ Press `Enter` to commencer la depose ...")
    raw_input()
    publisherPython.publish("depose")
    cartesian_plan, fraction = tutorial.plan_cartesian_path2()
    tutorial.execute_plan(cartesian_plan)
    publisherPython.publish("stop")

    print("============ Press `Enter` to decoupe ...")
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path3()
    tutorial.execute_plan(cartesian_plan)
    print("decoupe")
    publisherPython.publish("decoupe")

   



    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    print( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
