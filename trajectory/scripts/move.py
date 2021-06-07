#!/usr/bin/env python



import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_from_euler
import tf

## END_SUB_TUTORIAL

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

####utiles
def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
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
  ##############################
  



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
    group.set_goal_tolerance(0.01)
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)
   
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link


    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    

    # ca permet davoir la pose actuelle
    pose=group.get_current_pose().pose
    print("posee:::::::::::::::::::::::::")
    print(pose)


  # bouger le robot en articulaire
  def go_to_joint_state(self):
.
    group = self.group

    
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3


    group.go(joint_goal, wait=True)


    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.5)



  def go_to_pose_goal(self,k,m,p,h):
    
    group = self.group

   
    pose_goal = geometry_msgs.msg.Pose()
    print(h)
    pose_goal.orientation.x = h[0]
    pose_goal.orientation.y = h[1]
    pose_goal.orientation.z = h[2]
    pose_goal.orientation.w = h[3]
    pose_goal.position.x = k
    pose_goal.position.y = m
    pose_goal.position.z = p
    group.set_pose_target(pose_goal)

   
    plan = group.go(wait=True)
    
    group.stop()
    
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.1)


  

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

  

    print "============ Press `Enter` to execute a movement using a pose goal ..."
   

    r=0.0
    g=0.0
    b=0.0

    # on peut enregitrer une position initiale dans laquelle la config du robot permet de faire une depose
    # k m p correspondent a la position 
    k=-0.122834196968
    m=0.248392652225
    p=0.42890141044
    # h correpond a lorientation en quat
    h=[ 0.0789772857029,-0.0818255911872,0.673496930954,0.730389652833]


    # on repasse en euler 
    [r,g,b]= euler_from_quaternion(h[0],h[1],h[2],h[3])
    print("r :"+str(r)+"b :"+str(b)+"g :"+str(g))


    raw_input()
    # on va a la position enregistrer
    tutorial.go_to_pose_goal(k,m,p,h)

    # c correspond a la correction initaile on peut la modifie 
    c=20
    while(1==1):
        name = raw_input('Enter la touche: ')
        if(name=="c"):
          c = raw_input('choisir valeur de correction : ')
        if(name=='x'):
                k=k-0.05
		print(k)
                tutorial.go_to_pose_goal(k,m,p,h)
        if(name=='y'):
                m=m+0.1
		print(m)
                tutorial.go_to_pose_goal(k,m,p,h)
        if(name=='z'):
                p=p+0.1
		print(p)
                tutorial.go_to_pose_goal(k,m,p,h)
        if(name=='r'):
                r=r+int(c)
            
                h=tf.transformations.quaternion_from_euler(r*(math.pi/180),g*(math.pi/180),b*(math.pi/180))
		print(r)
                tutorial.go_to_pose_goal(k,m,p,h)
        if(name=='g'):
                g=g+int(c)
                h=tf.transformations.quaternion_from_euler(r*(math.pi/180),g*(math.pi/180),b*(math.pi/180))
          

		print(g)
                tutorial.go_to_pose_goal(k,m,p,h)
        if(name=='b'):
                b=b+int(c)
                h=tf.transformations.quaternion_from_euler(r*(math.pi/180),g*(math.pi/180),b*(math.pi/180))
		print(b)
                tutorial.go_to_pose_goal(k,m,p,h)


    print "============ DOne !"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

