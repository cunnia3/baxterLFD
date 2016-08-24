#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import tf

import numpy as np

def poseListGenerator(csvName='recording1.csv'):
    poseList = []
    # CSV columns = [ time | x | y | z | roll | pitch | yaw ]
    trajectory = np.genfromtxt(csvName,delimiter=',', \
        usecols = (0, 1, 2, 3, 4, 5, 6),dtype=np.float)
        
    exampleX = trajectory[:,1]
    exampleY = trajectory[:,2]
    exampleZ = trajectory[:,3]
    exampleRoll = trajectory[:,4]
    examplePitch = trajectory[:,5]
    exampleYaw = trajectory[:,6]
    
    for i in range(len(exampleX)):
        quaternion = tf.transformations.quaternion_from_euler \
            (exampleRoll[i], examplePitch[i], exampleYaw[i])
    
        myPose = Pose()
        myPose.position.x = exampleX[i]
        myPose.position.y = exampleY[i]
        myPose.position.z = exampleZ[i]
        myPose.orientation.x = quaternion[0]
        myPose.orientation.y = quaternion[1]
        myPose.orientation.z = quaternion[2]
        myPose.orientation.w = quaternion[3]
        poseList.append(myPose)
  
    return poseList

#print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
                
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
right_arm = moveit_commander.MoveGroupCommander("right_arm")
right_arm.clear_pose_targets() # get rid of previous runs targets

waypoints = poseListGenerator()
#waypoints.reverse()
#rospy.loginfo(waypoints)

#rospy.loginfo("Going to beginning!")
#right_arm.set_start_state_to_current_state()
#right_arm.set_pose_target(waypoints[-1])
#plan1 = right_arm.plan()
#right_arm.go()
#right_arm.clear_pose_targets()

fraction = 0.0
maxtries = 500
attempts = 0

right_arm.set_start_state_to_current_state()

# Plan the Carteisan path connecting the waypoints
while fraction < 1.0 and attempts < maxtries:
    rospy.loginfo("planning cart path")
    (plan3, fraction) = right_arm.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step
                                 0.0)         # jump_threshold
    attempts += 1
    rospy.loginfo("try number: %d", attempts)
    if attempts % 100 == 0:
        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

if attempts < maxtries:
    rospy.loginfo("executing cartesian path!")
    right_arm.execute(plan3)