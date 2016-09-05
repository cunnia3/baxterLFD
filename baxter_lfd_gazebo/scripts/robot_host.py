#!/usr/bin/python

# Author: Andrew Cunningham
# Description: Framework that specifies a Robot in Gazebo, contains methods
#              for implementing reinforcement learning

import rospy
from gazebo_msgs.srv import *

class Robot:
    """Absract base class
    joints is a dictionary mapping joint names to joint angles"""
    def __init__(self):
        self.set_model_service = rospy.ServiceProxy('set_model_configuration', SetModelConfiguration)
        self._joints = {}

    def get_joint_names(self):
        return self._joints.keys()
    
    def get_joint_values(self):
        return self._joints.values()
    
    def reset_robot(self):
        return
        
class BaxterRobot(Robot):
    def __init__(self):
        self.set_model_service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        
        self._joints = {'right_s0': 0, 'right_s1': 0, 'right_e0': 0,
                        'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0,
                        'left_s0': 0, 'left_s1': 0, 'left_e0': 0, 'left_e1': 0, 
                        'left_w0': 0, 'left_w1': 0, 'left_w2': 0}
        self.default_joints = self._joints.copy()    

    def reset_robot(self):
        model_name = 'baxter'
        joint_names = self.get_joint_names()
        joint_positions = self.default_joints.values()
        
        try:
            res = self.set_model_service(model_name=model_name, joint_names = joint_names, joint_positions = joint_positions)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            exit()
        