#!/usr/bin/env python

import rospy
import tf
from tf import TransformListener
import time
import csv
import thread

def stopCheckThread(list):
    """Thread used to check to see if the user has pressed enter"""
    raw_input("Recording, press enter to stop recording")
    list.append(None)

class RecorderNode:
    """This node is responsible for recording the pose of the end effector
    over the course of a human demonstration, it will log the poses over time
    as a csv with each line containing a pose. I.E. a line will look like:
    time,X,Y,Z,roll,pitch,yaw"""
    
    def __init__(self, csvName = "recording1.csv"):
        rospy.init_node('my_node_name')
        self.tf = TransformListener()
        self.poseList = [] #  X,Y,Z,quat1,quat2,quat3,quat4
        self.csvName = csvName
        self.startTime = time.time()

    def logPose(self):
        """get pose from TF tree"""
        if self.tf.frameExists("/base") and self.tf.frameExists("/right_gripper"):
            t = self.tf.getLatestCommonTime("/base", "/right_gripper")
            position, quaternion = self.tf.lookupTransform("/base", "/right_gripper", t)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            currentTime = time.time() - self.startTime
            poseInstance = [currentTime, position[0], position[1], position[2], roll, pitch, yaw]
            self.poseList.append(poseInstance)
            return
    
    def writeCSV(self):
        myfile = open(self.csvName, 'wb')
        wr = csv.writer(myfile)
        for poseInstance in self.poseList:
            wr.writerow(poseInstance)
            
    def runNode(self):
        list = []
        thread.start_new_thread(stopCheckThread, (list,))
        while not list:
            self.logPose()
            time.sleep(.1)
        self.writeCSV()

myNode = RecorderNode()
myNode.runNode()