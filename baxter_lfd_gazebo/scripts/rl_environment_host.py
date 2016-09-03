#!/usr/bin/python

# Author: Andrew Cunningham
# Description: Framework that will manage Gazebo for restarts for use in 
#              robot learning. It will accept specific test instances that
#              specify a starting configuration, cost function and an ending
#              condition

import rospy
from gazebo_msgs.srv import *

class AbstractHost:
    def __init__(self):
        self.delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.get_models_service = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        return        

    def get_model_names(self):    
        try:
            res = self.get_models_service()
            model_names = res.model_names
            return model_names
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            exit()
    
    def delete_model(self, model_name):
        try:
            self.delete_service(model_name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            exit()
            
    def delete_all_but_baxter(self):
        model_names = self.get_model_names()
        model_names.remove('baxter')
        model_names.remove('ground_plane')
        for model_name in model_names:
            print model_name
            self.delete_model(model_name)
            
    def reset_condition(self):
        return
        
class CanPickAndPlaceTask(AbstractHost):
    """Class used to manage the Gazebo environment for the can
    pick and place task"""
    
    def reset(self):
        self.delete_all_but_baxter()
        
test = CanPickAndPlaceTask()
test.reset()