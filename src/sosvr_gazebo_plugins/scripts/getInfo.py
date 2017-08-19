#!/usr/bin/env python

import rospy;
from sosvr_gazebo_plugins.srv import *


class GetInfo():
    def __init__(self):
        self.client=None;
        rospy.wait_for_service("distance_serivce");
        try:
           self.client = rospy.ServiceProxy("distance_serivce", distance_serivce);
           print ("server found");
        except rospy.ServiceException:
           print ("Service call failed ");


    def  Request(self,command="distance",robot1="sos1",robot2="sos2"):
           request = distance_serivceRequest(command,robot1,robot2);
           output=[None,None];
           try:
               response = self.client(request);
               output=[response.distance,response.number_of_objects];
           except rospy.ServiceException :
               print ("sending the request failed");
               response="failed";
           return output;
