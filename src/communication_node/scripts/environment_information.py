#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           MohammadHossein GohariNejad
# License:  BSD 3 clause

Utils package for environment information extraction.
"""

import tf
import rospy
import rosservice
from gazebo_information_plugins.srv import *


class GetInfo:
    def __init__(self,service_name=""):
        temp_list=rosservice.rosservice_find("distance_serivce")        rospy.wait_for_service(service_name)
        try:
                self.client=rospy.ServiceProxy(service_name, distance_serivce)
                print ("gazebo ",service_name," found")
        except rospy.ServiceException:
                print ("Service call failed ", service_name,"  not good")

    def request1(self, command="walls", robot1="sos1", robot2="sos2"):
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None]
        try:
            response = self.client(request)
            output = [response.distance, response.number_of_objects]
        except Exception as e:
            print (e)
            print ("sending the request failed")
            response = "failed"
            print(command,"---",robot1,"---",robot2)
        return output


temp_list=rosservice.rosservice_find("distance_serivce")
environment_info=[]
for i in temp_list:
    environment_info.append(GetInfo(i))
counter=0
def get_current_position(name_space):
    """Get current location of robot using tf translation
    :parameter
    namespace : string, robot namespace

    :returns
    location : float, location of robot

    Relations
    ----------
    subscribes from /map, /base_link topics,
    """
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    flag = True
    transform = 0
    while flag and not rospy.is_shutdown():
        try:
            # TODO lacks compatibility, uses /map which may be not available
            (transform, rot) = listener.lookupTransform((name_space + '/map'), (name_space + '/base_link'),
                                                        rospy.Time(0))
            flag = False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return transform


def get_n_walls_between(ns1, ns2):
    """Returns number of objects between object1 and object2 in Gazebo
    as well as distance between them
    :parameter
    ns1 : string, object1 namespace

    ns2 : string, object2 namespace

    :returns
    number_of_walls  : integer, number of all the objects between 2 robots
    should the name of robots be wrong or no model can be found with given names
    this method returns -1
    Relations
    ----------
    """
    output_info = environment_info.request(command="walls", robot1=ns1, robot2=ns2)
    distance = output_info[0]
    number_of_walls = output_info[1]
    if distance == -1:
        print("wrong model name")
        return -1
    return number_of_walls


def get_object_distance(ns1, ns2):
    """Returns object1 and object2 distances in Gazebo
    :parameter
    ns1 : string, object1 namespace

    ns2 : string, object2 namespace

    :returns
    distance : float, distance between 2 robot

    Relations
    ----------
    """
    global counter
    global environment_info
    global temp_list
    print (len(func_list))
    if counter >=len(environment_info) :
        counter =0
    output_info = (environment_info[counter]).request1(command="distance", robot1=ns1, robot2=ns2)
    counter=counter+1
    distance = output_info[0]
    if distance == -1:
        print("wrong model name")
    return distance


def get_temp():
    """Returns Temperature of Atmosphere in Gazebo

    :NOTE needs gazebo-8
    :returns
    temp : float,

    Relations
    ----------
    """

    output_info = environment_info.request(command="temp")
    temp = output_info[0]
    return temp


def get_pressure():
    """Returns Pressure of Atmosphere in Gazebo

    :NOTE needs gazebo-8
    :returns
    pressure : float,

    Relations
    ----------
    """

    output_info = environment_info.request(command="pressure")
    pressure = output_info[0]
    return pressure
