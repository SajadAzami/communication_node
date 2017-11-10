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
from gazebo_information_plugins.srv import *


class GetInfo:
    def __init__(self):
        self.client1 = None
        self.client2 = None
        self.client3 = None
        self.client4 = None
        rospy.wait_for_service("GzInfo_service1")
        try:
            self.client1 = rospy.ServiceProxy("GzInfo_service1", distance_serivce)
            print ("gazebo server1 found")
        except rospy.ServiceException:
            print ("Service call failed ")
        rospy.wait_for_service("GzInfo_service2")
        try:
            self.client2 = rospy.ServiceProxy("GzInfo_service2", distance_serivce)
            print ("gazebo server2 found")
        except rospy.ServiceException:
            print ("Service call failed ")
        rospy.wait_for_service("GzInfo_service3")
        try:
            self.client3 = rospy.ServiceProxy("GzInfo_service3", distance_serivce)
            print ("gazebo server3 found")
        except rospy.ServiceException:
            print ("Service call failed ")
        rospy.wait_for_service("GzInfo_service4")
        try:
            self.client4 = rospy.ServiceProxy("GzInfo_service4", distance_serivce)
            print ("gazebo server4 found")
        except rospy.ServiceException:
            print ("Service call failed ")

    def request1(self, command="walls", robot1="sos1", robot2="sos2"):
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None]
        try:
            response = self.client1(request)
            output = [response.distance, response.number_of_objects]
        except Exception as e:
            print (e)
            print ("sending the request failed")
            response = "failed"
            print(command,"---",robot1,"---",robot2)
        return output

    def request2(self, command="walls", robot1="sos1", robot2="sos2"):
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None]
        try:
            response = self.client2(request)
            output = [response.distance, response.number_of_objects]
        except Exception as e:
            print (e)
            print ("sending the request failed")
            response = "failed"
            print(command,"---",robot1,"---",robot2)
        return output

    def request3(self, command="walls", robot1="sos1", robot2="sos2"):
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None]
        try:
            response = self.client3(request)
            output = [response.distance, response.number_of_objects]
        except Exception as e:
            print (e)
            print ("sending the request failed")
            response = "failed"
            print(command,"---",robot1,"---",robot2)
        return output

    def request4(self, command="walls", robot1="sos1", robot2="sos2"):
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None]
        try:
            response = self.client4(request)
            output = [response.distance, response.number_of_objects]
        except Exception as e:
            print (e)
            print ("sending the request failed")
            response = "failed"
            print(command,"---",robot1,"---",robot2)
        return output


environment_info = GetInfo()
func_list=[environment_info.request1,environment_info.request2,environment_info.request3,environment_info.request4]
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
    global func_list
    print (len(func_list))
    if counter >3 :
        counter =0
    output_info = func_list[counter](command="distance", robot1=ns1, robot2=ns2)
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
