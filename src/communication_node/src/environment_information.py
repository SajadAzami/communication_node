# coding=utf-8
"""Environment Information.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           Saman Golestannejad
# License:  BSD 3 clause

Utils package for environment information extraction.
"""

import tf
import rospy
from gazebo_information_plugins.srv import *


class GetInfo:
    def __init__(self):
        self.client = None
        rospy.wait_for_service("GzInfo_service")
        try:
            self.client = rospy.ServiceProxy("GzInfo_service", distance_serivce)
            print ("server found")
        except rospy.ServiceException:
            print ("Service call failed ")

    def request(self, command="walls", robot1="sos1", robot2="sos2"):
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None]
        try:
            response = self.client(request)
            output = [response.distance, response.number_of_objects]
        except rospy.ServiceException:
            print ("sending the request failed")
            response = "failed"
        return output


envirnment_info = GetInfo()


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
    output_info = envirnment_info.request(command="walls", robot1=ns1, robot2=ns2)
    distance = output_info[0]
    number_of_walls = output_info[1]
    if (distance == -1):
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

    output_info = envirnment_info.request(command="distance", robot1=ns1, robot2=ns2)
    distance = output_info[0]
    if (distance == -1):
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

    output_info = envirnment_info.request(command="temp")
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

    output_info = envirnment_info.request(command="pressure")
    pressure = output_info[0]
    return pressure
