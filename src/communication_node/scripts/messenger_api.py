#!/usr/bin/env python
# coding=utf-8
"""Messenger API.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
# License:  BSD 3 clause

API for sending and receiving message

Relations
----------
subscribes from corresponding node's (generic)robot_namespace/inbox topic,
publishes on /message_server topic

"""

import rospy
from communication_node.msg import Data_Map, Data_Image, Data_Position
from registration_client import registration_client


def send_message(message=None, message_type=None,message_tag=""):
    """Sending a message
    This is the message sending protocol

    :parameter
    message_type : message_type in String
    message : *(message_type).msg type message

    Relations
    ----------
    """
    message_publisher = rospy.Publisher("/message_server_"+message_tag, message_type, queue_size=10)
    # rospy.init_node(robot_namespace + '_message_sender_node', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    message_publisher.publish(message)
    rate.sleep()


def __receive_message_callback(data):
    print data


def receive_message(name_space="", message_type=None,message_tag="", callback_function):
    """Receive a message
    This is the message sending protocol

    :parameter
    message_type : message_type in String
    name_space : your robot's name_space

    Relations
    ----------
    """
    rospy.Subscriber(name_space + "/inbox_"+message_tag, message_type, callback_function)


def register(robot_namespace):
    """Register a robot in communication_node
    This is needed for security, reliability and etc.

    :returns
    message : string
    robot_namespace : string, indicating robot name_space

    Relations
    ----------
    """
    # TODO registration api goes here
    result = registration_client(robot_namespace)
    print("Result:", ', '.join([str(n) for n in result.robots_list]))
    pass
