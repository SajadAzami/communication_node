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
from communication_node.msg import Data_Position
from registration_client import registration_client


def send_map_message(message):
    """Sending a message
    This is the message sending protocol

    :parameter
    message : Data_Map.msg type message

    Relations
    ----------
    """
    message_publisher = rospy.Publisher('/message_server', Data_Position, queue_size=10)
    # rospy.init_node(robot_namespace + '_message_sender_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        message_publisher.publish(message)
        rate.sleep()


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
    # AZAMI registration client python ro gozashtam boro biaresh inja harjoori mikhai estefade koni
    # maybe like this
    result = registration_client(robot_namespace)
    print("Result:", ', '.join([str(n) for n in result.robots_list]))
    pass
