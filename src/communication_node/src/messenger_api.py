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
from std_msgs.msg import String


def send_message(message, robot_namespace):
    """Sending a message
    This is the message sending protocol

    :parameter
    message : string
    robot_namespace : string, indicating robot name_space

    Relations
    ----------
    """
    message_publisher = rospy.Publisher('/message_server', String, queue_size=10)
    rospy.init_node(robot_namespace + '_message_sender_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    message_publisher.publish(message)
    rate.sleep()


def register(robot_namespace):
    """Register a robot in communication_nod
    This is needed for security, reliability and etc.

    :returns
    message : string
    robot_namespace : string, indicating robot name_space

    Relations
    ----------
    """
    # TODO registration api goes here(TAHER)
    pass
