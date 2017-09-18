#!/usr/bin/env python
# coding=utf-8
"""Message Handler.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
# License:  BSD 3 clause

Plays the server role in communication_node

Relations
----------
subscribes from /message_server topic,
publishes on corresponding nodes /ns/message_status topic

"""

import rospy
from communication_node.msg import Data_Position

from environment_information import get_object_distance
from propagation_models import one_slope_model_checker


def callback(data):
    # TODO handle for different message types
    # TODO prop_model = data.prop_model
    prop_model = '1sm'
    if prop_model == '1sm':
        # distance = get_object_distance("pioneer3at", "Dumpster")
        distance = get_object_distance(data.destination, data.source)
        print distance
        result = one_slope_model_checker(distance=distance)
        if result:
            message_publisher = rospy.Publisher(data.destination + '/inbox', Data_Position, queue_size=10)
            rate = rospy.Rate(10)  # 10hz
            while not rospy.is_shutdown():
                message_publisher.publish(data)
                rate.sleep()
            print "communication is possible"
        else:
            # TODO, ignore the message, send feedback
            print "communication is not possible"


def listener():
    rospy.init_node('communication_node_message_handler')
    rospy.Subscriber("/message_server", Data_Position, callback)
    rospy.spin()


listener()
