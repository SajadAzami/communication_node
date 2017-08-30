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
from std_msgs.msg import String

from environment_information import get_object_distance
from propagation_models import one_slope_model_checker


def callback(data):
    distance = get_object_distance("pioneer3at", "Dumpster")
    result = one_slope_model_checker(distance=distance)
    print result
    print "Done!"
    # data.data


def listener():
    rospy.init_node('communication_node_message_handler', anonymous=True)
    rospy.Subscriber("/message_server", String, callback)
    rospy.spin()


listener()
