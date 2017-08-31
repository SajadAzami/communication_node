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
from std_msgs.msg import String  # TODO to be replaced with WSS message

from environment_information import get_object_distance
from propagation_models import one_slope_model_checker


def callback(data):
    # data.data
    prop_model = data.prop_model
    if prop_model == '1sm':
        distance = get_object_distance("pioneer3at", "Dumpster")
        # TODO distance = get_object_distance(data.sender, data.receiver)
        result = one_slope_model_checker(distance=distance)
        if result:
            # TODO send the message
            print "communication is not possible"
            pass
        else:
            # TODO, ignore the message, send feedback
            pass


def listener():
    rospy.init_node('communication_node_message_handler', anonymous=True)
    rospy.Subscriber("/message_server", String, callback)
    rospy.spin()


# TODO Run the registration_server
listener()
