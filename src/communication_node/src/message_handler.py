# coding=utf-8
"""Message Handler.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           Saman Golestannejad
# License:  BSD 3 clause

Plays the server role in communication_node

Relations
----------
subscribes from /message_server topic,
publishes on corresponding nodes /ns/message_status topic

"""

import rospy
from environment_information import get_robot_distances


# TODO design subscriber and publisher to message_handler topics
rospy.loginfo(get_robot_distances('sos1', 'sos2'))
