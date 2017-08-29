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
from environment_information import get_n_walls_between
from environment_information import get_object_distance


# TODO design subscriber and publisher to message_handler topics
print(get_n_walls_between("drc_practice_blue_cylinder_26_clone_4","asphalt_plane_0"))
