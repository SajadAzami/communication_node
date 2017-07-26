# coding=utf-8
"""Environment Information.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           Saman Golestannejad
# License:  BSD 3 clause

Utils package for environment information extraction.
"""

import tf
import rospy


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


def get_robot_distances(ns1, ns2):
    """Returns robot1 and robot2 distances
    :parameter
    ns1 : string, robot1 namespace

    ns2 : string, robot2 namespace

    :returns
    distance : float, distance between 2 robot

    Relations
    ----------
    """
    rate = rospy.Rate(10.0)
    transform_1 = get_current_position(ns1)
    transform_2 = get_current_position(ns2)
    return abs(transform_2 - transform_1)
