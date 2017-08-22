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


def get_n_walls_between():
    """
    """
    # TODO Gohari
    pass


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
    # TODO Gohari

    rate = rospy.Rate(10.0)
    transform_1 = get_current_position(ns1)
    transform_2 = get_current_position(ns2)
    return abs(transform_2 - transform_1)
