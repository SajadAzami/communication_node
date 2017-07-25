"""Propagation Models.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           Saman Golestannejad
# License:  BSD 3 clause

This package implements propagation model functions to compute signal strength(dBm).

Provides "Propagation Models" below
    1. One-Slope Model,
        [1] Rappaport, T.S., 1996. Wireless communications: principles and practice (Vol. 2). New Jersey: prentice hall PTR.
        [2] Zvanovec, Stanislav, Pavel Pechac, and Martin Klepal. "Wireless LAN networks design: site survey or propagation modeling?." Radioengineering 12.4 (2003): 42-49.
    2. Multi-Wall Model
    3. Rayleigh Fading Model
    4. Floor Attenuation

Using different models
----------
Setting the parameters before running the node in param.yaml
each message has parameter 'prop_model_type' that can be set.

Relations
----------
subscribes from /change_model_topic

"""

import math
import random
import sys
import threading
import time

import numpy as np

import actionlib
import roslib
import rospy
import smach
import smach_ros
import tf
import turtlesim
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from smach_ros import ServiceState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from tf import TransformListener


def _one_slope_model_checker(l0, decay_factor, distance, mode='yes_or_no'):
    # TODO handle different modes, need for a threshold
    return l0 + 10 * decay_factor * np.log10(distance)


def one_slope_model_checker(distance,
                            decay_factor=4.0,
                            l0=33.3,
                            mode='yes_or_no'):
    """Compute the signal strength using 1SM method.
        l0 and decay_factor are empirical parameters for a given environment.
        Tab.1 in [2] presents a few values taken from various references.
        Here are some recommended values:
         ------------------------------
        |  l0  |  n  |    comment      |
         ------------------------------
        | 33.3 | 4.0 |    office       |
        | 37.5 | 2.0 |    open space   |
        | 39.2 | 1.4 |    corridor     |
        | 38.5 | 3.5 | office building |
        | 38.0 | 2.0 |    passage      |
        | 38.0 | 1.3 |    corridor     |
        | 40.2 | 4.2 | office building |
        | 40.2 | 1.2 |    corridor     |
        | 40.0 | 3.5 | office building |
        | 46.4 | 3.5 | office building |
         ------------------------------
    :parameter
    mode : result mode, string, default 'yes_or_no'
        - 'yes_or_no'
        - 'loss_percentage'

    decay_factor : power decay factor or path loss exponent

    distance : distance between robots

    l0 : reference loss value for the distance of 1m


    :return:
    result : boolean or integer(depending on the mode), indicating signal strength
    """

    return _one_slope_model_checker(l0, decay_factor, distance, mode)
