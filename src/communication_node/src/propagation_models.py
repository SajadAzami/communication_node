"""Propagation Models.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           Saman Golestannejad
# License:  BSD 3 clause

Provides "Propagation Models" below
    1. One-Slope Model
    2. Multi-Wall Model
    3. Rayleigh Fading Model
    4. Floor Attenuation

Using different models
----------
Setting the parameters before running the node in param.yaml
or publishing a message to /change_model_topic

Relations
----------
subscribes from /change_model_topic

"""

import math
import random
import sys
import threading
import time

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
