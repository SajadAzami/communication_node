#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
# License:  BSD 3 clause

"""


import rospy;
import tf;
from geometry_msgs.msg import *;
from nav_msgs.msg import *;
from std_msgs.msg import *;
import threading;

t_lock=threading.Lock();
explored_percent=0;
md=999999999;
def call_back(map_data):
    global explored_percent;
    global t_lock;
    global md;
    t_lock.acquire();
    explored_percent=0;
    md=map_data.info.height*map_data.info.width;
    for i in map_data.data:
        if i>=0 :
            explored_percent+=1;
    t_lock.release();



def talker():
    global explored_percent;
    global t_lock;
    global md;
    rospy.init_node('map_percent', anonymous=True)
    global_map=rospy.get_param("global_map", default="none");
    robot_name_space = rospy.get_param("robot_name", default="sos1");
    subscribing_topic="";
    publishing_topic="";
    if global_map=="none":
        subscribing_topic="/"+robot_name_space+"/map";
        publishing_topic="/"+robot_name_space+"/percent_of_map";
    else:
        subscribing_topic="/global_map";
        publishing_topic="/"+"global_map"+"/percent_of_map";

    rospy.Subscriber(subscribing_topic, OccupancyGrid, call_back)
    pub = rospy.Publisher(publishing_topic, Float64, queue_size=10)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        temp_data=Float64();
        t_lock.acquire();
        temp_data.data=explored_percent/2500.0;
        t_lock.release();
        pub.publish(temp_data)
        rate.sleep()
    rospy.spin()

talker();
