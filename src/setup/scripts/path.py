#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
# License:  BSD 3 clause

"""
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
import math
poses_lenght=0;
path_lenght=0.0;

def call_back(data):
    global poses_lenght;
    global path_lenght;
    for i in range(poses_lenght,len(data.poses)-1):
        x=(data.poses[i].pose.position.x-data.poses[i+1].pose.position.x)*(data.poses[i].pose.position.x-data.poses[i+1].pose.position.x)
        y=(data.poses[i].pose.position.y-data.poses[i+1].pose.position.y)*(data.poses[i].pose.position.y-data.poses[i+1].pose.position.y)
        path_lenght+=float("{0:.2f}".format(math.sqrt(y+x)));
    poses_lenght=len(data.poses);


def talker():
    global poses_lenght;
    global path_lenght;
    rospy.init_node('path_calculator', anonymous=True)
    robot_name_space = rospy.get_param("robot_name", default="sos1");
    rospy.Subscriber("/"+robot_name_space+"/trajectory", Path, call_back)
    pub = rospy.Publisher("/"+robot_name_space+"/path_lenght", Float64, queue_size=10)
    rate = rospy.Rate(1.5) # 10hz
    while not rospy.is_shutdown():
        temp_data=Float64();
        temp_data.data=path_lenght
        pub.publish(temp_data)
        rate.sleep()
    rospy.spin()

talker();
