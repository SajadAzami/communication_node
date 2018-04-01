#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
# License:  BSD 3 clause

"""

import os
import signal
import sys
from time import gmtime,strftime
import rospy;
import tf;
from geometry_msgs.msg import *;
from nav_msgs.msg import *;
from std_msgs.msg import *;
import threading;

info_list=[];
map_logger=None;
path_logger=None;
base_time=0;

def on_exit(*args):
    global information_logger
    print ( "\n EXITING MESSAGE HANDLER")
    if map_logger!=None :
         map_logger.write("\n The Test has finished on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         map_logger.write("\n ======================== \n ======================== \n \n \n")
         map_logger.close()
    if path_logger!=None :
         path_logger.write("\n The Test has finished on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         path_logger.write("\n ======================== \n ======================== \n \n \n")
         path_logger.close()
    sys.exit(0)

class map_path:
    def __init__(self,robot_name_space,t_lock_map):
        subscribing_topic="/global_map";
        self.map_sub=rospy.Subscriber(subscribing_topic, OccupancyGrid, self.map_callback);
        self.map_data=None;
        self.map_percent=0.0;
        self.t_lock_map=t_lock_map;
        self.robot=robot_name_space;

    def map_callback(self,input_map_data):
        self.t_lock_map.acquire();
        self.map_data=input_map_data.data;
        self.t_lock_map.release();

    def map_percent_calculator(self):
        self.t_lock_map.acquire();
        explored_percent=0;
        if self.map_data==None:
            self.t_lock_map.release();
            return 0.0;
        for i in self.map_data.data:
            if i>=0 :
                explored_percent+=1;
        self.map_percent=explored_percent/10000.0;
        self.t_lock_map.release();
        return self.map_percent;




def main():
    global info_list;
    global path_logger;
    global map_logger;
    global base_time;
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)
    rospy.init_node('info_node', anonymous=True)
    debuger_mode=rospy.get_param("debuger_mode",default=False)
    if debuger_mode==True :
         log_file=rospy.get_param("log_file",default="results")
         log_folder=rospy.get_param("log_folder",default="map")
         if not os.path.exists("/home/sosvr/communication_node_project/communication_node/results_pack/"+log_folder):
             os.makedirs("/home/sosvr/communication_node_project/communication_node/results_pack/"+log_folder)
         map_logger =  open("/home/sosvr/communication_node_project/communication_node/results_pack/"+log_folder+"/"+log_file+"_map.log", "w")
         map_logger.write("\n \n \n ###################### \n ###################### \n")
         map_logger.write("\n This is the result of test on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")

    for i in ["global_map"]:
        info_list.append(map_path(i,threading.Lock()));

    rate = rospy.Rate(0.05)
    base_time = 0;
    while (not rospy.is_shutdown()) and base_time<1500:
        if(base_time%100==0):print(str(base_time/100),"seconds");
        if debuger_mode==True:
            for i in info_list:
                i.map_percent_calculator();
                map_logger.write("\n "+i.robot+","+str(i.map_percent)+" ,"+str(int(base_time)));
                print(i.map_percent);
        base_time+=20;
        rate.sleep();
    print("finished");
    rospy.spin()

main();
