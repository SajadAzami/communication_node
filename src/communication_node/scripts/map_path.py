#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
# License:  BSD 3 clause

"""

#3b3c32e9


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
    def __init__(self,robot_name_space,t_lock_map,t_lock_path):
        self.map_sub=rospy.Subscriber("/"+robot_name_space+"/percent_of_map", Float64, self.map_callback);
        self.path_sub=None;
        if(robot_name_space!="global_map"):
            self.path_sub=rospy.Subscriber("/"+robot_name_space+"/path_lenght", Float64, self.path_callback);
        self.path_lenght=0;
        self.map_percent=0;
        self.t_lock_map=t_lock_map;
        self.t_lock_path=t_lock_path;
        self.robot=robot_name_space;

    def map_callback(self,map_data):
        self.t_lock_map.acquire();
        self.map_percent=map_data.data;
        self.t_lock_map.release();

    def path_callback(self,path_data):
        self.t_lock_path.acquire();
        self.path_lenght=path_data.data;
        self.t_lock_path.release();



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
         if not os.path.exists("/home/sosvr/communication_node_project/communication_node/test_results/"+log_file):
             os.makedirs("/home/sosvr/communication_node_project/communication_node/test_results/"+log_file)
         map_logger =  open("/home/sosvr/communication_node_project/communication_node/test_results/"+log_file+"/"+log_file+"_map.log", "w")
         map_logger.write("\n \n \n ###################### \n ###################### \n")
         map_logger.write("\n This is the result of test on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         path_logger =  open("/home/sosvr/communication_node_project/communication_node/test_results/"+log_file+"/"+log_file+"_path.log", "w")
         path_logger.write("\n \n \n ###################### \n ###################### \n")
         path_logger.write("\n This is the result of test on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")


    for i in ["robot0","robot1","robot2","robot3","global_map"]:
        info_list.append(map_path(i,threading.Lock(),threading.Lock()));

    rate = rospy.Rate(0.5)
    base_time = 0;
    while (not rospy.is_shutdown()) and base_time<1200:
        if(base_time%100==0):print("100 seconds");
        if debuger_mode==True:
            for i in info_list:
                i.t_lock_map.acquire();
                map_logger.write("\n "+i.robot+","+str(i.map_percent)+" ,"+str(int(base_time)))
                i.t_lock_map.release();
                i.t_lock_path.acquire();
                if(i.robot!="global_map"):
                    path_logger.write("\n "+i.robot+" ,"+str(i.path_lenght)+" ,"+str(int(base_time)))
                i.t_lock_path.release();
        base_time+=2;
        rate.sleep()
    rospy.spin()

main();
