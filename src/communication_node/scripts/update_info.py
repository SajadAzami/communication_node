#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
#           Sajjad Azami <sajjadaazami@gmail.com>
# License:  BSD 3 clause

Utils package for environment information extraction.
"""
import os
import signal
import sys
from time import gmtime,strftime
import rospy
from communication_node.msg import *
from nav_msgs.msg import *
from environment_information import get_object_distance ,get_n_walls_between
from propagation_models import one_slope_model_checker,multi_wall_model_checker
propagation_parameters={ "decay_factor":2.0,"l0":40.0,"threshold":93}
connection_list=[];
direct_connection=[];
debuger_mode=False;
information_logger=None;
robots_list=[];
prop_model="1sm";


def on_exit(*args):
    global information_logger
    print ( "\n EXITING MESSAGE HANDLER")
    if information_logger!=None :
         information_logger.write("\n The Test has finished on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         information_logger.write("\n ======================== \n ======================== \n \n \n")
         information_logger.close()
    sys.exit(0)




def line_of_sight():
    global prop_model;
    global connection_list;
    global robots_list;
    global direct_connection;
    for i in range(0,len(connection_list)):
        for j in range(0,len(robots_list)):
            if (connection_list[i][0]==robots_list[j]):continue;
            if prop_model=="1sm":
                distance = get_object_distance(data.destination, data.source);
                if(distance==-1 or distance==None):
                    connection_list[i][1+j]=0;
                    direct_connection[i][1+j]=0;
                    continue;
                result = one_slope_model_checker(distance=distance,decay_factor=propagation_parameters["decay_factor"],l0=propagation_parameters["l0"],threshold=propagation_parameters["threshold"])
                if(result[0]==True):
                    connection_list[i][1+j]=1;
                    direct_connection[i][1+j]=1;
                else:
                     connection_list[i][1+j]=0;
                     direct_connection[i][1+j]=0;
            elif prop_model=="mwm":
                distance_and_walls = get_object_distance(data.destination, data.source);
                if(distance_and_walls==-1 or distance_and_walls==None):
                    connection_list[i][1+j]=0;
                    direct_connection[i][1+j]=0;
                    continue;
                result = multi_wall_model_checker(distance=distance_and_walls[0],number_of_walls=distance_and_walls[1],decay_factor=propagation_parameters["decay_factor"],l0=propagation_parameters["l0"],threshold=propagation_parameters["threshold"])
                if(result[0]==True):
                    connection_list[i][1+j]=1;
                    direct_connection[i][1+j]=1;
                else:
                    connection_list[i][1+j]=0;
                    direct_connection[i][1+j]=0;

def multihub():
    global connection_list;
    global robots_list;
    for i in range (0,len(connection_list)):
        for j in range (0,len(robots_list)):
            if (connection_list[i][0]==robots_list[j]):continue;
            if (connection_list[i][1+j]==1 or connection_list[j][i+1]==1):
                for k in range(0,len(connection_list)):
                    if (connection_list[k][i+1]==1 or connection_list[i][k+1]==1):
                        connection_list[k][j+1]=1;
                        connection_list[j][k+1]=1;
def main():
    global connection_list;
    global robots_list;
    global direct_connection;
    global debuger_mode;
    global information_logger;
    global prop_model;
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)
    rospy.init_node("update_info", anonymous=True)
    robots_list=rospy.get_param("/robots_list")
    debuger_mode=rospy.get_param("debuger_mode",default=False)
    if debuger_mode==True :
         log_file=rospy.get_param("log_file",default="results")
         if not os.path.exists("/home/sosvr/communication_node_project/communication_node/test_results/"+log_file):
             os.makedirs("/home/sosvr/communication_node_project/communication_node/test_results/"+log_file)
         information_logger =  open("/home/user/project_franchesco/communication_node/test_results/"+log_file+"/"+log_file+"2.log", "a")
         information_logger.write("\n \n \n ###################### \n ###################### \n")
         information_logger.write("\n This is the result of test on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         information_logger.write("propagation model =>"+prop_model+" --- propagation parameters===>>>"+" [decay_factor="+str(propagation_parameters["decay_factor"])+" ]--[ l0="+str(propagation_parameters["l0"])+"]--[ threshold="+str(propagation_parameters["threshold"])+ "]\n")
         information_logger.write("robotname ")
         for i in robots_list:
             information_logger.write("---concetion to "+ i);
         information_logger.write("\n ");

    for i in robots_list:
       temp_list=[i];
       for j in robots_list:
           temp_list.append(0);
       connection_list.append(list(temp_list));
       direct_connection.append(list(temp_list));
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        line_of_sight();
        multihub();
        for i in range(0,connection_list):
            rospy.set_param("/connection_list_"+connection_list[i][0],connection_list[i]);
        print("update done");
        if debuger_mode==True:
            information_logger.write("information for multihub \n ");
            for j in connection_list:
                for k in j :
                    information_logger.write("--"+k+"--");
                information_logger.write("\n");
            information_logger.write("information for direct connection \n ");
            for j in direct_connection:
                for k in j :
                    information_logger.write("--"+k+"--");
                information_logger.write("\n");

    rospy.spin();
