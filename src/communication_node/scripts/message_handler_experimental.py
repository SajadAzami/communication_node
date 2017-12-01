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
import os
import signal
import sys
from time import gmtime,strftime
import rospy
from communication_node.msg import *
from nav_msgs.msg import *
from environment_information import get_object_distance
from propagation_models import one_slope_model_checker

debuger_mode=False
information_logger=None
rate=None
message_handlers_list=[];

def on_exit(*args):
    global information_logger
    print ( "\n EXITING MESSAGE HANDLER")
    if information_logger!=None :
         information_logger.write("\n ###################### \n ###################### \n")
         information_logger.close()
    sys.exit(0)

class message_handle:
    def _init_(self,subs_topic="",pub_topic="",data_type=None,tag="",alt_type=None):
        self.subs_topic=subs_topic;
        self.pub_topic="";
        self.data_type=data_type;
        self.alt_type=alt_type;
        self.tag=tag;
        self.subscriber=rospy.Subscriber(self.subs_topic+self.tag,self.data_type, self.callback_function,queue_size=50);
        self.message_publisher=None;
    def callback_function(self,data):
            global information_logger
            # TODO handle for different message types
            # TODO prop_model = data.prop_model
            print("new "+tag+" received")
            prop_model = '1sm'
            if prop_model == '1sm':
                # distance = get_object_distance("pioneer3at", "Dumpster")
                distance = get_object_distance(data.destination, data.source)
                print (type(distance),tag)
                while (distance==None):
                    distance = get_object_distance(data.destination, data.source)
                result = one_slope_model_checker(distance=distance)
                if result:
                    if (self.alt_type!=None):
                       self.message_publisher = rospy.Publisher(data.source + self.pub_topic, self.alt_type, queue_size=10)
                   else:
                       self.message_publisher = rospy.Publisher(data.source + self.pub_topic, self.data_type, queue_size=10)
                    i=0
                    while not ( rospy.is_shutdown() or i>1):
                        self.message_publisher.publish(data.data)
                        i+=1
                        rate.sleep()
                    i=0
                    print "communication is possible"
                    if debuger_mode==True :
                       information_logger.write(self.tag+"".join(["-" for k in range(0,11-len(self.tag))]))
                       information_logger.write(data.source+"".join(["-" for k in range(0,11-len(data.source))]))
                       information_logger.write(data.destination+"".join(["-" for k in range(0,20-len(data.destination))]))
                       information_logger.write(str(distance)+"".join(["-" for k in range(0,18-len(str(distance)))]))
                       information_logger.write("message sent"+"\n")

                else:
                    # TODO, ignore the message, send feedback
                    if debuger_mode==True :
                      information_logger.write(self.tag+"".join(["-" for k in range(0,11-len(self.tag))]))
                      information_logger.write(data.source+"".join(["-" for k in range(0,11-len(data.source))]))
                      information_logger.write(data.destination+"".join(["-" for k in range(0,20-len(data.destination))]))
                      information_logger.write(str(distance)+"".join(["-" for k in range(0,18-len(str(distance)))]))
                      information_logger.write("failed"+"\n")
                    print "communication is not possible"



def listener():
    global information_logger
    global rate
    global debuger_mode
    global message_handlers_list;
    rospy.init_node('communication_node_message_handler')
    rate=rospy.Rate(10)
    debuger_mode=rospy.get_param("debuger_mode",default=False)
    if debuger_mode==True :
         log_file=rospy.get_param("log_file",default="results")
         if not os.path.exists("/home/user/project_franchesco/communication_node/test_results/"+log_file):
             os.makedirs("/home/user/project_franchesco/communication_node/test_results/"+log_file)
         information_logger =  open("/home/user/project_franchesco/communication_node/test_results/"+log_file+"/"+log_file+".log", "a")
         information_logger.write("\n ###################### \n ###################### \n")
         information_logger.write("\n This is the result of test on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         information_logger.write("Type-------Source-----Destination---------Distance----------Outcome\n");
    message_list=[["/message_server_","/inbox_MtA",Data_MtA,"MtA"],["/message_server_","/inbox_AtM",Data_AtM,"AtM"],["/message_server_","/g_map",Data_Map,"map",alt_type=OccupancyGrid],["/message_server_","/inbox_Odom",Data_Odom,"Odom"]];
    for i in range (0,len(message_list)):
        message_handlers_list.append(message_handle(message_list[i][0],message_list[i][1],message_list[i][2],message_list[i][3]));
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)
    rospy.spin()


listener()
