#!/usr/bin/env python


"""
Testing the communication_node
"""
import rospy
from messenger_api import send_map_message
from messenger_api import register
from communication_node.msg import Data_Map

robot_namespace = 'robot1'
rospy.init_node(robot_namespace + '_registration_send_msg_test_node', anonymous=True)
register(robot_namespace)  # TODO register for test
msg = Data_Map()
msg.source = "robot1"
msg.destination = "base_station"
msg.command = "update map - test 1"
# msg.data = 
print(msg)
send_map_message(msg, msg.destination)

