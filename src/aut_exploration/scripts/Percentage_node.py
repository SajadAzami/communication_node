#!/usr/bin/env python


import roslib;
import rospy;
import tf;
import math;
from aut_exploration.srv import *;
from geometry_msgs.msg import *;
from nav_msgs.msg import *;
from std_msgs.msg import Header;

map_data=None;

def percentage_calculator(x_max,y_max,x_min,y_min):
    global map_data;
    local_variable = map_data;
    min_x=local_variable.info.origin.position.x;
    min_y=local_variable.info.origin.position.y;
    max_x=min_x + int(local_variable.info.resolution * local_variable.info.width);
    max_y=min_y + int(local_variable.info.resolution * local_variable.info.height);
    if (x_max <= max_x -1):
        max_x=x_max;
    if (x_min >= min_x +1):
        min_x=x_min;
    if (y_max <= max_y -1):
        max_y=y_max;
    if (y_min >= min_y +1):
        min_y=y_min;



    min_y=int((min_y-local_variable.info.origin.position.y)/local_variable.info.resolution);
    min_x=int((min_x-local_variable.info.origin.position.x)/local_variable.info.resolution);

    max_y=int((max_y-local_variable.info.origin.position.y)/local_variable.info.resolution);
    max_x=int((max_x-local_variable.info.origin.position.x)/local_variable.info.resolution);

    sum=0;
    for i in range(min_y,max_y):
        for j in range(min_x,max_x):
            if local_variable.data[(i*local_variable.info.width)+j]>=0:
                sum+=1;
            else:
                sum+=-1;


    return (sum+(max_y-min_y)*(max_x-min_x))*100.0/(2.0*(max_y-min_y)*(max_x-min_x));






def service_Handler(request):
    x_max,y_max,x_min,y_min = request.points[0].x,request.points[0].y,request.points[0].x,request.points[0].y;
    for i in range (0,4):
        if request.points[i].x >= x_max:
            x_max=request.points[i].x ;
        elif request.points[i].x <= x_min:
            x_min=request.points[i].x ;
        if request.points[i].y >= y_max:
            y_max=request.points[i].y ;
        elif request.points[i].y <= y_min:
            y_min=request.points[i].y ;
    response=Percentage_serviceResponse();
    response.percentage=percentage_calculator(x_max,y_max,x_min,y_min);
    return response;



def set_Map(map_merger_data):
    global map_data;
    map_data=map_merger_data;
    # print (str(map_data.info.resolution));


if (__name__ == "__main__"):
    rospy.init_node("percentage_python_script");
    map_merger_topic = rospy.get_param("map_merger_topic", default="/global_map");
    map_subscriber=rospy.Subscriber(map_merger_topic, OccupancyGrid, set_Map);
    percentage_server = rospy.Service("percentage_server", Percentage_service, service_Handler);
    # a=OccupancyGrid();
    # a.info.origin.position.x=0;
    # a.info.origin.position.y=0;
    # a.info.origin.position.z=0;
    # a.info.origin.orientation.x=0;
    # a.info.origin.orientation.y=0;
    # a.info.origin.orientation.z=0;
    # a.info.origin.orientation.w=1;
    # a.info.width=200;
    # a.info.height=100;
    # a.info.resolution=0.2;
    # a.info.map_load_time=rospy.Time.now();
    # a.header.frame_id="/map";
    # a.header.seq=1;
    # a.header.stamp=a.info.map_load_time;
    #
    #
    # b=[];
    # for i in range(0,100):
    #     for j in range(0,200):
    #         if  i==1 :
    #             b.insert((i*200)+j,1);
    #         else :
    #             b.insert((i*200)+j,-1);
    # a.data=b;
    # public=rospy.Publisher("himap",OccupancyGrid,queue_size=100);
    # r= rospy.Rate(1);
    # while not rospy.is_shutdown():
    #     print ("hi mother fucker \n");
    #     a.info.map_load_time=rospy.Time.now();
    #     a.header.stamp=a.info.map_load_time;
    #     public.publish(a);
    #     r.sleep();


    rospy.spin();
