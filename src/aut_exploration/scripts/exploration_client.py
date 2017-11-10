#!/usr/bin/env python


# this file is a behavoir that we have created to to run a robot in full autonomous mode
# to do that we are using a stat machine created using the smach class of python
# using the smach is the main reasen we are using the python
# the robot will try to explore the it's global costmap by picking a 36 by 36 meter square and then it divides it into
# 9 smaller 18 by 18 meter blockes ...the robot it self is in the center block
# meaning the block with index of 4
# then the robot will calculate which one these blocks is explored less and the chossen block must be under 75 % explored
# after picking a cell the robot will try to visit the center and corners of the cell during this process the cell will be most probobly explored by more than 75 %
# so after that the robot will try to chose another cell and get explore it
# if by any chance all the 9 cells within the block are explored more than 75% then it will contact the master and try to get to somewhere that has no been explored yet
# when robot is moving if the robot detects a victim then the robot will enter the a state called victim detected and there
# inside that state it will wait for several seconds and then move
# however if before coming to this state robot will check to see if it has already seen the victim and then after that it will move on
import roslib;
import rospy;
import smach;
import smach_ros;
import actionlib;
import tf;
import math;
from visualization_msgs.msg import Marker;
from visualization_msgs.msg import MarkerArray;
from aut_exploration.msg import *;
from aut_exploration.srv import *;
from geometry_msgs.msg import *;
from nav_msgs.msg import *;
from actionlib_msgs.msg import *;
from move_base_msgs.msg import *;
from smach_ros import ServiceState;
from std_msgs.msg import *;
from tf import TransformListener;
from communication_node.msg import Data_Map



new_command= False;
command_from_master=None;
markers=[];
detection_time=-1;
victims=[];
mark_counter=0;
move_base_cancel_publisher=None;
GCostmap_data = None;
robot_name_space = None;
Odom_data = None;
current_victim_status=None;
current_goal_status = 0 ; # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
sac=None;
map_publisher=None;
client_subscriber=None;
odom_publisher=None;
client_publisher=None;
command_from_master=None;
exploration_boundry=None;
driving=False;
verbose=True;

def elemntry_move(x,y):
        global sac;
        goal = MoveBaseGoal();
        goal.target_pose.pose.position.x = float(x);
        goal.target_pose.pose.position.y = float(y);
        goal.target_pose.pose.orientation.w = 1.0;
        goal.target_pose.header.frame_id = robot_name_space + "/odom";
        goal.target_pose.header.stamp = rospy.Time.now();
        sac.send_goal(goal);
        sac.wait_for_result();


def victim_callback2(data):
    global mark_counter;
    global current_victim_status;
    global victims;
    global detection_time;
    if  rospy.get_time()-detection_time > 24 :
       x=data.x;
       y=data.y;
       if(verbose):print ("verbosing --"+robot_name_space+"-- possible victim detected at x={} and y={}".format(x,y));
       detection_time=rospy.get_time();
       conform=mark_location(x,y,mark_counter,data);
       if conform == "invalid":
           return;
       current_victim_status="victim_detected";
       mark_counter+=1;
       rospy.loginfo("victim detected at %f in the position %f--%f",detection_time,x,y);

def victim_callback(data):
    global mark_counter;
    global current_victim_status;
    global victims;
    global detection_time;
    if len(data.objects)>0 and rospy.get_time()-detection_time> 10 :
        if data.objects[0].label=="Human" :
            detection_time=rospy.get_time();
            x=Odom_data.pose.pose.position.x;
            y=Odom_data.pose.pose.position.y;
            rospy.loginfo("victim detected at %f in the position %f--%f",detection_time,x,y);

    #if data.
     #  victims.insert(len(victims),Odom_data.pose.pose.position);
      # current_victim_status="victim_detected";


def mark_location(x, y, mark_id,data):
    #when we call this method , it checks the  array of existing markeres with the new marker
    #and if the new marker is not within 4 meters of any of the existing markers
    #the new marker will be added to the arrey and it  will be shown on the rviz
    global markers;
    vic_is_alive=0 # kar dare inja bayad az data ke az node mostafa miad age vic zende bashe 1 vagarna 0 bezarim
    shape = Marker.CUBE;
    pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size=100);

    for i in markers:
        if in_range(x,y,i.pose.position.x,i.pose.position.y) < 4 :
            return "invalid";


    marker = Marker();
    marker.header.frame_id = "/map";
    marker.header.stamp = rospy.Time.now();

    marker.ns = "basic_shapes";
    marker.id = mark_id;

    marker.type = shape;

    marker.action = Marker.ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = rospy.Duration();
    markers.insert(len(markers),marker);
    victims.insert(len(victims),javadi(x,y,vic_is_alive));

    rospy.loginfo(marker);
    pub.publish(markers);
    return "valid";

    #rate.sleep()
    #while not rospy.is_shutdown():
    #    pub.publish(marker)
    #    rate.sleep()


def move_base_tools():
    global move_base_cancel_publisher;
    global sac;
    move_base_cancel_publisher=rospy.Publisher("move_base/cancel",GoalID,queue_size=10);
    sac = actionlib.SimpleActionClient("move_base", MoveBaseAction);
    print (robot_name_space);
    sac.wait_for_server();

def in_range(x, y, w, z):
    return math.sqrt((x-w) ** 2 + (y-z) ** 2);

def setOdom(rawodomdata):
    global Odom_data;
    global odom_publisher;
    Odom_data = rawodomdata;
    new_data=Data_Odom();
    new_data.odom=rawodomdata;
    new_data.source=robot_name_space;
    new_data.destination="exploration_master";
    odom_publisher.publish(new_data);

def setMap(costmap_data):
    global GCostmap_data;
    global map_publisher;
    GCostmap_data = costmap_data;
    new_data=Data_Map();
    new_data.source=robot_name_space;
    new_data.destination="exploration_master";
    new_data.data=costmap_data;
    map_publisher.publish(new_data);

def PxCalculator(mapdata):
    sum = 0;
    for i in range(0, len(mapdata)):
        if mapdata[i] >= 0:
            sum += 1;
        else:
            sum -= 1;
    final_sum = (sum + len(mapdata)) / (2 * len(mapdata)) * 100;
    return final_sum;

# subscriber method callback from /move_base/status
def callback_goal_status(data):
    global current_goal_status;
    if len(data.status_list)==0 :
        return;
    current_goal_status = data.status_list[len(data.status_list) - 2].status;

# subscriber method from /move_base/status
def start_listening():
    if(verbose):print ("verbosing--"+robot_name_space+"--reading data");
    rospy.Subscriber("move_base/status", GoalStatusArray, callback_goal_status);
    rospy.Subscriber("odom", Odometry, setOdom);
    rospy.Subscriber("map", OccupancyGrid, setMap);
    rospy.Subscriber("victim_detected",Point,victim_callback2);
    rospy.Subscriber("/drive_mode",String,drivemodecallback);

def drivemodecallback(data):
    global driving;
    driving = True;




def master_request_handler(req):
    global command_from_master;
    global Odom_data;
    global new_command;
    if req.destination!= robot_name_space : return;
    new_command=True;
    if(verbose):print ("verbosing--"+robot_name_space+"--received new command from master");
    command_from_master=req;


def start_services():
     global client_publisher;
     global client_subscriber;
     global odom_publisher;
     global map_publisher;
     client_subscriber = rospy.Subscriber("inbox_MtA", Data_MtA, master_request_handler);
     client_publisher=rospy.Publisher("/message_server_AtM", Data_AtM,queue_size=15);
     odom_publisher=rospy.Publisher("/message_server_Odom", Data_Odom,queue_size=15);
     map_publisher=rospy.Publisher("/message_server_map", Data_Map,queue_size=15);



def Block_chooser(mapdata):
    global current_goal_status;
    # listener_goal_status()

    length = int(math.sqrt(len(mapdata)));
    dividor3 = int(length / 3);
    blocks = [];
    a = 0;
    for i in range(0, length, dividor3):
        for k in range(0, length, dividor3):
            blocks.insert(a,[]) ;
            for j in range(0, dividor3):
                blocks[a].extend(mapdata[i * length + j * length + k:i * length + j * length + k + dividor3 - 1]);
    a += 1;
    minPX = PxCalculator(blocks[0]);
    minIndex = 0;
    for w in range(1, 9):
        if PxCalculator(blocks[w]) < minPX:
            minPX = PxCalculator(blocks[w]);
            minIndex = w;

    if minPX > 75:
        return [-1, blocks[minIndex]];
    else:
        return [minIndex, blocks[minIndex]];

# define some auxiliary classes
##############################
##############################

class MyPoint():
    def __init__(self, xVector, yVector,initial_result=None):
        self.type = "Point";
        self.result = initial_result;
        self.x = xVector;
        self.y = yVector;

class Block():
    def __init__(self, center, tl, tr, br, bl, matrix=[]):
        self.type = "Block";
        self.center = center;
        self.topLeft = tl;
        self.topRight = tr;
        self.bottomRight = br;
        self.bottomLeft = bl;
        self.matrix = matrix;

class taher():
       def __init__(self):
           self.origin = None;
           self.data=None;

class javadi():
    def __init__(self,vic_x,vic_y,is_alive):
        self.is_alive=is_alive;
        self.vic_x=vic_x;
        self.vic_y=vic_y;


# define state Chose_block
##############################3
##############################

class ChoseBlock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["block_choosed", "no_block_found"], input_keys=["CB_input"],
                             output_keys=["CB_output"]);
        self.counter = 0;
        self.block = None;
        self.center = None;
        self.topLeft = None;
        self.topRight = None;
        self.bottomRight = None;
        self.bottomLeft = None;

    def Calculations(self):
        global GCostmap_data;
        global Odom_data;
        map_res=GCostmap_data.info.resolution;
        local_variable =GCostmap_data;
        rospy.loginfo("ChoseBlock Calculations");

        lenght = local_variable.info.width ;
        lenght2 = local_variable.info.height ;
        x = (Odom_data.pose.pose.position.x - local_variable.info.origin.position.x) /map_res;
        y = (Odom_data.pose.pose.position.y - local_variable.info.origin.position.y) /map_res;
        #we get the x and y of robot reletive to the starting x and y of map
        # sincd  pose and origin positions are in meters we have to divide it by map_res since we have n cells per a meter (resulotion of cost map 1/n)
        if x < (27.5/map_res):
            x_gc = 0;
        elif x > lenght - (27.5/map_res):
            x_gc = int(lenght - (27.5/map_res));
        else:
            x_gc = int(x - (27/map_res));
        if y < (27.5/map_res):
            y_gc = 0;
        elif y > lenght2 - (27.5/map_res):
            y_gc = int(lenght2 - (27.5/map_res));
        else:
            y_gc = int(y - (27/map_res));
        #here we calculate the the x and the y of the starting cell  of the nine blocks as a whole

        Cell = [];
        for i in range(0, int(18/map_res)):
            #here we pick only the cells from the array of costmap that they fall inside the nine blocks
            # and because the array of costmap is in 1 deminsion i cant say data[i][j] instead i have to as follows
            Cell.extend(local_variable.data[(y_gc + i) * lenght:(1 + y_gc + i) * lenght]);
            if (i == int(9/map_res) and verbose):
                # it is for debugging
                print ("verbosing--"+robot_name_space+"--oh yeh");
                print ((y_gc + i) * lenght);
                print (len(local_variable.data));
        if len(Cell)<3 :
            #this if is for debugging and if the array named Cell has a lenght of smaller than 3 it tells us
            print ("oh yeh");
            print (local_variable.info.width);
            print (local_variable.info.height);
            print (y_gc);
        resultList = Block_chooser(Cell);
        block_index=resultList[0];
        if block_index == -1:
            return "contact the master";
        else:
            #each block is 18 meter wide and long and x_gc and y_gc are the number of cells to the bottomLeft corner
            #of block so we have to multiply it by the map_res because 1 meter means n cells with a resulotion of 1/n
            cx = x_gc *map_res + (block_index % 3) * 18.0 + 9.0;
            cy = y_gc *map_res + (int(block_index / 3)) * 18.0 + 9.0;
            self.center = MyPoint(cx, cy);
            self.topLeft = MyPoint(cx - 7, cy + 7);
            self.topRight = MyPoint(cx + 7, cy + 7);
            self.bottomRight = MyPoint(cx + 7, cy - 7);
            self.bottomLeft = MyPoint(cx - 7, cy - 7);
            self.block = Block(self.center, self.topLeft, self.topRight, self.bottomRight, self.bottomLeft,
                               resultList[1]);

    def execute(self, userdata):
        rospy.loginfo("Executing state Chose_block");
        a = self.Calculations();
        if a == "contact the master":
            b=taher();
            b.origin="explore";
            b.data=MyPoint(Odom_data.x,Odom_data.y,"request_goal");
            userdata.CB_output = b;
            return "no_block_found";
        else:
            userdata.CB_output = self.block;
            return "block_choosed";


# define state Explore_Center
##############################3
##############################
class ExploreBlock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["chose_another_block","gofor_task", "point_reached","victim detected"],
                             input_keys=["EB_input"], output_keys=["EB_output"]);
        self.block = None;
        self.status = None;
        self.points = None;
        self.count = 0;
        self.pointNumber=0;
        self.retry=False;


    def move_to_goal(self, pos_x, pos_y, pos_z=0, ornt_w=1, ornt_x=0, ornt_y=0, ornt_z=1):
        rospy.loginfo("sending goal inorder to explore a block");
        # Simple Action Client
        global current_goal_status;
        global current_victim_status;
        global sac;
        global new_command;

        # create goal
        goal = MoveBaseGoal();

        # set goal
        goal.target_pose.pose.position.x = pos_x;
        goal.target_pose.pose.position.y = pos_y;
        goal.target_pose.pose.orientation.w = ornt_w;
        goal.target_pose.pose.orientation.z = ornt_z;
        goal.target_pose.header.frame_id = robot_name_space + "/odom";
        goal.target_pose.header.stamp = rospy.Time.now();

        # start listener

        # send goal
        sac.send_goal(goal);
        odom_temp = Odom_data;
        rate = rospy.Rate(3);  # 3hz
        i = 0;
        while not rospy.is_shutdown():
            if new_command==True:
                return;
            i+=1;
            if odom_temp.pose.pose.position.x == Odom_data.pose.pose.position.x and odom_temp.pose.pose.position.y == Odom_data.pose.pose.position.y:
                i += 3;
            elif in_range(odom_temp.pose.pose.position.x ,odom_temp.pose.pose.position.y ,Odom_data.pose.pose.position.x,Odom_data.pose.pose.position.y)<0.02 :
                i += 3;
            else:
                odom_temp = Odom_data;
            if current_victim_status=="victim_detected" :
                self.status="victim detected";
                rospy.loginfo("victim detected during exploration of a block");
                return;
            elif current_goal_status==3 or current_goal_status==4 or current_goal_status==5 or current_goal_status==9:
               rospy.sleep(5);
               if current_goal_status==3 or current_goal_status==4 or current_goal_status==5 or current_goal_status==9:
                  current_goal_status=43;
                  px=PxCalculator(self.block.matrix);
                  if px >82 :
                      self.status="fully explored";
                      if(verbose):print ("verbosing--"+robot_name_space+"--fully explored move base finished");
                      return;
                  else:
                      self.status="not explored yet";
                      if(verbose):print ("verbosing--"+robot_name_space+"--not explored yet move base finished");
                      return ;
            elif i>450 :
                i=0;
                px = PxCalculator(self.block.matrix);
                if px > 75:
                    self.status = "fully explored";
                    if(verbose):print ("verbosing--"+robot_name_space+"--fully explored timed out");
                    return;
                else:
                    self.status = "not explored yet";
                    if(verbose):print ("verbosing--"+robot_name_space+"--not explored yet timed out");
                    return;

            rate.sleep();

            # finish
            # print (result)
            # goal_result = sac.get_result()

    def execute(self, userdata):
        global new_command;
        global command_from_master;
        self.status=None;
        rospy.loginfo(robot_name_space+"--Executing state ExploreBlock");
        if(verbose):print ("verbosing--"+robot_name_space+"--this was the number that you expected--"+str(self.count));
        a = userdata.EB_input;
        userdata.EB_output=a;
        if a.type == "Block" and self.count == 0:
            if(verbose):print ("verbosing--"+robot_name_space+"--going to the center of the block");
            self.block = a;
            self.points = [ (a.center),(a.topLeft),(a.topRight),(a.bottomRight),(a.bottomLeft)];
            self.move_to_goal(self.points[self.count].x,self.points[self.count].y);
            if self.status=="not explored yet" :
                self.count+=1;
        elif a.type == "Block" and self.count > 0:
            if(verbose):print ("verbosing--"+robot_name_space+"--going for the points");
            self.move_to_goal(self.points[self.count].x,self.points[self.count].y);
            if self.status=="not explored yet" :
                self.count+=1;
            if self.count > 4:
                if self.status == "not explored yet" and self.retry==False :
                    self.status = "retry";
                    self.retry=True
                elif self.status == "not explored yet" and self.retry==True:
                    self.retry=False;
                    self.status="exploration failed";
                self.count = 0;
        if new_command==True:
            new_command=False;
            if command_from_master.command=="explore":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"exploration_center");
                exploration_boundry=command_from_master.blocks;
                userdata.EB_output=b;
                return "gofor_task";
            elif command_from_master.command=="save_victim" or command_from_master.command=="standby":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
                userdata.EB_output=b;
                return "gofor_task";
            elif command_from_master.command=="detect_victim":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"detection_center");
                exploration_boundry=command_from_master.blocks;
                userdata.EB_output=b;
                return "gofor_task";

        if self.status == "fully explored":
            return "chose_another_block";
        elif self.status == "exploration failed":
            return "chose_another_block";
        elif self.status == "not explored yet" or self.status=="retry":
            return "point_reached";
        elif self.statu=="victim detected":
            return "victim detected";
        else:
            return "chose_another_block";


# define state Contact_master
##############################
##############################


class ContactMaster(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["move_toGoal", "keep_exploring","keep_detecting"], input_keys=["CM_input"], output_keys=["CM_output"]);
        self.goal = None;
        self.initial=True

    def send_the_service(self,service_toMaster):
        global client_publisher;
        client_publisher.publish(service_toMaster);

    def execute(self, userdata):
        global move_base_cancel_publisher;
        global client_publisher;
        global markers;
        global Dead_or_Alive;
        global Odom_data;
        global exploration_boundry;
        global new_command;
        global command_from_master;
        rospy.loginfo("Executing state Contact_master");
        a = "taher";
        service_toMaster=Data_AtM();
        rate = rospy.Rate(1);
        while self.initial:
            if(new_command==True):
                break;
            rate.sleep();


        if(self.initial):
            if(verbose):print ("verbosing--"+robot_name_space+"-- initial state of ours");
            self.initial=False;

        elif(type(userdata.CM_input) is str):
             if(verbose):print ("verbosing--"+robot_name_space+"--verbosing--"+userdata.CM_input);
             service_toMaster.source=robot_name_space;
             service_toMaster.destination="exploration_master";
             service_toMaster.agent_state="vicexp_finished";
             service_toMaster.agent_x=Odom_data.pose.pose.position.x;
             service_toMaster.agent_y=Odom_data.pose.pose.position.y;
             self.send_the_service(service_toMaster);

        elif(userdata.CM_input.origin=="gotogoal"):
            if(verbose):print("verbosing--"+robot_name_space+"--failing to move");
            service_toMaster.source=robot_name_space;
            service_toMaster.destination="exploration_master";
            service_toMaster.agent_state="failed_toMOVE";
            service_toMaster.agent_x=Odom_data.pose.pose.position.x;
            service_toMaster.agent_y=Odom_data.pose.pose.position.y;
            self.send_the_service(service_toMaster);

        elif(userdata.CM_input.origin=="victim_searching"):
            service_toMaster.source=robot_name_space;
            service_toMaster.destination="exploration_master";
            service_toMaster.agent_state="vicexp_finished";
            service_toMaster.agent_x=Odom_data.pose.pose.position.x;
            service_toMaster.agent_y=Odom_data.pose.pose.position.y;
            self.send_the_service(service_toMaster);

        elif(userdata.CM_input.origin=="victim"):
            service_toMaster.source=robot_name_space;
            service_toMaster.destination="exploration_master";
            service_toMaster.agent_state="victim_is_detected";
            service_toMaster.agent_x=Odom_data.pose.pose.position.x;
            service_toMaster.agent_y=Odom_data.pose.pose.position.y;
            self.send_the_service(service_toMaster);
        elif(userdata.CM_input.origin=="explore"):
            service_toMaster.source=robot_name_space;
            service_toMaster.destination="exploration_master";
            service_toMaster.agent_state="exp_finished";
            service_toMaster.agent_x=Odom_data.pose.pose.position.x;
            service_toMaster.agent_y=Odom_data.pose.pose.position.y;
            self.send_the_service(service_toMaster);

      # we keep waitng for command from master for 30 seconds
        rate = rospy.Rate(1);
        timer=0;
        while new_command==False:
            timer+=1;
            rate.sleep();
            # if 30 seconds pass we break
            if timer >20 :
                if(verbose):print ("verbosing--"+robot_name_space+"--no command was received");
                break;

        # if no new command has been received from master then we keep the value of  timer variable
        if new_command==True:
            new_command=False;
            timer=0;

        if timer>0:
                b=MyPoint(0.0,0.0,"exploration_center");
                timer=0;
                exploration_boundry=[Point(20.0,20.0,0.0),Point(-20.0,20.0,0.0),Point(-20.0,-20.0,0.0),Point(20.0,-20.0,0.0)];
                userdata.CM_output=b;
                return "move_toGoal";

        elif command_from_master.command == "victim_confirmed":
                move_base_cancel_publisher.publish(GoalID());
                rospy.sleep(0.2);
                move_base_cancel_publisher.publish(GoalID());
                rospy.sleep(0.2);
                move_base_cancel_publisher.publish(GoalID());
                rospy.sleep(0.2);
                move_base_cancel_publisher.publish(GoalID());
                rospy.sleep(0.2);
                if(verbose):print ("verbosing--"+robot_name_space+"--goal was canceled due to the detected vitcim");
                rospy.sleep(7);
                userdata.CM_output=userdata.CM_input.data;
                if userdata.CM_input.data.type=="Point":
                    return "move_toGoal";
                elif userdata.CM_input.data.type=="Block":
                    return "keep_exploring";
                elif userdata.CM_input.data.type=="victimsearching":
                    return "keep_detecting";

        elif command_from_master.command == "go_on":
                if(verbose):print ("verbosing--"+robot_name_space+"--victim was not new");
                userdata.CM_output=userdata.CM_input.data;
                if userdata.CM_input.data.type=="Point":
                    return "move_toGoal";
                elif userdata.CM_input.data.type=="Block":
                    return "keep_exploring";
                elif userdata.CM_input.data.type=="victimsearching":
                    return "keep_detecting";

        elif command_from_master.command=="explore":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"exploration_center");
                exploration_boundry=command_from_master.blocks;
                userdata.CM_output=b;
                return "move_toGoal";

        elif command_from_master.command=="standby":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
                userdata.CM_output=b;
                return "move_toGoal";

        elif command_from_master.command=="detect_victim":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"detection_center");
                exploration_boundry=command_from_master.blocks;
                userdata.CM_output=b;
                return "move_toGoal";

        elif command_from_master.command=="save_victim":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
                userdata.CM_output=b;
                return "move_toGoal";


##############################
##############################

class GotoGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["starting_Exploration","starting_VictimDetection","new task","Standing_by","failed_toReach","victim detected"], input_keys=["GG_input"], output_keys=["GG_output"]);
        self.goal = None;
        self.status=None;

    def move_to_goal(self,goal_x,goal_y, goal_z=0, ornt_w=1, ornt_x=0, ornt_y=0, ornt_z=1):
        rospy.loginfo("sending goal inorder to go to a very far goal");
        global sac;
        global new_command;
        global current_goal_status;
        global current_victim_status;
        # create goal
        self.goal=MyPoint(goal_x,goal_y);
        fargoal = MoveBaseGoal();
        # set goal
        fargoal.target_pose.pose.position.x = goal_x;
        fargoal.target_pose.pose.position.y = goal_y;
        fargoal.target_pose.pose.orientation.w = ornt_w;
        fargoal.target_pose.pose.orientation.z = ornt_z;
        fargoal.target_pose.header.frame_id = robot_name_space + "/odom";
        fargoal.target_pose.header.stamp = rospy.Time.now();
        # send goal
        sac.send_goal(fargoal);
        odom_temp = Odom_data;
        rate = rospy.Rate(3);  # 3hz
        i = 0;
        while not rospy.is_shutdown():
            if new_command==True:
                return;
            i+=1;
            if odom_temp.pose.pose.position.x == Odom_data.pose.pose.position.x and odom_temp.pose.pose.position.y == Odom_data.pose.pose.position.y:
                i += 2.5;
            elif in_range(odom_temp.pose.pose.position.x ,odom_temp.pose.pose.position.y ,Odom_data.pose.pose.position.x,Odom_data.pose.pose.position.y)<0.03 :
                i += 2.5;
            else:
                odom_temp = Odom_data;
            if current_victim_status=="victim_detected" :
                self.status="victim detected";
                rospy.loginfo("victim detected when moving to a very far goal");
                return;
            elif current_goal_status==3 or current_goal_status==4 or current_goal_status==5 or current_goal_status==9:
               if (current_goal_status==3 ):
                  self.status="goal_reached";
                  current_goal_status=43;
                  return;
               rospy.sleep(10);
               if (current_goal_status==4 or current_goal_status==5 or current_goal_status==9 ):
                  current_goal_status=43;
                  self.status="stucked";
                  return;

            elif i>450 :
                i=0;
                j=0;
                self.status="stucked";
                return;

            rate.sleep();


            # finish

            # print (result)
            # goal_result = sac.get_result()


    def execute(self, userdata):
        global new_command;
        global command_from_master;
        self.status=None;
        result_type=None;
        print (robot_name_space);
        rospy.loginfo("--Executing state GOTO_Goal");
        if (userdata.GG_input.type=="Point"):
            result_type=userdata.GG_input.result;
            self.move_to_goal(userdata.GG_input.x,userdata.GG_input.y);
        if new_command==True:
            new_command=False;
            if command_from_master.command=="explore":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"exploration_center");
                exploration_boundry=command_from_master.blocks;
                userdata.GG_output=b;
                return "new task";
            elif command_from_master.command=="save_victim" or command_from_master.command=="standby":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
                userdata.GG_output=b;
                return "new task";
            elif command_from_master.command=="detect_victim":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"detection_center");
                exploration_boundry=command_from_master.blocks;
                userdata.GG_output=b;
                return "new task";
        if self.status=="goal_reached":
            if userdata.GG_input.result=="standby":
                return "Standing_by";
            elif userdata.GG_input.result=="detection_center":
                userdata.GG_output="new boundry";
                return "starting_VictimDetection";
            elif userdata.GG_input.result=="exploration_center":
                return "starting_Exploration";
        elif self.status=="stucked":
            a=taher();
            a.origin="gotogoal";
            a.data=self.goal;
            userdata.GG_output=a;
            return "failed_toReach";
        elif self.status=="victim detected":
            userdata.GG_output=self.goal;
            return "victim detected" ;



# define the state VictimDetected
##############################
##############################


class VictimDetected(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["contacting_master"], input_keys=["VD_input"],output_keys=["VD_output"]);
        self.goal = None;

    # def waiting(self):
    #     global move_base_cancel_publisher;
    #     global client_publisher;
    #     global markers;
    #     global Dead_or_Alive;
    #     global Odom_data;
    #     move_base_cancel_publisher.publish(GoalID());
    #     rospy.sleep(0.2);
    #     move_base_cancel_publisher.publish(GoalID());
    #     rospy.sleep(0.2);
    #     move_base_cancel_publisher.publish(GoalID());
    #     rospy.sleep(0.2);
    #     move_base_cancel_publisher.publish(GoalID());
    #     rospy.sleep(0.2);
    #     print (robot_name_space);
    #     print ("goal was canceled due to the detected vitcim");
    #     a=rospy.get_time();
    #     service_toMaster=Agent_serviceGoal();
    #     service_toMaster.agent_state="victim_is_detected";
    #     service_toMaster.agent_x=Odom_data.pose.pose.position.x;
    #     service_toMaster.agent_y=Odom_data.pose.pose.position.y;
    #     service_toMaster.vic_state=1+Dead_or_Alive;
    #     service_toMaster.vic_x=markers[len(markers)-1].pose.position.x;
    #     service_toMaster.vic_y=markers[len(markers)-1].pose.position.y;
    #     response_fromMaster=None;
    #     try:
    #        response_fromMaster = client_publisher(service_toMaster);
    #     except rospy.ServiceException, e:
    #         print ("sending cordinates of the victim to master failed");
    #         response_fromMaster="failed";
    #     rospy.sleep(10.0-(rospy.get_time()-a));
    #     current_victim_status="no victim";
    #     return response_fromMaster;


    def execute(self, userdata):
        print (robot_name_space);
        rospy.loginfo("Executing state VictimDetected");
        current_victim_status="no victim";
        a=taher();
        a.data=userdata.VD_input;
        a.origin="victim";
        userdata.VD_output=a;
        return "contacting_master";

class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["gofor_task"],input_keys=["SB_input"],output_keys=["SB_output"]);



    def execute(self, userdata):
        global move_base_cancel_publisher;
        global new_command;
        new_command=False;
        print (robot_name_space);
        rospy.loginfo("Executing state Standby");
        move_base_cancel_publisher.publish(GoalID());
        rospy.sleep(0.2);
        move_base_cancel_publisher.publish(GoalID());
        rospy.sleep(0.2);
        move_base_cancel_publisher.publish(GoalID());
        rospy.sleep(0.2);
        move_base_cancel_publisher.publish(GoalID());
        rospy.sleep(0.2);
        rate = rospy.Rate(1.5);
        while not rospy.is_shutdown():
            if new_command==True:
                break;
            rate.sleep();
        if command_from_master.command=="explore":
            b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"exploration_center");
            exploration_boundry=command_from_master.blocks;
            userdata.CM_output=b;
            return "gofor_task";
        elif command_from_master.command=="save_victim":
            b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
            userdata.CM_output=b;
            return "gofor_task";
        elif command_from_master.command=="detect_victim":
            b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"detection_center");
            exploration_boundry=command_from_master.blocks;
            userdata.CM_output=b;
            return "gofor_task";
        elif command_from_master.command=="standby":
            b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
            userdata.CM_output=b;
            return "gofor_task";


class  VictimSearching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["search completed","gofor_task","victim detected","point reached"],input_keys=["VS_input"],output_keys=["VS_output"]);
        self.points=[];
        self.counter=0;
        self.result=None;
    def point_planner(self):
        if(verbose):print("verbosing--"+robot_name_space+"--staring point planner");
        global exploration_boundry;
        self.counter=0;
        self.points=[];
        x_max,y_max,x_min,y_min=exploration_boundry[0].x,exploration_boundry[0].y,exploration_boundry[0].x,exploration_boundry[0].y;
        for i in exploration_boundry:
            if i.x > x_max:
                x_max=i.x;
            elif i.x < x_min:
                x_min=i.x;
            if i.y > y_max:
                y_max=i.y;
            elif i.y < y_min:
                y_min=i.y;
        if(verbose):print ("verbosing--"+robot_name_space+str(y_min)+"--"+str(y_max)+"--"+str(x_min)+"--"+str(x_max))
        for iy in range(int(y_min),int(y_max),5):
                    for ix in range(int(x_min),int(x_max),5):
                        if(verbose):print("verboing--"+robot_name_space+"---"+str(ix)+"--"+str(iy));
                        self.points.append(Point(ix,iy,0.0));
        if(verbose):print("verbosing--"+robot_name_space+"--point planner finished");

    def go_for_point(self):
            print (robot_name_space);
            rospy.loginfo("just entered the go for points function");
            # Simple Action Client
            global current_goal_status;
            global current_victim_status;
            global sac;
            global new_command;
            global command_from_master;

            # create goal
            goal = MoveBaseGoal();

            # set goal
            goal.target_pose.pose.position.x = self.points[self.counter].x;
            goal.target_pose.pose.position.y = self.points[self.counter].y;
            goal.target_pose.pose.orientation.w = 1;
            goal.target_pose.pose.orientation.z = 1;
            goal.target_pose.header.frame_id = robot_name_space + "/odom";
            goal.target_pose.header.stamp = rospy.Time.now();
            if(verbose):print("verbosing--"+robot_name_space+"--about to send the goal to find victim");

            # start listener

            # send goal
            sac.send_goal(goal);
            if(verbose):print("verbosing--"+robot_name_space+"--just send the goal to find the victim");
            odom_temp = Odom_data;
            rate = rospy.Rate(3);  # 3hz
            i = 0;
            if(verbose):print("verbosing--"+robot_name_space+"--"+str(self.counter));
            while not rospy.is_shutdown():
                if new_command==True:
                    if(verbose):print("verbosing"+robot_name_space+"new command");
                    return;
                i+=1;
                if odom_temp.pose.pose.position.x == Odom_data.pose.pose.position.x and odom_temp.pose.pose.position.y == Odom_data.pose.pose.position.y:
                    i += 2;
                elif in_range(odom_temp.pose.pose.position.x ,odom_temp.pose.pose.position.y ,Odom_data.pose.pose.position.x,Odom_data.pose.pose.position.y)<0.02 :
                    i += 2;
                else:
                    odom_temp = Odom_data;
                if current_victim_status=="victim_detected" :
                    self.result="victim detected";
                    rospy.loginfo("victim detected during exploration of a block");
                    return;
                elif current_goal_status==3 or current_goal_status==5 or current_goal_status==4 or current_goal_status==9 :
                    self.result="point reached";
                    current_goal_status=43;
                    if(verbose):print("verbosing--"+robot_name_space+"--"+str(self.counter));
                    self.counter +=1;
                    return;
                elif i>390 :
                    i=0;
                    self.result="point reached";
                    self.counter +=1;
                    return;
                rate.sleep();

    def execute(self, userdata):
        global new_command;
        global command_from_master;
        print (robot_name_space);
        rospy.loginfo("Executing state VictimSearching");
        if(userdata.VS_input=="comingback"):
                if(verbose):print("verbosing--"+robot_name_space+"--i came back baby");
        elif(userdata.VS_input=="new boundry"):
            self.point_planner();
        if (self.counter >= len(self.points)):
            bf=taher();
            bf.origin="victim_searching";
            bf.data="kf";
            userdata.VS_output=bf;
            return "search completed";
        if(verbose):print("verbosing--"+robot_name_space+"--we are about to go for points");
        self.go_for_point();
        if(verbose):print("verbosing--"+robot_name_space+"--we just passed going for points");
        if new_command==True:
            new_command=False;
            if command_from_master.command=="explore":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"exploration_center");
                exploration_boundry=command_from_master.blocks;
                userdata.VS_output=b;
                return "gofor_task";
            elif command_from_master.command=="save_victim" or command_from_master.command=="standby":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
                userdata.VS_output=b;
                return "gofor_task";
            elif command_from_master.command=="detect_victim":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"detection_center");
                exploration_boundry=command_from_master.blocks;
                userdata.VS_output=b;
                return "gofor_task";
            elif command_from_master.command=="standby":
                b=MyPoint(command_from_master.goal_x,command_from_master.goal_y,"standby");
                userdata.VS_output=b;
                return "gofor_task";
        if self.result=="victim detected":
            a=MyPoint(1,1);
            a.type="victimsearching";
            userdata.VS_output=a;
            return "victim detected";
        elif self.result=="search completed":
            bf=taher();
            bf.origin="victim_searching";
            bf.data="kf";
            userdata.VS_output=bf;
            return "search completed";
        elif self.result=="point reached":
            userdata.VS_output="comingback";
            return "point reached";
        else :
            print ("this is not right");


# define the states of the nested state machine called recovery
##############################
##############################



class SimpleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["shuting_down", "comon"]);

    def Calculations(self):
        rospy.loginfo("calculationg");

    def execute(self, userdata):
        print (robot_name_space);
        rospy.loginfo("Executing state Request_different_goal");
        a = "taher";
        if a == "javadi":
            return "shuting_down";
        else:
            return "comon";


# our main function
##############################3
##############################





def main():
    # Create the top level SMACH state machine
    start_services();
    start_listening();
    move_base_tools();
    print ("reading finished \n ");

    rospy.sleep(2);

    sm_exploration = smach.StateMachine(outcomes=["hold", "shut_down"]);

    # Open the container
    with sm_exploration:

        smach.StateMachine.add("Contact_master", ContactMaster(),
                               transitions={"keep_exploring": "Explore_Block", "move_toGoal": "GOTO_Goal","keep_detecting":"Victim_Searching"},
                               remapping={"CM_input": "passing_data", "CM_output": "passing_data"});


        smach.StateMachine.add("Chose_block", ChoseBlock(),
                               transitions={"block_choosed": "Explore_Block", "no_block_found": "Contact_master"},
                               remapping={"CB_input": "passing_data", "CB_output": "passing_data"});


        smach.StateMachine.add("Explore_Block", ExploreBlock(),
                               transitions={"point_reached": "Explore_Block","chose_another_block": "Chose_block",
                                            "victim detected":"Victim_Detected","gofor_task":"GOTO_Goal"},
                               remapping={"EB_input": "passing_data", "EB_output": "passing_data"});


        smach.StateMachine.add("GOTO_Goal", GotoGoal(),
                               transitions={"starting_VictimDetection":"Victim_Searching","starting_Exploration": "Chose_block","new task":"GOTO_Goal",
                                            "Standing_by":"Stand_By","failed_toReach": "Contact_master","victim detected":"Victim_Detected"},
                               remapping={"GG_input":"passing_data", "GG_output": "passing_data"});

        smach.StateMachine.add("Victim_Detected", VictimDetected(),
                               transitions={"contacting_master": "Contact_master"},
                               remapping={"VD_input": "passing_data", "VD_output": "passing_data"});

        smach.StateMachine.add("Stand_By", Standby(),
                               transitions={"gofor_task":"GOTO_Goal"},
                               remapping={"SB_input": "passing_data", "SB_output": "passing_data"});

        smach.StateMachine.add("Victim_Searching", VictimSearching(),
                                transitions={"search completed":"Contact_master","victim detected":"Victim_Detected",
                                             "point reached":"Victim_Searching","gofor_task":"GOTO_Goal"},
                                remapping={"VS_input": "passing_data", "VS_output": "passing_data"});

        # Execute SMACH plan
    aut_explore = sm_exploration.execute();

if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine');
    robot_name_space = rospy.get_param("namespace", default="sos1");
    main();
    rospy.spin();
