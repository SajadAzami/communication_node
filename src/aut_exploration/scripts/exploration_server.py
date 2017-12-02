#!/usr/bin/env python

import math
import time
import roslib
import rospy
import actionlib
import smach
import smach_ros
from enum import Enum
from geometry_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from nav_msgs.msg import *
from aut_exploration.srv import *
from aut_exploration.msg import *
from communication_node.msg import *
import yaml


#TODO : check global using
# max number of robots = 8

gridsNum, robotsNum, aliveVictimsNum, deadVictimsNum = 0, 0, 0, 0
map_x1, map_y1, map_x2, map_y2 = 0.0, 0.0, 0.0, 0.0

lastRobotsCommand_Type = [] # [int:robotNumber] -> True:Goal, False:Grid
lastRobotsCommand_X = [] # [int:robotNumber] -> if is Goal -> (x) , else (grid number)
lastRobotsCommand_Y = [] # [int:robotNumber] -> if is Goal -> (y)

isFinishValidatingParams = False

totalNamespace = "robot"


targetPercentage = 75 # initial percentage

finishTime = 0 # all time in seconds
startTime = 0 # remained time = finishTime - (rospy.Time.now().secs - startTime)

isFinishedExploringGrid = [] # [int:robotNumber] -> bool

server_publisher = None

stateNumber = -1
Grids = []
gridCenters = [] # [int:gridNumber] -> Point
sortedGridArray = []  # [int:index] -> int (grid number)
robotsAliveVictims, robotsDeadVictims = [], []  # [int:robotNum] -> Array[Victim]
aliveVictimsFoundNum, deadVictimsFoundNum = 0, 0

robotsPose = [] # [int:robotNum] -> thePoint
isHalted = [] # [int:robotNum] -> boolean

robotsGoal = [] # [int:robotNum] -> thePoint
robotsExploreGrid = [] # [int:robotNum] -> int:gridNumber
goalStyleStatus = [] # [int:robotNum] -> GoalStyle
tryNum = [] # [int:robotNum][int:gridNum] -> int:try numbers

defaultGrids = [] # [int:robotNumber] -> int: default grid number

robotIsEnable = [] # [int:robotNumber] -> True/False


class GoalStyle(Enum):
    GOAL = 1,
    SORTED_GRID = 2,
    DEFAULT_GRID = 3,
    HELP_GRID = 4
def robotPoseSubscriber(data): # for <totalNamespace>1
    global robotsPose
    robotsPose[(int(data.source[-1]))-1] = thePoint(data.data.pose.pose.position.x, data.data.pose.pose.position.y)

def enterInDriveMode(robotName):
    global robotIsEnable, robotsNum, totalNamespace
    print "enter in drive mode"
    robotIsEnable = [True] * robotsNum
    robotNum = -1
    if robotName == "robot1":
        robotNum = 1
    elif robotName == "robot2":
        robotNum = 2
    elif robotName == "robot3":
        robotNum = 3
    elif robotName == "robot4":
        robotNum = 4
    elif robotName == "robot5":
        robotNum = 5
    elif robotName == "robot6":
        robotNum = 6
    elif robotName == "robot7":
        robotNum = 7
    elif robotName == "robot8":
        robotNum = 8
    else:
        return


    robotIsEnable[robotNum] = False

def exitFromDriveMode():
    global robotIsEnable, robotsNum, lastRobotsCommand_Type, gridsNum, gridCenters, robotsPose, defaultGrids
    print "exit driver mode"
    for i in range(0, robotsNum):
        if not robotIsEnable[i]:
            robotIsEnable[i] = True
            if lastRobotsCommand_Type[i]:
                goToGoal(i, int(thePoint(lastRobotsCommand_X[i]), int(lastRobotsCommand_Y[i])))
            else: #not lastRobotsCommand_X[i] == -1: ## if is -1 => means that isn't set a command yet
                minDistance =  float('inf');
                index = -1;
                for j in range (0, gridsNum):
                    d = distanceP2(gridCenters[j].x, gridCenters[j].y,robotsPose[i].x,robotsPose[i].y)
                    if (d <= minDistance):
                        minDistance = d;
                        index = j;
                defaultGrids[i] = index
                goToGrid(i, index)

def checkJoyData(data):
    if (data.buttons[5] == 1 or data.buttons[4] == 1):
        exitFromDriveMode()

def isRepeatedVictim(x, y, isAlive):
    if isAlive == True:
        global robotsAliveVictims
        for i in range(0, len(robotsAliveVictims)):
            for victim in robotsAliveVictims[i]:
                if isSameVictim(victim, x, y):
                    return True

    else:
        global robotsDeadVictims
        for i in range(0, len(robotsDeadVictims)):
            for victim in robotsDeadVictims[i]:
                if isSameVictim(victim.x, x, y):
                    return True

    return False


def isSameVictim(oldVictPoint, newX, newY):
    return oldVictPoint.distance(newX, newY) < 2


def victimFound(robotNumber, x, y, isAlive):
    global robotsAliveVictims, robotsDeadVictims, aliveVictimsFoundNum, deadVictimsFoundNum, isHalted, robotsExploreGrid, stateNumber

    if isRepeatedVictim(x,y, isAlive):
        return false

    if isAlive == True:
        robotsAliveVictims[robotNumber].append(Victim(x, y, robotsExploreGrid[robotNumber]))
        if stateNumber == 4:
            isHalted[robotNumber] = True
    else:
        robotsDeadVictims[robotNumber].append(Victim(x, y, robotsExploreGrid[robotNumber]))

    return true # victim is confirmed

def goToGoal(robotNumber, goalPoint):
    global isHalted, robotIsEnable, server_publisher, robotsGoal, goalStyleStatus

    if isHalted[robotNumber] or not robotIsEnable[robotNumber]:
        return

    lastRobotsCommand_Type[robotNumber] = True
    lastRobotsCommand_X[robotNumber] = goalPoint.x
    lastRobotsCommand_Y[robotNumber] = goalPoint.y

    req = Data_MtA()
    req.source="exploration_master"
    req.destination="robot"+str(robotNumber);
    req_data=Master_toAgent();
    req_data.command = 'standby'
    req_data.goal_x = goalPoint.x
    req_data.goal_y = goalPoint.y
    req.data=req_data;
    server_publisher.publish(req)

    print ("goal send to robot (" + str(robotNumber) + ") : ( " + str(goalPoint.x) + ", " + str(goalPoint.y) + ")")


    global robotsGoal, goalStyleStatus
    robotsGoal[robotNumber] = goalPoint
    goalStyleStatus[robotNumber] = GoalStyle.GOAL

def goToGrid(robotNumber, gridNumber):
    global isHalted, targetPercentage, gridCenters, isFinishedExploringGrid, aliveVictimsFoundNum, deadVictimsFoundNum, aliveVictimsNum, deadVictimsNum, robotsExploreGrid, Grids, robotIsEnable, server_publisher
    if isHalted[robotNumber] or not robotIsEnable[robotNumber]:
        return

    lastRobotsCommand_Type[robotNumber] = False
    lastRobotsCommand_X[robotNumber] = gridNumber

    isFinishedExploringGrid[robotNumber] = False


    req = Data_MtA()
    req.source="exploration_master"
    req.destination="robot"+str(robotNumber)
    req_data=Master_toAgent();
    if targetPercentage >= 85 and not aliveVictimsFoundNum + deadVictimsFoundNum == aliveVictimsNum + deadVictimsNum:
        req_data.command = 'detect_victim'
    else:
        req_data.command = 'explore'

    req_data.blocks = Grids[gridNumber]
    req_data.goal_x = gridCenters[gridNumber].x
    req_data.goal_y = gridCenters[gridNumber].y
    # TODO : percentage ro bgire
    req.data=req_data;
    server_publisher.publish(req)

    print ("goal send to robot (" + str(robotNumber) + ") : gridNum = " + str(gridNumber) + " gridCenter:( "+ str(gridCenters[gridNumber].x) + ", " + str(gridCenters[gridNumber].y) + ")")

    robotsExploreGrid[robotNumber] = gridNumber

def getGridIndexInSortedArray(gridNumber): # return index of grid number in sorted array
    global sortedGridArray
    for i in range(0, len(sortedGridArray)):
        if sortedGridArray[i] == gridNumber:
            return i
    return -1

def agentCallback(agent_Data):
    global isFinishedExploringGrid, robotIsEnable, server_publisher
    agentData=agent_Data.data
    agent_number = int(agent_Data.source[-1])
    print ("agent (1) callback:" + agentData.agent_state)
    if robotIsEnable[agent_number]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[agent_number] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(agent_number, agentData.vic_x, agentData.vic_y, isAlive)

            req = Data_MtA()
            req_data=Master_toAgent();
            req.source="exploration_master"
            req.destination="robot"+str(agent_number)
            if isConfirmed:
                req_data.command = 'victim_confirmed'
            else:
                req_data.command = 'go_on'
            req.data=req_data;
            server_publisher.publish(req)

            if isConfirmed:
                rn = agent_number
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])

        else: # failed_toMOVE
            couldntReach(agent_number)



def couldntReach(robotNumber):
    global goalStyleStatus, tryNum, robotsGoal, robotsExploreGrid, isHalted, sortedGridArray, defaultGrids

    print ("robot " + str(robotNumber) + " couldn't reach")

    if isHalted[robotNumber] == True:
        return


    if goalStyleStatus[robotNumber] == GoalStyle.GOAL:
        goToGoal(robotNumber, robotsGoal[robotNumber])
    elif goalStyleStatus[robotNumber] == GoalStyle.SORTED_GRID or goalStyleStatus[robotNumber] == GoalStyle.HELP_GRID:
        tryNum[robotNumber][robotsExploreGrid[robotNumber]] = tryNum[robotNumber][robotsExploreGrid[robotNumber]] + 1
        if tryNum[robotNumber][robotsExploreGrid[robotNumber]] >= 3:
            index = getGridIndexInSortedArray(robotsExploreGrid[robotNumber])
            if index < len(sortedGridArray) - 1:
                index = index + 1
            goToGrid(robotNumber, sortedGridArray[index])
        else:
            goToGrid(robotNumber, robotsExploreGrid[robotNumber])
    else: # goalStyleStatus[robotNumber] == GoalStyle.DEFAULT_GRID
        tryNum[robotNumber][robotsExploreGrid[robotNumber]] = tryNum[robotNumber][robotsExploreGrid[robotNumber]] + 1
        if tryNum[robotNumber][robotsExploreGrid[robotNumber]] >= 3:
            gn = getSortedGridsNumbers()[0]
            defaultGrids[robotNumber] = gn
            goToGrid(robotNumber, gn)
        else:
            goToGrid(robotNumber, defaultGrids[robotNumber])


class Victim:
    def __init__(self,x_init,y_init, gridNumber):
        self.x = x_init
        self.y = y_init
        self.gridNumber = gridNumber
        self.hasSaviour = False # use in state 3, 4

class thePoint:
    def __init__(self,x_init=0,y_init=0):
        self.x = x_init
        self.y = y_init

    def shift(self, x, y):
        self.x += x
        self.y += y

    def __repr__(self):
        return "".join(["Point(", str(self.x), ",", str(self.y), ")"])

    def distance(self, p2):
        return math.sqrt((self.x-p2.x)**2+(self.y-p2.y)**2)

    def distance(self, x, y):
        return math.sqrt((self.x-x)**2+(self.y-y)**2)



class Exploring(smach.State): # state 1
    def __init__(self):
        smach.State.__init__(self, outcomes=['shut_down', 'time_over', 'explore_finished', 'victims_finished'])

    def execute(self, userdata):
        global stateNumber, targetPercentage, finishTime, startTime, aliveVictimsFoundNum, deadVictimsFoundNum, aliveVictimsNum, deadVictimsNum, robotsNum, defaultGrids, goalStyleStatus
        print "enter in state 1"

        stateNumber = 1

        global targetPercentage, finishTime, startTime
        global aliveVictimsFoundNum, deadVictimsFoundNum, aliveVictimsNum, deadVictimsNum, robotsNum, defaultGrids, goalStyleStatus

        for i in range(0, robotsNum):
            goToGrid(i, defaultGrids[i])

        while not rospy.is_shutdown():
            isFinished = True
            for i in range(0, robotsNum):
                if not isFinishedExploringGrid[i]:
                    isFinished = False
                else:
                    gn = getSortedGridsNumbers()[0]
                    goToGrid(i, gn)
                    goalStyleStatus[i] == GoalStyle.HELP_GRID
                    print("enter in help mode")



            if aliveVictimsFoundNum + deadVictimsFoundNum == aliveVictimsNum + deadVictimsNum:
                return 'victims_finished'

            if finishTime < rospy.Time.now().secs - startTime:
                return 'time_over'

            if isFinished:
                return 'explore_finished'

            rospy.sleep(1.) # check the exploration percentages each 1 seconds

        return 'shut_down'

class RaisePercentage(smach.State): # state 2
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_exploring'])

    def execute(self, userdata):
        global stateNumber, targetPercentage
        print "enter in state 2"

        stateNumber = 2

        global targetPercentage
        targetPercentage += 5

        return 'continue_exploring'



class VictimsFinished(smach.State): # state 3
    def __init__(self):
        smach.State.__init__(self, outcomes=['shut_down'])

    def execute(self, userdata):
        global stateNumber, robotsNum, robotsAliveVictims
        print "enter in state 3"

        stateNumber = 3

        hasGoal = []
        for i in range(0, robotsNum):
            if (len(robotsAliveVictims[i]) == 0):
                hasGoal.append(False)
            else:
                lastAliveVictIndex = len(robotsAliveVictims[i]) - 1
                goToGoal(i, robotAliveVictims[i][lastAliveVictIndex])
                robotsAliveVictims[i][lastAliveVictIndex].hasSaviour = True
                hasGoal.append(True)


        for r in range(0, robotsNum):
            if(hasGoal[i] == False):
                minDist = thePoint(map_x1, map_y1).distance(map_x2, map_y2)
                nearestVict = None
                rPose = robotsPose[r]
                for i in range(0, robotsNum):
                    for aliveVict in robotsAliveVictims[i]:
                        if(aliveVict.hasSaviour == False):
                            dist = thePoint(aliveVict.x, aliveVict.y).distance(rPose)
                            if(dist < minDist):
                                minDist = dist
                                nearestVict = aliveVict

                if(not nearestVict is None):
                    goToGoal(r, nearestVict)
                    nearestVict.hasSaviour = True
                    hasGoal[i] = True


        while not rospy.is_shutdown():
            rospy.sleep(10)

        return 'shut_down'

class Rescue(smach.State): # state 4
    def __init__(self):
        smach.State.__init__(self, outcomes=['shut_down'])

    def execute(self, userdata):

        global stateNumber, robotsGoal, robotsAliveVictims, robotsNum, robotsPose, targetPercentage, goalStyleStatus
        print "enter in state 4"

        stateNumber = 4

        global robotsGoal, robotsAliveVictims, robotsNum, robotsPose, targetPercentage
        targetPercentage = 100

        hasGoal = []

        for i in range(0, robotsNum):
            if (len(robotsAliveVictims[i]) == 0):
                hasGoal.append(False)
            else:
                lastAliveVictIndex = len(robotsAliveVictims[i]) - 1
                goToGoal(i, robotAliveVictims[i][lastAliveVictIndex])
                robotsAliveVictims[i][lastAliveVictIndex].hasSaviour = True
                hasGoal.append(True)


        for r in range(0, robotsNum):
            if(hasGoal[i] == False):
                minDist = thePoint(map_x1, map_y1).distance(map_x2, map_y2)
                nearestVict = None
                rPose = robotsPose[r]
                for i in range(0, robotsNum):
                    for aliveVict in robotsAliveVictims[i]:
                        if(aliveVict.hasSaviour == False):
                            dist = thePoint(aliveVict.x, aliveVict.y).distance(rPose)
                            if(dist < minDist):
                                minDist = dist
                                nearestVict = aliveVict

                if(not nearestVict is None):
                    goToGoal(r, nearestVict)
                    nearestVict.hasSaviour = True
                    hasGoal[i] = True


        sortedGridArray = getSortedGridsNumbers()
        gna = 0
        for i in range(0, robotsNum):
            if(hasGoal[i] == False):
                goToGrid(i, sortedGridArray[gna])
                goalStyleStatus[i] = GoalStyle.SORTED_GRID
                gna = gna + 1

        while not rospy.is_shutdown():
            rospy.sleep(10)

        return 'shut_down'


# --- grid

def getGridPercentage(gridNumber):
    global Grids
    getPercentage = rospy.ServiceProxy("percentage_server", Percentage_service)
    req = Percentage_serviceRequest()
    maxX = float('-inf')
    minX = float('inf')
    maxY = float('-inf')
    minY = float('inf')
    for k in range (0, len(Grids[gridNumber])):
        if (Grids[gridNumber][k].x > maxX):
            maxX = Grids[gridNumber][k].x;
        if (Grids[gridNumber][k].x < minX):
            minX = Grids[gridNumber][k].x;
        if (Grids[gridNumber][k].y > maxY):
            maxY = Grids[gridNumber][k].y;
        if (Grids[gridNumber][k].y < minY):
            minY = Grids[gridNumber][k].y;
    points = []
    points.append(Point(minX, minY, 0))
    points.append(Point(minX, maxY, 0))
    points.append(Point(maxX, minY, 0))
    points.append(Point(maxX, maxY, 0))
    req.points = points
    for vn in range(0,4):
      try:
          a = getPercentage(req)
          print ("percentage of grid number " + str(gridNumber) + " : " + str(a))
          return a.percentage
      except Exception as e:
          print (e)
          print ("percentage of grid failed failed failed failed failed")


def getSortedGridsNumbers():
    global gridsNum, sortedGridArray
    sortedGridArray = []
    for i in range(0, gridsNum):
        sortedGridArray.append(i)

    for i in range(0, gridsNum-1):
        for k in range(i, gridsNum-1):
            if getGridPercentage(sortedGridArray[k]) > getGridPercentage(sortedGridArray[k+1]):
                temp = sortedGridArray[k]
                sortedGridArray[k] = sortedGridArray[k+1]
                sortedGridArray[k+1] = temp

    return sortedGridArray

def xIsBigger(x,y):
    if x >= y:
        return 1;
    else:
        return 0;

def gridMap():
    global gridsNum, map_x1, map_x2, map_y1, map_y2, Grids
    print("grid map called with gridNumber = " + str(gridsNum))
    #working on grids
    x1 = map_x1;
    x2 = map_x2;
    y1 = map_y1;
    y2 = map_y2;
    #age tedade grid ha 1 bashe
    if gridsNum == 1:
        p1 = Point();
        p1.x = x1;
        p1.y = y1;

        p2 = Point();
        p2.x = x2;
        p2.y = y1;

        p3 = Point();
        p3.x = x2;
        p3.y = y2;

        p4 = Point();
        p4.x = x1;
        p4.y = y2;

        Grids.insert(0,[p1,p2,p3,p4]);
    #age tedade grid ha 4 bashe
    if gridsNum == 4:
        p1 = Point();
        p1.x = x1;
        p1.y = y1;

        p2 = Point();
        p2.x = (x1+x2)/2;
        p2.y = y1;

        p3 = Point();
        p3.x = x2;
        p3.y = y1;

        p4 = Point();
        p4.x = x2;
        p4.y = (y1+y2)/2;

        p5 = Point();
        p5.x = x2;
        p5.y = y2;

        p6 = Point();
        p6.x = (x1+x2)/2;
        p6.y = y2;

        p7 = Point();
        p7.x = x1;
        p7.y = y2;

        p8 = Point();
        p8.x = x1;
        p8.y = (y1+y2)/2;

        p9 = Point();
        p9.x = (x1+x2)/2;
        p9.y = (y1+y2)/2;

        Grids.insert(0,[p1,p2,p9,p8]);
        Grids.insert(1,[p2,p3,p4,p9]);
        Grids.insert(2,[p4,p5,p6,p9]);
        Grids.insert(3,[p6,p7,p8,p9]);

    if gridsNum == 5:
        p1 = Point();
        p1.x = x1;
        p1.y = y1;

        p2 = Point();
        p2.x = (x1+x2)/2;
        p2.y = y1;

        p3 = Point();
        p3.x = x2;
        p3.y = y1;

        p4 = Point();
        p4.x = x2;
        p4.y = (y1+y2)/2;

        p5 = Point();
        p5.x = x2;
        p5.y = y2;

        p6 = Point();
        p6.x = (x1+x2)/2;
        p6.y = y2;

        p7 = Point();
        p7.x = x1;
        p7.y = y2;

        p8 = Point();
        p8.x = x1;
        p8.y = (y1+y2)/2;

        p10 = Point();
        p10.x = (3*x1+x2)/4;
        p10.y = (3*y1+y2)/4;

        p11 = Point();
        p11.x = (x1+x2)/2;
        p11.y = (3*y1+y2)/4;

        p12 = Point();
        p12.x = (x1+3*x2)/4;
        p12.y = (3*y1+y2)/4;

        p13 = Point();
        p13.x = (x1+3*x2)/4;
        p13.y = (y1+y2)/2;

        p14 = Point();
        p14.x = (x1+3*x2)/4;
        p14.y = (y1+3*y2)/4;

        p15 = Point();
        p15.x = (x1+x2)/2;
        p15.y = (y1+3*y2)/4;

        p16 = Point();
        p16.x = (3*x1+x2)/4;
        p16.y = (y1+3*y2)/4;

        p17 = Point();
        p17.x = (3*x1+x2)/4;
        p17.y = (y1+y2)/2;

        Grids.insert(0,[p1,p2,p11,p10,p17,p8]);
        Grids.insert(1,[p2,p3,p4,p13,p12,p11]);
        Grids.insert(2,[p13,p4,p5,p6,p15,p14]);
        Grids.insert(3,[p6,p7,p8,p17,p16,p15]);
        Grids.insert(4,[p10,p12,p14,p16]);








    #age tedade grid ha 2 va 3 bashe
    elif xIsBigger(abs(x2 - x1), abs(y2 - y1)) == 1:
        if gridsNum == 2:
            p1 = Point();
            p1.x = x1;
            p1.y = y1;

            p2 = Point();
            p2.x = (x1+x2)/2;
            p2.y = y1;

            p3 = Point();
            p3.x = (x1+x2)/2;
            p3.y = y2;

            p4 = Point();
            p4.x = x1;
            p4.y = y2;

            p5 = Point();
            p5.x = x2;
            p5.y = y1;

            p6 = Point();
            p6.x = x2;
            p6.y = y2;

            Grids.insert(0,[p1,p2,p3,p4]);
            Grids.insert(1,[p2,p5,p6,p3]);
        elif gridsNum == 3:
            p1 = Point();
            p1.x = x1;
            p1.y = y1;

            p2 = Point();
            p2.x = (2*x1 + x2)/3;
            p2.y = y1;

            p3 = Point();
            p3.x = (2*x2 + x1)/3;
            p3.y = y1;

            p4 = Point();
            p4.x = x2;
            p4.y = y1;

            p5 = Point();
            p5.x = x2;
            p5.y = y2;

            p6 = Point();
            p6.x = (2*x2 + x1)/3;
            p6.y = y2;

            p7 = Point();
            p7.x = (2*x1 + x2)/3;
            p7.y = y2;

            p8 = Point();
            p8.x = x1;
            p8.y = y2;
            Grids.insert(0,[p1,p2,p7,p8]);
            Grids.insert(1,[p2,p3,p6,p7]);
            Grids.insert(2,[p3,p4,p5,p6]);
        elif gridsNum == 6:
            p1 = Point();
            p1.x = x1;
            p1.y = y1;

            p2 = Point();
            p2.x = (2*x1+x2)/3;
            p2.y = y1;

            p3 = Point();
            p3.x = (x1+2*x2)/3;
            p3.y = y1;

            p4 = Point();
            p4.x = x2;
            p4.y = y1;

            p5 = Point();
            p5.x = x2;
            p5.y = (y1+y2)/2;

            p6 = Point();
            p6.x = x2;
            p6.y = y2;

            p7 = Point();
            p7.x = (x1+2*x2)/3;
            p7.y = y2;

            p8 = Point();
            p8.x = (2*x1+x2)/3;
            p8.y = y2;

            p9 = Point();
            p9.x = x1;
            p9.y = y2;

            p10 = Point();
            p10.x = x1;
            p10.y = (y1+y2)/2;

            p11 = Point();
            p11.x = (2*x1+x2)/3;
            p11.y = (y1+y2)/2;

            p12 = Point();
            p12.x = (x1+2*x2)/3;
            p12.y = (y1+y2)/2;


            Grids.insert(0,[p1,p2,p11,p10]);
            Grids.insert(1,[p2,p3,p12,p11]);
            Grids.insert(2,[p10,p11,p8,p9]);
            Grids.insert(3,[p11,p12,p7,p8]);
            Grids.insert(4,[p12,p5,p4,p3]);
            Grids.insert(5,[p12,p5,p6,p7]);




    elif xIsBigger(abs(x2 - x1), abs(y2 - y1)) == 0:
        if gridsNum == 2:
            p1 = Point();
            p1.x = x1;
            p1.y = y1;

            p2 = Point();
            p2.x = x2;
            p2.y = y1;

            p3 = Point();
            p3.x = x2;
            p3.y = (y1+y2)/2;

            p4 = Point();
            p4.x = x1;
            p4.y = (y1+y2)/2;

            p5 = Point();
            p5.x = x1;
            p5.y = y2;

            p6 = Point();
            p6.x = x2;
            p6.y = y2;

            Grids.insert(0,[p1,p2,p3,p4]);
            Grids.insert(1,[p3,p4,p5,p6]);
        elif gridsNum == 3:
            p1 = Point();
            p1.x = x1;
            p1.y = y1;

            p2 = Point();
            p2.x = x2;
            p2.y = y1;

            p3 = Point();
            p3.x = x2;
            p3.y = (2*y1+y2)/3;

            p4 = Point();
            p4.x = x2;
            p4.y = (y1+2*y2)/3;

            p5 = Point();
            p5.x = x2;
            p5.y = y2;

            p6 = Point();
            p6.x = x1;
            p6.y = y2;

            p7 = Point();
            p7.x = x1;
            p7.y = (y1+2*y2)/3;

            p8 = Point();
            p8.x = x1;
            p8.y = (2*y1+y2)/3;

            Grids.insert(0,[p1,p2,p3,p8]);
            Grids.insert(1,[p3,p4,p7,p8]);
            Grids.insert(2,[p4,p5,p6,p7]);

        elif gridsNum == 6:
            p1 = Point();
            p1.x = x1;
            p1.y = y1;

            p2 = Point();
            p2.x = (x1+x2)/2;
            p2.y = y1;

            p3 = Point();
            p3.x = x2;
            p3.y = y1;

            p4 = Point();
            p4.x = x2;
            p4.y = (2*y1+y2)/3;

            p5 = Point();
            p5.x = x2;
            p5.y = (y1+2*y2)/3;

            p6 = Point();
            p6.x = x2;
            p6.y = y2;

            p7 = Point();
            p7.x = (x1+x2)/2;
            p7.y = y2;

            p8 = Point();
            p8.x = x1;
            p8.y = y2;

            p9 = Point();
            p9.x = x1;
            p9.y = (y1+2*y2)/3;

            p10 = Point();
            p10.x = x1;
            p10.y = (2*y1+y2)/3;

            p11 = Point();
            p11.x = (x1+x2)/2;
            p11.y = (2*y1+y2)/3;

            p12 = Point();
            p12.x = (x1+x2)/2;
            p12.y = (y1+2*y2)/3;

            Grids.insert(0,[p1,p2,p11,p10]);
            Grids.insert(1,[p2,p3,p4,p11]);
            Grids.insert(2,[p10,p11,p12,p9]);
            Grids.insert(3,[p11,p4,p5,p12]);
            Grids.insert(4,[p9,p12,p7,p8]);
            Grids.insert(5,[p12,p5,p6,p7]);

# --- grid:end



def buildSM():
    print("buildSM called")
    sm_master = smach.StateMachine(outcomes=['shut_down']);
    with sm_master: # main state machine
        smach.StateMachine.add('EXPLORING', Exploring(),
                               transitions={'time_over':'RESCUE', 'explore_finished':'RAISE_PERCENTAGE', 'victims_finished':'VICTIMS_FINISHED'})

        smach.StateMachine.add('RAISE_PERCENTAGE', RaisePercentage(),
                               transitions={'continue_exploring':'EXPLORING'})

        smach.StateMachine.add('VICTIMS_FINISHED', VictimsFinished(),
                               transitions={})

        smach.StateMachine.add('RESCUE', Rescue(),
                               transitions={})


    sis = smach_ros.IntrospectionServer('master_viewer', sm_master, '/SM_MASTER')
    sis.start()
    outcome = sm_master.execute()



def distanceP2(x1,y1,x2,y2):
    return ((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

def saveParameters():
    print ("master is shutdown")
    if isFinishValidatingParams == False:
        return

    rospy.set_param('targetPercentage', targetPercentage)

    tempDict = {}
    for i in range(0, len(isFinishedExploringGrid)):
        tempDict[str(i)] = isFinishedExploringGrid[i]
    rospy.set_param('isFinishedExploringGrid', str(tempDict))

    tempDict = {}
    for i in range(0, len(isHalted)):
        tempDict[str(i)] = isHalted[i]
    rospy.set_param('isHalted', str(tempDict))

    tempDictType = {}
    tempDictX = {}
    tempDictY = {}
    for i in range(0, len(lastRobotsCommand_Type)):
        tempDictType[str(i)] = lastRobotsCommand_Type[i]
        tempDictX[str(i)] = lastRobotsCommand_X[i]
        tempDictY[str(i)] = lastRobotsCommand_Y[i]
    rospy.set_param('lastRobotsCommandType', str(tempDictType))
    rospy.set_param('lastRobotsCommandX', str(tempDictX))
    rospy.set_param('lastRobotsCommandY', str(tempDictY))

    rospy.set_param('aliveVictimsFoundNum', aliveVictimsFoundNum)
    rospy.set_param('deadVictimsFoundNum', deadVictimsFoundNum)


    tempDict = {}
    for i in range(0, len(robotsAliveVictims)):
        tempNestedDict = {}
        for j in range(0, len(robotsAliveVictims[i])):
            tempNestedDict[str(j)] = robotsAliveVictims[i][j]
        tempDict[str(i)] = tempNestedDict
    rospy.set_param('robotsAliveVictims', str(tempDict))

    tempDict = {}
    for i in range(0, len(robotsDeadVictims)):
        tempNestedDict = {}
        for j in range(0, len(robotsDeadVictims[i])):
            tempNestedDict[str(j)] = robotsDeadVictims[i][j]
        tempDict[str(i)] = tempNestedDict
    rospy.set_param('robotsDeadVictims', str(tempDict))


if __name__ == '__main__':
    rospy.init_node('master')

    rospy.on_shutdown(saveParameters)

    robotsNum = rospy.get_param("robotsNum");
    aliveVictimsNum = rospy.get_param("aliveVictimsNum");
    gridsNum = rospy.get_param("gridsNum");
    deadVictimsNum = rospy.get_param("deadVictimsNum");
    finishTime = rospy.get_param("finishTime")

    totalNamespace = rospy.get_param("total_namespace")
    print("master started")
    map_x1 = rospy.get_param("map_x1");
    map_y1 = rospy.get_param("map_y1");
    map_x2 = rospy.get_param("map_x2");
    map_y2 = rospy.get_param("map_y2");

    robotsExploreGrid = [0]*robotsNum
    tryNum = [[0]*gridsNum]*robotsNum
    goalStyleStatus = [GoalStyle.DEFAULT_GRID]*robotsNum
    robotsPose = [thePoint(0,0)]*robotsNum
    defaultGrids = [None] * robotsNum

    lastRobotsCommand_Type = [False] * robotsNum
    lastRobotsCommand_X = [-1] * robotsNum
    lastRobotsCommand_Y = [0] * robotsNum

    robotIsEnable = [True] * robotsNum


    rospy.Subscriber('/exploration_master/inbox_Odom', Data_Odom , robotPoseSubscriber);
    gridCenters = [thePoint()] * gridsNum
    gridMap()

    rospy.sleep(2) # to be sure that initile positions are ready
    #for i in range (0, robotsNum):
    #    robotsPose[i].x = rospy.get_param("robot" + str(i+1) + "_x");
    #    robotsPose[i].y = rospy.get_param("robot" + str(i+1) + "_y");

    occupatedGrid = [0] * gridsNum

    for j in range (0, gridsNum):
        maxX = float('-inf');
        minX = float('inf');
        maxY = float('-inf');
        minY = float('inf');
        for k in range (0, len(Grids[j])):
            if (Grids[j][k].x > maxX):
                maxX = Grids[j][k].x;
            if (Grids[j][k].x < minX):
                minX = Grids[j][k].x;
            if (Grids[j][k].y > maxY):
                maxY = Grids[j][k].y;
            if (Grids[j][k].y < minY):
                minY = Grids[j][k].y;
            gridCenters[j] = thePoint((maxX + minX)/2, (maxY + minY)/2)

    for i in range (0, robotsNum):
        minDistance =  float('inf');
        index = -1;
        for j in range (0, gridsNum):
            if((occupatedGrid[j] == 0) or (i >= gridsNum)):
                centerX = gridCenters[j].x
                centerY = gridCenters[j].y
                d = distanceP2(centerX, centerY,robotsPose[i].x,robotsPose[i].y)
                print("distance = " + str(d))
                if (d <= minDistance):
                    minDistance = d;
                    index = j;
        occupatedGrid[index] = 1;
        defaultGrids[i] = index;

    print("robotsnumber = " + str(robotsNum))
    server_subscriber = rospy.Subscriber( '/exploration_master/inbox_AtM', Data_AtM,  agentCallback)
    # just for keep action servers in some variables

    rospy.Subscriber('drive_mode', String, enterInDriveMode)
    rospy.Subscriber('joy', Joy, checkJoyData)


    server_publisher = rospy.Publisher('/message_server_MtA', Data_MtA,queue_size=10);

    for i in range(0, robotsNum):
        robotsAliveVictims.insert(i,[]);
        robotsDeadVictims.insert(i,[]);


    print("waiting for service :percentage_server")
    rospy.wait_for_service("percentage_server")

    print("waiting finished")

    # --- server parameters
    if rospy.has_param('startTime'):
        startTime = rospy.get_param('startTime')
    else:
        startTime = rospy.Time.now().secs
        rospy.set_param('startTime', startTime)

    if rospy.has_param('targetPercentage'):
        targetPercentage = rospy.get_param('targetPercentage')

    isFinishedExploringGrid = [False] * robotsNum
    if rospy.has_param('isFinishedExploringGrid'):
        tempDict = yaml.load(rospy.get_param('isFinishedExploringGrid'))
        print (tempDict)
        for i in range(0, robotsNum):
            isFinishedExploringGrid[i] = tempDict[str(i)]

    isHalted = [False]*robotsNum
    if rospy.has_param('isHalted'):
        tempDict = yaml.load(rospy.get_param('isHalted'))
        for i in range(0, robotsNum):
            isHalted[i] = tempDict[str(i)]



    if rospy.has_param('aliveVictimsFoundNum'):
        aliveVictimsFoundNum = rospy.get_param('aliveVictimsFoundNum')

    if rospy.has_param('deadVictimsFoundNum'):
        deadVictimsFoundNum = rospy.get_param('deadVictimsFoundNum')

    if rospy.has_param('robotsAliveVictims'):
        tempDict = yaml.load(rospy.get_param('robotsAliveVictims'))
        for i in range(0, robotsNum):
            for j in range(0, len(tempDict[str(i)])):
                robotsAliveVictims[i][j] = tempDict[str(i)][str(j)]


    if rospy.has_param('robotsDeadVictims'):
        tempDict = yaml.load(rospy.get_param('robotsDeadVictims'))
        for i in range(0, robotsNum):
            for j in range(0, len(tempDict[str(i)])):
                robotsDeadVictims[i][j] = tempDict[str(i)][str(j)]

    if rospy.has_param('lastRobotsCommandType'):
        tempDictType = yaml.load(rospy.get_param('lastRobotsCommandType'))
        tempDictX = yaml.load(rospy.get_param('lastRobotsCommandX'))
        tempDictY = yaml.load(rospy.get_param('lastRobotsCommandY'))
        for i in range(0, robotsNum):
            lastRobotsCommand_Type[i] = tempDictType[str(i)]
            lastRobotsCommand_X[i] = tempDictX[str(i)]
            lastRobotsCommand_Y[i] = tempDictY[str(i)]

    isFinishValidatingParams = True
    # --- server parameters :end


    buildSM()

    rospy.spin()
