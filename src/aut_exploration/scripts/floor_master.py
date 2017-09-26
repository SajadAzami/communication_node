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
import yaml


#TODO : check global using
# max number of robots = 8

gridsNum, robotsNum, aliveVictimsNum, deadVictimsNum = 0, 0, 0, 0
map_x1, map_y1, map_x2, map_y2 = 0.0, 0.0, 0.0, 0.0

lastRobotsCommand_Type = [] # [int:robotNumber] -> True:Goal, False:Grid
lastRobotsCommand_X = [] # [int:robotNumber] -> if is Goal -> (x) , else (grid number)
lastRobotsCommand_Y = [] # [int:robotNumber] -> if is Goal -> (y)

isFinishValidatingParams = False

totalNamespace = "sos"
floorNamespace = "floor"
floorNumber = 1

targetPercentage = 75 # initial percentage

finishTime = 0 # all time in seconds
startTime = 0 # remained time = finishTime - (rospy.Time.now().secs - startTime)

isFinishedExploringGrid = [] # [int:robotNumber] -> bool

agent_action = []

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

victNoticePublisher = rospy.Publisher('victim_found_notice', String, queue_size=10)


class GoalStyle(Enum):
    GOAL = 1,
    SORTED_GRID = 2,
    DEFAULT_GRID = 3,
    HELP_GRID = 4

def getFloorNamespace():
    global floorNamespace, floorNumber
    return floorNamespace + str(floorNumber) + "_" # TODO : "_" or "/"

def robotPoseSubscriber0(odom): # for <floorNamespace><floorNumber>/<totalNamespace>1
    global robotsPose
    robotsPose[0] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)
def robotPoseSubscriber1(odom): # for <floorNamespace><floorNumber>/<totalNamespace>2
    global robotsPose
    robotsPose[1] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)
def robotPoseSubscriber2(odom): # for <floorNamespace><floorNumber>/<totalNamespace>3
    global robotsPose
    robotsPose[2] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)
def robotPoseSubscriber3(odom): # for <floorNamespace><floorNumber>/<totalNamespace>4
    global robotsPose
    robotsPose[3] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)
def robotPoseSubscriber4(odom): # for <floorNamespace><floorNumber>/<totalNamespace>5
    global robotsPose
    robotsPose[4] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)
def robotPoseSubscriber5(odom): # for <floorNamespace><floorNumber>/<totalNamespace>6
    global robotsPose
    robotsPose[5] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)
def robotPoseSubscriber6(odom): # for <floorNamespace><floorNumber>/<totalNamespace>7
    global robotsPose
    robotsPose[6] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)
def robotPoseSubscriber7(odom): # for <floorNamespace><floorNumber>/<totalNamespace>8
    global robotsPose
    robotsPose[7] = thePoint(odom.pose.pose.position.x, odom.pose.pose.position.y)

poseCallBackFunctions = {'0':robotPoseSubscriber0, '1':robotPoseSubscriber1, '2':robotPoseSubscriber2, '3':robotPoseSubscriber3,
                         '4':robotPoseSubscriber4, '5':robotPoseSubscriber5, '6':robotPoseSubscriber6, '7':robotPoseSubscriber7}

def enterInDriveMode(robotName):
    global robotIsEnable, robotsNum, floorNamespace, totalNamespace
    print "enter in drive mode"
    robotIsEnable = [True] * robotsNum
    robotNum = -1
    if robotName == getFloorNamespace() + totalNamespace + "1":
        robotNum = 1
    elif robotName == getFloorNamespace() + totalNamespace + "2":
        robotNum = 2
    elif robotName == getFloorNamespace() + totalNamespace + "3":
        robotNum = 3
    elif robotName == getFloorNamespace() + totalNamespace + "4":
        robotNum = 4
    elif robotName == getFloorNamespace() + totalNamespace + "5":
        robotNum = 5
    elif robotName == getFloorNamespace() + totalNamespace + "6":
        robotNum = 6
    elif robotName == getFloorNamespace() + totalNamespace + "7":
        robotNum = 7
    elif robotName == getFloorNamespace() + totalNamespace + "8":
        robotNum = 8
    else:
        return

 
    robotIsEnable[robotNum] = False

def exitFromDriveMode():
    global robotIsEnable, robotsNum
    print "exit driver mode"
    for i in range(0, robotsNum):
        if not robotIsEnable[i]:
            robotIsEnable[i] = True
            if lastRobotsCommand_Type[i]:
                goToGoal(i, int(thePoint(lastRobotsCommand_X[i]), int(lastRobotsCommand_Y[i])))
            elif not lastRobotsCommand_X[i] == -1: # if is -1 => means that isn't set a command yet
                goToGrid(i, lastRobotsCommand_X[i])

def checkJoyData(data):
    if (data.buttons[5] == 1 or data.buttons[4] == 1):
        exitFromDriveMode        

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
    global robotsAliveVictims, robotsDeadVictims, aliveVictimsFoundNum, deadVictimsFoundNum, isHalted, robotsExploreGrid, stateNumber, victNoticePublisher
    
    othersFoundVictim("update")
    if isRepeatedVictim(x,y, isAlive):
        return false

    print ("victim found: pose(" + str(x) + " ," + str(y) + ") : " + "?isAlive? " + str(isAlive))

    if isAlive == True:
        robotsAliveVictims[robotNumber].append(Victim(x, y, robotsExploreGrid[robotNumber]))
        aliveVictimsFoundNum = aliveVictimsFoundNum + 1
        
        tempDict = {}
        for i in range(0, len(robotsAliveVictims)):
            tempNestedDict = {}
            for j in range(0, len(robotsAliveVictims[i])):
                tempNestedDict[str(j)] = robotsAliveVictims[i][j]
            tempDict[str(i)] = tempNestedDict
        rospy.set_param('robotsAliveVictims', str(tempDict))
        rospy.set_param('aliveVictimsFoundNum', aliveVictimsFoundNum)
        if stateNumber == 4:
            isHalted[robotNumber] = True
    else:
        robotsDeadVictims[robotNumber].append(Victim(x, y, robotsExploreGrid[robotNumber]))
        deadVictimsFoundNum = deadVictimsFoundNum + 1

        tempDict = {}
        for i in range(0, len(robotsDeadVictims)):
            tempNestedDict = {}
            for j in range(0, len(robotsDeadVictims[i])):
                tempNestedDict[str(j)] = robotsDeadVictims[i][j]
            tempDict[str(i)] = tempNestedDict
        rospy.set_param('robotsDeadVictims', str(tempDict))
        rospy.set_param('deadVictimsFoundNum', deadVictimsFoundNum)

    victNoticePublisher.publish("a new victim")
    return true # victim is confirmed

def othersFoundVictim(stringStatus):
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


def goToGoal(robotNumber, goalPoint):
    global isHalted, robotIsEnable, agent_action, robotsGoal, goalStyleStatus
    if isHalted[robotNumber] or not robotIsEnable[robotNumber]:
        return

    lastRobotsCommand_Type[robotNumber] = True
    lastRobotsCommand_X[robotNumber] = goalPoint.x
    lastRobotsCommand_Y[robotNumber] = goalPoint.y

    req = Master_toAgentGoal()
    req.command = 'standby'
    req.goal_x = goalPoint.x
    req.goal_y = goalPoint.y
    agent_action[robotNumber].send_goal(req)

    robotsGoal[robotNumber] = goalPoint
    goalStyleStatus[robotNumber] = GoalStyle.GOAL

def goToGrid(robotNumber, gridNumber):
    global isHalted, targetPercentage, gridCenters, isFinishedExploringGrid, aliveVictimsFoundNum, deadVictimsFoundNum, aliveVictimsNum, deadVictimsNum, robotsExploreGrid, Grids, robotIsEnable, agent_action
    
    if isHalted[robotNumber] or not robotIsEnable[robotNumber]:
        return

    lastRobotsCommand_Type[robotNumber] = False
    lastRobotsCommand_X[robotNumber] = gridNumber

    isFinishedExploringGrid[robotNumber] = False


    req = Master_toAgentGoal()
    if targetPercentage >= 85 and not aliveVictimsFoundNum + deadVictimsFoundNum == aliveVictimsNum + deadVictimsNum:
        req.command = 'detect_victim'
    else:
        req.command = 'explore'

    req.blocks = Grids[gridNumber]
    req.goal_x = gridCenters[gridNumber].x
    req.goal_y = gridCenters[gridNumber].y
    # TODO : percentage ro bgire
    agent_action[robotNumber].send_goal(req)


    robotsExploreGrid[robotNumber] = gridNumber

def getGridIndexInSortedArray(gridNumber): # return index of grid number in sorted array
    global sortedGridArray
    for i in range(0, len(sortedGridArray)):
        if sortedGridArray[i] == gridNumber:
            return i
    return -1

def agentCallback0(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[0]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[0] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(0, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[0].send_goal(req)

            if isConfirmed:
                rn = 0
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])

        else: # failed_toMOVE
            couldntReach(0)

def agentCallback1(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[1]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[1] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(1, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[1].send_goal(req)

            if isConfirmed:
                rn = 1
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])
        else: # failed_toMOVE
            couldntReach(1)
def agentCallback2(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[2]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[2] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(2, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[2].send_goal(req)
            if isConfirmed:
                rn = 2
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])           
        else: # failed_toMOVE
            couldntReach(2)
def agentCallback3(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[3]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[3] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(3, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[3].send_goal(req)
            if isConfirmed:
                rn = 3
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])
        else: # failed_toMOVE
            couldntReach(3)
def agentCallback4(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[4]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[4] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(4, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[4].send_goal(req)
            if isConfirmed:
                rn = 4
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])
        else: # failed_toMOVE
            couldntReach(4)
def agentCallback5(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[5]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[5] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(5, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[5].send_goal(req)
            if isConfirmed:
                rn = 5
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn]) 
        else: # failed_toMOVE
            couldntReach(5)
def agentCallback6(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[6]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[6] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(6, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[6].send_goal(req)
            if isConfirmed:
                rn = 6
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])
        else: # failed_toMOVE
            couldntReach(6)
def agentCallback7(agentData):
    global isFinishedExploringGrid, robotIsEnable, agent_action
    if robotIsEnable[7]:
        agentState = agentData.agent_state
        if agentState == 'exp_finished' or agentState == 'vicexp_finished':
            isFinishedExploringGrid[7] = True
        elif agentState == 'victim_is_detected':
            isAlive = False
            if agentData.vic_state == 2: # victim is alive
                isAlive = True
            isConfirmed = victimFound(7, agentData.vic_x, agentData.vic_y, isAlive)

            req = Master_toAgentGoal()
            if isConfirmed:
                req.command = 'victim_confirmed'
            else: 
                req.command = 'go_on'
            agent_action[7].send_goal(req)
            if isConfirmed:
                rn = 7
                if lastRobotsCommand_Type[rn]:
                    goToGoal(rn, int(thePoint(lastRobotsCommand_X[rn]), int(lastRobotsCommand_Y[rn])))
                elif not lastRobotsCommand_X[rn] == -1: # if is -1 => means that isn't set a command yet
                    goToGrid(rn, lastRobotsCommand_X[rn])
        else: # failed_toMOVE
            couldntReach(7)



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
    maxX = float('-inf');
    minX = float('inf');
    maxY = float('-inf');
    minY = float('inf');
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
    a = getPercentage(req) 
    print ("percentage of grid number " + gridNumber + " : " + str(a))
    return a 

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

    rospy.set_param(getNodeName() + 'targetPercentage', targetPercentage)

    tempDict = {}
    for i in range(0, len(isFinishedExploringGrid)):
        tempDict[str(i)] = isFinishedExploringGrid[i]
    rospy.set_param(getNodeName() + 'isFinishedExploringGrid', str(tempDict))

    tempDict = {}
    for i in range(0, len(isHalted)):
        tempDict[str(i)] = isHalted[i]
    rospy.set_param(getNodeName() + 'isHalted', str(tempDict))

    tempDictType = {}
    tempDictX = {}
    tempDictY = {}
    for i in range(0, len(lastRobotsCommand_Type)):
        tempDictType[str(i)] = lastRobotsCommand_Type[i]
        tempDictX[str(i)] = lastRobotsCommand_X[i]
        tempDictY[str(i)] = lastRobotsCommand_Y[i]
    rospy.set_param(getNodeName() + 'lastRobotsCommandType', str(tempDictType))
    rospy.set_param(getNodeName() + 'lastRobotsCommandX', str(tempDictX))
    rospy.set_param(getNodeName() + 'lastRobotsCommandY', str(tempDictY))

    # rospy.set_param('aliveVictimsFoundNum', aliveVictimsFoundNum)
    # rospy.set_param('deadVictimsFoundNum', deadVictimsFoundNum)

    
    # tempDict = {}
    # for i in range(0, len(robotsAliveVictims)):
    #     tempNestedDict = {}
    #     for j in range(0, len(robotsAliveVictims[i])):
    #         tempNestedDict[str(j)] = robotsAliveVictims[i][j]
    #     tempDict[str(i)] = tempNestedDict
    # rospy.set_param('robotsAliveVictims', str(tempDict))

    # tempDict = {}
    # for i in range(0, len(robotsDeadVictims)):
    #     tempNestedDict = {}
    #     for j in range(0, len(robotsDeadVictims[i])):
    #         tempNestedDict[str(j)] = robotsDeadVictims[i][j]
    #     tempDict[str(i)] = tempNestedDict
    # rospy.set_param('robotsDeadVictims', str(tempDict))
 
 
def getNodeName():
    return  rospy.get_name() + "/"

if __name__ == '__main__':
    rospy.init_node('master')

    rospy.on_shutdown(saveParameters)
    

    robotsNum = rospy.get_param(getNodeName() + "robotsNum")
    aliveVictimsNum = rospy.get_param("aliveVictimsNum")
    gridsNum = rospy.get_param(getNodeName() + "gridsNum")
    deadVictimsNum = rospy.get_param("deadVictimsNum")
    finishTime = rospy.get_param("finishTime")

    totalNamespace = rospy.get_param("total_namespace")
    floorNumber = rospy.get_param(getNodeName() + "floorNumber")

    map_x1 = rospy.get_param(getNodeName() + "map_x1")
    map_y1 = rospy.get_param(getNodeName() + "map_y1")
    map_x2 = rospy.get_param(getNodeName() + "map_x2")
    map_y2 = rospy.get_param(getNodeName() + "map_y2")

    print("master started")

    robotsExploreGrid = [0]*robotsNum
    tryNum = [[0]*gridsNum]*robotsNum
    goalStyleStatus = [GoalStyle.DEFAULT_GRID]*robotsNum
    robotsPose = [thePoint(0,0)]*robotsNum
    defaultGrids = [None] * robotsNum

    lastRobotsCommand_Type = [False] * robotsNum
    lastRobotsCommand_X = [-1] * robotsNum
    lastRobotsCommand_Y = [0] * robotsNum

    robotIsEnable = [True] * robotsNum

    for i in range(0, robotsNum):
        rospy.Subscriber(getFloorNamespace() + totalNamespace+str(i+1)+'/odom', Odometry , poseCallBackFunctions[str(i)]);
    gridCenters = [thePoint()] * gridsNum
    gridMap()

    for i in range (0, robotsNum):
        robotsPose[i].x = rospy.get_param(getNodeName() + "robot" + str(i+1) + "_x");
        robotsPose[i].y = rospy.get_param(getNodeName() + "robot" + str(i+1) + "_y");
    occupatedGrid = [0] * gridsNum
    for i in range (0, robotsNum):
        minDistance =  float('inf');
        index = -1;
        for j in range (0, gridsNum):
            if((occupatedGrid[j] == 0) or (i >= gridsNum)):
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
                centerX = (maxX + minX)/2
                centerY = (maxY + minY)/2
                gridCenters[j] = thePoint(centerX, centerY)
                d = distanceP2(centerX, centerY,robotsPose[i].x,robotsPose[i].y)
                print("distance = " + str(d))
                if (d <= minDistance):
                    minDistance = d;
                    index = j;
        occupatedGrid[index] = 1;
        defaultGrids[i] = index;
    print("robotsnumber = " + str(robotsNum))

    agent_server0 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '1/master_server', Agent_toMasterAction, execute_cb = agentCallback0, auto_start = True)
    agent_server1 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '2/master_server', Agent_toMasterAction, execute_cb = agentCallback1, auto_start = True)
    agent_server2 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '3/master_server', Agent_toMasterAction, execute_cb = agentCallback2, auto_start = True)
    agent_server3 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '4/master_server', Agent_toMasterAction, execute_cb = agentCallback3, auto_start = True)
    agent_server4 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '5/master_server', Agent_toMasterAction, execute_cb = agentCallback4, auto_start = True)
    agent_server5 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '6/master_server', Agent_toMasterAction, execute_cb = agentCallback5, auto_start = True)
    agent_server6 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '7/master_server', Agent_toMasterAction, execute_cb = agentCallback6, auto_start = True)
    agent_server7 = actionlib.SimpleActionServer(getFloorNamespace() + totalNamespace + '8/master_server', Agent_toMasterAction, execute_cb = agentCallback7, auto_start = True)
    # just for keep action servers in some variables

    rospy.Subscriber('drive_mode', String, enterInDriveMode)
    rospy.Subscriber('joy', Joy, checkJoyData)
    rospy.Subscriber('victim_found_notice', String, othersFoundVictim)


    agent_action = [None]*robotsNum

    for i in range(0, robotsNum):
        robotsAliveVictims.insert(i,[]);
        robotsDeadVictims.insert(i,[]);
        print("waiting for server :" + getFloorNamespace() + totalNamespace + str(i+1) + '/agent_server')
        agent_action[i] = actionlib.SimpleActionClient(getFloorNamespace() + totalNamespace+str(i+1)+'/agent_server', Master_toAgentAction);
        agent_action[i].wait_for_server();


    print("waiting for service :percentage_server")
    rospy.wait_for_service("percentage_server")

    print("waiting finished")

    # --- server parameters
    if rospy.has_param('startTime'):
        startTime = rospy.get_param('startTime')
    else:
        startTime = rospy.Time.now().secs
        rospy.set_param('startTime', startTime)

    if rospy.has_param(getNodeName() + 'targetPercentage'):
        targetPercentage = rospy.get_param(getNodeName() + 'targetPercentage')        

    isFinishedExploringGrid = [False] * robotsNum 
    if rospy.has_param(getNodeName() + 'isFinishedExploringGrid'):
        tempDict = yaml.load(rospy.get_param(getNodeName() + 'isFinishedExploringGrid'))
        print (tempDict)
        for i in range(0, robotsNum):
            isFinishedExploringGrid[i] = tempDict[str(i)]            

    isHalted = [False]*robotsNum
    if rospy.has_param(getNodeName() + 'isHalted'):
        tempDict = yaml.load(rospy.get_param(getNodeName() + 'isHalted'))
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

    if rospy.has_param(getNodeName() + 'lastRobotsCommandType'):
        tempDictType = yaml.load(rospy.get_param(getNodeName() + 'lastRobotsCommandType'))
        tempDictX = yaml.load(rospy.get_param(getNodeName() + 'lastRobotsCommandX'))
        tempDictY = yaml.load(rospy.get_param(getNodeName() + 'lastRobotsCommandY'))
        for i in range(0, robotsNum):
            lastRobotsCommand_Type[i] = tempDictType[str(i)]
            lastRobotsCommand_X[i] = tempDictX[str(i)]
            lastRobotsCommand_Y[i] = tempDictY[str(i)]

    isFinishValidatingParams = True
    # --- server parameters :end


    buildSM()
    
    rospy.spin()
