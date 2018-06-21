#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import String, Float32, Int16MultiArray, MultiArrayDimension
from nav_msgs.msg import Odometry, OccupancyGrid

from scipy.ndimage import gaussian_filter
import numpy as np
import math
import time

from robot import Robot
from map import Map

class StigmergyMap:

    def __init__(self):
        rospy.init_node('pheromone_map', anonymous=True)
        
        self.environment_width = rospy.get_param('~env_width', 100)
        self.environment_height = rospy.get_param('~env_height', 100)
        # how often to publish the local maps of each robot (Hz)
        self.publisher_rate = rospy.get_param('~update_rate', 2)
        #This specifies the name of the robot from range(a, number_of_robots)
        #This specifies the size of the stigmergy map
        self.map_resolution = rospy.get_param('~pheromone_resolution', 0.25)

        self.stigmergyMap_width = int( self.environment_width /  self.map_resolution)
        self.stigmergyMap_height = int( self.environment_height /  self.map_resolution)
        #This specifies the transform between the middle of the map to the top left corner of the stigmergy map (in metres)
        self.x_transform = 50
        self.y_transform = 50
        #This specifies by how many squares in the x and y direction do we want to sense local pheromones
        self.localResolution_x = 12
        self.localResolution_y = 12
        self.robotTrail_width = 3
        self.robotTrail_height = 3

        self.robotPheromoneStrength = rospy.get_param('~robot_trail_value', 12700)
        self.wallPheromoneStrength = rospy.get_param('~wall_trail_value', 32700)
        self.maxPheromoneStrength = 15500

        self.grid_map = OccupancyGrid()
        self.robot_map = Map( self.grid_map)
        self.map_initiated = False

        #This is the Map where we store the stigmergy globally
        self.stigmergyMap_groundExploration = np.zeros(( self.stigmergyMap_height,  self.stigmergyMap_width), dtype=np.uint16)
        self.stigmergyMap_walls = np.zeros(( self.stigmergyMap_height,  self.stigmergyMap_width), dtype=np.uint16)

        self.diffusion_sigma = rospy.get_param('~diffusion_sigma', 0.5)
        self.diffusion_rate = rospy.get_param('~diffusion_rate', int((1/ self.publisher_rate)*0.25))
        self.diffusion_counter = 0

        np.set_printoptions(threshold=np.nan)
        self.pub = []
        self.robots = []
        self.localMapStore = []
        self.mapsRecieved = []

   
        self.robot_names = self.get_robot_names()       
        self.robot_myName = rospy.get_namespace() 

        self.number_of_robots = len(self.robot_names)
        for i in range(0, self.number_of_robots):
            tempRobot_name = str(self.robot_names[i])
            pubName = "/ground/localPheromone/" + tempRobot_name
            temp = rospy.Publisher(pubName, Int16MultiArray, queue_size=10)
            self.pub.append(temp)
            print "Created a publisher to: " + str(pubName)

            self.robots.append( Robot(0.5,0.5,i,tempRobot_name))

            robotName_odom = "/" + tempRobot_name + '/odom'
            robotName_map = "/" + tempRobot_name + '/map'
            if self.robot_myName == tempRobot_name:
                 robotName_odom = '/odom'
                 robotName_map = '/map'
                 
            rospy.Subscriber(robotName_odom, Odometry, self.callBackOdom)
            print "Created a Subscriber to: " + str(robotName_odom)
            rospy.Subscriber(robotName_map, OccupancyGrid, self.callBackMap)
            print "Created a Subscriber to: " + str(robotName_map)

            self.localMapStore.append([])

        np.set_printoptions(threshold=np.nan, linewidth=500)

    def get_robot_names( self):
        robot_names = set()
        for topic in rospy.get_published_topics():
            topic_name = topic[0]
            if "camera" in topic_name:
                robot_names.add(topic_name.split("/")[1])
        
        robot_names = sorted(list(robot_names))
        rospy.loginfo("Found Robots: %s" % robot_names)
        print "Found Robots: %s" % robot_names
        return robot_names


    def getRobotNamespace( self, child_frame_id):
        #child_frame_id: "robot3/base_link"
        sp = child_frame_id.split('/', 1)
        robotName = sp[0]
        return robotName

    def getRobotLocalIndex(self, nameSpace):
        return self.robot_names.index(nameSpace)

    def getLocalArea( self,  x_robot, y_robot):
        x = int(math.floor(x_robot))
        y = int(math.floor(y_robot))
        lower_x = x-self.localResolution_x
        upper_x = x+self.localResolution_x+1
        lower_y = y-self.localResolution_y
        upper_y = y+self.localResolution_y+1

        if upper_x >=  self.stigmergyMap_height:
            upper_x =  self.stigmergyMap_height -1
        if upper_y >=  self.stigmergyMap_width:
            upper_y =  self.stigmergyMap_width -1
        if lower_x < 0:
            lower_x = 0
        if lower_y < 0:
            lower_y = 0

        localStigmergyMap = np.array(self.stigmergyMap_groundExploration[lower_y:upper_y, lower_x:upper_x])
        wallMap = self.stigmergyMap_walls[lower_y:upper_y, lower_x:upper_x]
        #wallMap = self.findWalls(wallMap)
        localStigmergyMap = self.trimWalls(localStigmergyMap, wallMap)

        return localStigmergyMap


    def trimWalls( self, stigmergy, walls):
        shape = walls.shape
        for i in range(0, shape[0]):
            for j in range(0, shape[1]):
                if walls[i,j] == self.wallPheromoneStrength:
                    stigmergy[i,j] = self.wallPheromoneStrength

        return stigmergy

    def findWalls( self, walls):
        for n in range(0,4):
            shape = walls.shape
            midpoint = int(shape[0]/2)
            for i in range(midpoint+1, shape[0]):
                b = (walls[i] == np.uint16(self.wallPheromoneStrength))
                wallCount = 0
                for j in range(0, shape[1]):
                    if b[j] == False:
                        break
                    elif b[j] == True:
                        wallCount = wallCount + 1
                if wallCount == shape[1]:
                    walls[i:shape[1]] = np.uint16(self.wallPheromoneStrength)
            walls = np.rot90(walls)

        return walls

    def storeLocalArea( self, x_robot, y_robot, ID):
        localStigmergyMap = self.getLocalArea( x_robot, y_robot)
        self.localMapStore[ID] = localStigmergyMap


    def printLocalArea( self, x_robot, y_robot):
        localStigmergyMap = self.getLocalArea( x_robot, y_robot)
        print localStigmergyMap

    def diffuse( self):
        self.stigmergyMap_groundExploration = gaussian_filter(self.stigmergyMap_groundExploration, sigma=self.diffusion_sigma)

    def transformRoboPosToPheromoneMap( self, x_robot, y_robot):
        newX = (x_robot +  self.x_transform) /  self.map_resolution
        newY = ( self.y_transform - y_robot) /  self.map_resolution
        return newX, newY

    def leaveTrail( self, x_robot, y_robot):
        x = int(math.floor(x_robot))
        y = int(math.floor(y_robot))

        x_start = x - self.robotTrail_width
        y_start = y - self.robotTrail_height
        x_end = x + self.robotTrail_width
        y_end = y + self.robotTrail_height

        if x_start < 0:
            x_start=0
        if y_start < 0:
            y_start=0
        if x_end >= self.stigmergyMap_width:
            x_end=self.stigmergyMap_width
        if y_end >= self.stigmergyMap_height:
            y_end=self.stigmergyMap_height

        # self.stigmergyMap_groundExploration[ y_start:y_end, x_start:x_end] = np.clip((self.stigmergyMap_groundExploration[ y_start:y_end, x_start:x_end] + np.uint16(self.robotPheromoneStrength) * 0.75), 0, self.maxPheromoneStrength)
        self.stigmergyMap_groundExploration[ y_start:y_end, x_start:x_end] = self.robotPheromoneStrength




    def transformMapToPheromones( self, array):

        array = np.flipud(array)
        shape = array.shape
        stride_w = shape[1] /  self.stigmergyMap_width
        stride_h = shape[0] /  self.stigmergyMap_height

        for i in range(0,  self.stigmergyMap_width):
            start_i = i * stride_w
            end_i = start_i + stride_w
            for j in range(0,  self.stigmergyMap_height):
                start_j = j * stride_h
                end_j = start_j + stride_h
                subArr = array[start_i:end_i, start_j:end_j]
                maxCell =  np.amax(subArr)
                if maxCell > 50:
                    self.stigmergyMap_walls[i,j] = np.uint16(self.wallPheromoneStrength)

    def callBackOdom( self, data):
        rawX = data.pose.pose.position.x
        rawY = data.pose.pose.position.y
        robotNameSpace = self.getRobotNamespace(data.child_frame_id)
        robotID = self.getRobotLocalIndex(robotNameSpace)
        robot = self.robots[robotID]
        robot.x = rawX
        robot.y = rawY
        self.robots[robotID] = robot
        mapX, mapY = self.transformRoboPosToPheromoneMap( rawX, rawY)
        self.leaveTrail( mapX, mapY)
        self.storeLocalArea( mapX, mapY, robotID)
        #print "-------------------------"
        #print "ROBOT ID: " + str(robotID)
        #print "ODOM CALLBACK"
        #print mapX, mapY
        #print "-------------------------"

            
    def callBackMap( self, data):
        self.robot_map = Map( data)
        self.robot_map.chopGrid()
        self.transformMapToPheromones(self.robot_map.chopMap)
        
    
    def publisher( self):

        if self.diffusion_counter == self.diffusion_rate:
            self.diffuse()
            self.diffusion_counter = 0
        else: 
            self.diffusion_counter = self.diffusion_counter + 1
        for i in range(0, self.number_of_robots):
            localStigmergyMap = self.localMapStore[i]

            # print "-------------------------"
            # print "ROBOT ID: " + str(i)
            # print localStigmergyMap
            # print "-------------------------"

            if localStigmergyMap != []:
                origShape = np.shape(localStigmergyMap)
                #has to be flat to send as message (no idea why)
                localStigmergyMap = localStigmergyMap.flatten()
                flatShape = np.shape(localStigmergyMap)
                tmp = []
                #message also doesn't like np integers so create a temp array in which to store ints
                for k in range(0,flatShape[0]):
                    data = int(localStigmergyMap[k])
                    tmp.append(data)
                #create the message
                message = Int16MultiArray()
                #it's a message of 5*5 even though it's flattened, so this is length 25 at steps of 5
                message.layout.dim = [MultiArrayDimension("data", flatShape[0],  origShape[0])]
                message.data = tmp
                myPublisher = self.pub[i]
                myPublisher.publish(message)
            else:
                print "Robot " + self.robot_names[i] + " missing a pheromone map"



    def listener( self):

        while not rospy.is_shutdown():

            self.publisher()
            r = rospy.Rate( self.publisher_rate) # 10hz
            r.sleep()
