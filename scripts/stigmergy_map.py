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
# import imp


class StigmergyMap:

    def __init__(self):
        rospy.init_node('pheromone_map', anonymous=True)
        


        self.environment_width = rospy.get_param('~env_width', 100)
        self.environment_height = rospy.get_param('~env_height', 100)
        # how often to publish the local maps of each robot (Hz)
        self.publisher_rate = rospy.get_param('~update_rate', 10)
        #This specifies the name of the robot from range(a, number_of_robots)
        self.number_of_robots = rospy.get_param('~n_robots', 1)
        #This specifies the size of the stigmergy map
        self.map_resolution = rospy.get_param('~pheromone_resolution', 0.25)

        self.stigmergyMap_width = int( self.environment_width /  self.map_resolution)
        self.stigmergyMap_height = int( self.environment_height /  self.map_resolution)
        #This specifies the transform between the middle of the map to the top left corner of the stigmergy map (in metres)
        self.x_transform = 50
        self.y_transform = 50
        #This specifies by how many squares in the x and y direction do we want to sense local pheromones
        self.localResolution_x = int( rospy.get_param('~pheromone_sensing_radius', 1.6) / self.map_resolution)
        self.localResolution_y = int( rospy.get_param('~pheromone_sensing_radius', 1.6) / self.map_resolution)

        self.robotTrail_width = int( rospy.get_param('~robot_trail_radius',0.8) / self.map_resolution)
        self.robotTrail_height = int( rospy.get_param('~robot_trail_radius',0.8) / self.map_resolution)

        self.robotPheromoneStrength = rospy.get_param('~robot_trail_value', 75)
        self.wallPheromoneStrength = rospy.get_param('~wall_trail_value', 255)

        self.grid_map = OccupancyGrid()
        self.robot_map = Map( self.grid_map)
        self.map_initiated = False

        #This is the Map where we store the stigmergy globally
        self.stigmergyMap_groundExploration = np.zeros(( self.stigmergyMap_height,  self.stigmergyMap_width), dtype=np.uint16)
        self.stigmergyMap_walls = np.zeros(( self.stigmergyMap_height,  self.stigmergyMap_width), dtype=np.uint16)

        self.diffusion_sigma = rospy.get_param('~diffusion_sigma', 0.5)
        self.diffusion_rate = rospy.get_param('~diffusion_rate', 100)
        self.diffusion_counter = 0

        np.set_printoptions(threshold=np.nan)
        self.pub = []
        self.robots = []
        self.localMapStore = []

        # for i in range(0,  self.number_of_robots):
        #     self.localMapStore.append([])
        #     pubName = "localPheromone" + str(i)
        #     temp = rospy.Publisher(pubName, Int16MultiArray, queue_size=10)
        #     self.pub.append(temp)
    
        self.robot_names = self.get_robot_names()

        n_robots = len(self.robot_names)
        for i in range(0, n_robots):
            pubName = "localPheromone/" + str(self.robot_names[i])
            temp = rospy.Publisher(pubName, Int16MultiArray, queue_size=10)
            self.pub.append(temp)
            print "Created a publisher to: " + str(pubName)



    def getRobotID( self, child_frame_id):
        #child_frame_id: "robot3/base_link"
        sp = child_frame_id.split('/', 1)
        robotName = sp[0]
        robotID = robotName[-1]
        robotID = int(robotID)
        return robotID


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
        wallMap = self.findWalls(wallMap)
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
        shape = walls.shape
        midpoint = int(shape[0]/2)
        for n in range(0,4):
            for i in range(midpoint+1, shape[0]):
                b = (walls[i] == np.uint16(self.wallPheromoneStrength))
                wallCount = 0
                for j in range(0, shape[1]):
                    if b[j] == False:
                        break
                    elif b[j] == True:
                        wallCount = wallCount + 1
                if wallCount == shape[1]:
                    walls[i:shape[0]] = np.uint16(self.wallPheromoneStrength)
            walls = np.rot90(walls)

        return walls



    def storeLocalArea( self, x_robot, y_robot, ID):
        localStigmergyMap = self.getLocalArea( x_robot, y_robot)
        self.localMapStore[ID] = localStigmergyMap


    def printLocalArea( self, x_robot, y_robot):
        localStigmergyMap = self.getLocalArea( x_robot, y_robot)
        print localStigmergyMap

    def sendLocalArea( self, x_robot, y_robot):
        localStigmergyMap = self.getLocalArea( x_robot, y_robot)
        test = Int16MultiArray()
        test.data.insert(0, localStigmergyMap)
        self.pub.publish(test)

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

        self.stigmergyMap_groundExploration[ y_start:y_end, x_start:x_end] = self.stigmergyMap_groundExploration[ y_start:y_end, x_start:x_end] + np.uint16(self.robotPheromoneStrength)

    def transformMapToPheromones( self, array):

        array = np.flipud(array)
        shape = array.shape
        stride_w = shape[0] /  self.stigmergyMap_width
        stride_h = shape[1] /  self.stigmergyMap_height

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
        robotID = self.getRobotID(data.child_frame_id)
        robot = self.robots[robotID]
        robot.x = rawX
        robot.y = rawY
        self.robots[robotID] = robot
        mapX, mapY = self.transformRoboPosToPheromoneMap( rawX, rawY)
        self.leaveTrail( mapX, mapY)
        self.storeLocalArea( mapX, mapY, robotID)
        if robotID == 0:
            local = self.localMapStore[0]
            
    def callBackMap( self, data):
        self.robot_map = Map( data)
        self.robot_map.chopGrid()
        self.transformMapToPheromones(self.robot_map.chopMap)
        np.set_printoptions(threshold=np.nan)
    
    def mapsMissing( self):
        missing = False
        for i in range(0,  self.number_of_robots):
            if self.localMapStore[i] == []:
                print "THE MISSING MAP HAS ID NUMBER: " + str(i)
                missing = True
        return missing
    
    def publisher( self):
        if self.mapsMissing():
            print "EMPTY LOCAL STORES EMPTY LOCAL STORES EMPTY LOCAL STORES EMPTY LOCAL STORES EMPTY LOCAL STORES EMPTY LOCAL STORES EMPTY LOCAL STORES "

        else:
            local = self.localMapStore[0]
            if self.diffusion_counter == self.diffusion_rate:
                self.diffuse()
                self.diffusion_counter = 0
            else: 
                self.diffusion_counter = self.diffusion_counter + 1
            for i in range(0, self.number_of_robots):
                localStigmergyMap = self.localMapStore[i]
                print "ROBOT ID: " + str(i)
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

    def listener( self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        # j = 3
        # robotName = 'robot' + str(j) + '/odom'
        # rospy.Subscriber(robotName, Odometry, callback)

        for i in range(0, self.number_of_robots):
            robotName_odom = '/robot' + str(i) + '/odom'
            robotName_map = '/robot' + str(i) + '/map'
            self.robots.append( Robot(0.5,0.5,i))
            rospy.Subscriber(robotName_odom, Odometry, self.callBackOdom)
            rospy.Subscriber(robotName_map, OccupancyGrid, self.callBackMap)


        while not rospy.is_shutdown():

            self.publisher()
            r = rospy.Rate( self.publisher_rate) # 10hz
            r.sleep()
