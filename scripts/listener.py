#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from nav_msgs.msg import Odometry

from scipy.ndimage import gaussian_filter
import numpy as np
import math

# how often to publish the local maps of each robot (Hz)
publisher_rate = 1
#This specifies the name of the robot from range(a, number_of_robots)
number_of_robots = 4
#This specifies the size of the stigmergy map
#for now one square is 1metre
stigmergyMap_width = 100
stigmergyMap_height = 100
#This specifies the transform between the middle of the map to the top left corner of the stigmergy map (in metres)
x_transform = 50
y_transform = 50
#This specifies by how many squares in the x and y direction do we want to sense local pheromones
localResolution_x = 2
localResolution_y = 2

#This is the Map where we store the stigmergy globally
stigmergyMap = np.zeros((stigmergyMap_width, stigmergyMap_height), dtype=np.uint8)

pub = []

localMapStore = []

for i in range(0, number_of_robots):
    localMapStore.append([])
    pubName = "localPheromone" + str(i)
    temp = rospy.Publisher(pubName, Int16MultiArray, queue_size=10)
    pub.append(temp)


def getRobotID(child_frame_id):
    #child_frame_id: "robot3/base_link"
    sp = child_frame_id.split('/', 1)
    robotName = sp[0]
    robotID = robotName[-1]
    robotID = int(robotID)
    return robotID

def getLocalArea( x_robot, y_robot):
    x = int(math.floor(x_robot))
    y = int(math.floor(y_robot))
    lower_x = x-localResolution_x
    upper_x = x+localResolution_x+1
    lower_y = y-localResolution_y
    upper_y = y+localResolution_y+1

    if upper_y >= stigmergyMap_height:
        upper_y = stigmergyMap_height -1
    if upper_x >= stigmergyMap_width:
        upper_x = stigmergyMap_width -1
    if lower_x < 0:
        lower_x = 0
    if lower_y < 0:
        lower_y = 0

    localStigmergyMap = stigmergyMap[lower_x:upper_x, lower_y:upper_y]
    return localStigmergyMap

def storeLocalArea( x_robot, y_robot, ID):
    localStigmergyMap = getLocalArea( x_robot, y_robot)
    localMapStore[ID] = localStigmergyMap


def printLocalArea( x_robot, y_robot):
    localStigmergyMap = getLocalArea( x_robot, y_robot)
    print localStigmergyMap

def sendLocalArea( x_robot, y_robot):
    localStigmergyMap = getLocalArea( x_robot, y_robot)
    test = Int16MultiArray()
    test.data.insert(0, localStigmergyMap)
    # rospy.loginfo(test)
    pub.publish(test)



def transformRoboPosToPheromoneMap( x_robot, y_robot):
    newX = x_robot + x_transform
    newY = y_robot + y_transform
    return newX, newY

def leaveTrail( x_robot, y_robot):
    x = int(math.floor(x_robot))
    y = int(math.floor(y_robot))
    stigmergyMap[x, y] = np.uint8(255)
    # print("CLEAN DATA: " + str(x) + " " + str(y))

def callback(data):
    # print data
    # print 'Subscribing'
    rawX = data.pose.pose.position.x
    rawY = data.pose.pose.position.y
    robotID = getRobotID(data.child_frame_id)
    mapX, mapY = transformRoboPosToPheromoneMap( rawX, rawY)
    leaveTrail( mapX, mapY)
    storeLocalArea( mapX, mapY, robotID)


def publisher():

    if localMapStore[0] == []:
        rospy.loginfo("EMPTY!")
    else:
        for i in range(0,number_of_robots):
            localStigmergyMap = localMapStore[i]
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
            myPublisher = pub[i]
            rospy.loginfo("Sent message about robot "+ str(i))
            myPublisher.publish(message)


def listener():
    rospy.init_node('pheromone_map', anonymous=True)
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # j = 3
    # robotName = 'robot' + str(j) + '/odom'
    # rospy.Subscriber(robotName, Odometry, callback)

    for i in range(0,number_of_robots):
        robotName = 'robot' + str(i) + '/odom'
        print robotName
        rospy.Subscriber(robotName, Odometry, callback)

    while not rospy.is_shutdown():

        publisher()
        r = rospy.Rate(publisher_rate) # 10hz
        r.sleep()



    # test = multiArray()
    # test.data.insert(0, [380,399,380,380,380,380,380,380] )
    # print test

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.sleep(1.0)
    # rospy.spin()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass




