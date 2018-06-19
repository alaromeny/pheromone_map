#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

from scipy.ndimage import gaussian_filter
import numpy as np
import math
import time

from robot import Robot
from map import Map
from stigmergy_map import StigmergyMap

if __name__ == '__main__':
    try:
        myMap = StigmergyMap()
        myMap.listener()
    except rospy.ROSInterruptException:
        pass




