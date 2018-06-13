from nav_msgs.msg import OccupancyGrid
from exceptions import IndexError
from geometry_msgs.msg import Point
import rospy
import math
import numpy as np

class Map:
    def __init__(self, grid_map):
        self.map = grid_map
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution
        self.env_width = 100
        self.env_height = 100
        self.midPoint_w = self.env_width/2
        self.midPoint_h = self.env_height/2
        self.chopMap = np.array(grid_map.data)
        self.npMap = np.array(self.map.data)


        self.origin = Point()
        self.origin.x = grid_map.info.origin.position.x
        self.origin.y = grid_map.info.origin.position.y

    def chopGrid(self):
        top_left_x, top_left_y = self.coord_to_indices( -1*self.midPoint_w, -1*self.midPoint_h)
        bottom_right_x, bottom_right_y = self.coord_to_indices( self.midPoint_w, self.midPoint_h)
        self.chopMap = np.array(self.map.data)
        self.chopMap = self.chopMap.reshape((self.width,self.height))
        self.chopMap = self.chopMap[top_left_x:bottom_right_x, top_left_y:bottom_right_y]

    def get_by_index(self, i, j):
        if not self.are_indices_in_range(i, j):
            raise IndexError()
        return self.map.data[i*self.width + j]

    # i is for row (y), j is for col (x)
    def get_by_coord(self, x, y):
        return self.get_by_index(*self.coord_to_indices(x, y))

    def coord_to_indices(self, x, y):
        i = int((y - self.origin.y) / self.resolution)
        j = int((x - self.origin.x) / self.resolution)
        return (i, j)

    def are_indices_in_range(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width

    def is_allowed(self, x_pos, y_pos, robot):
        was_error = False
        i, j = self.coord_to_indices( x_pos, y_pos)
        side = int(math.floor((max(robot.width, robot.height) / self.resolution) / 2))
        cells = []
        try:
            for s_i in range(i-side, i+side):
                for s_j in range(j-side, j+side):
                    cell = self.get_by_index(s_i, s_j)
                    cells.append(cell)
                    if cell == 100 or cell == -1:
                        return False
        except IndexError as e:
            # rospy.loginfo("Indices are out of range")
            was_error = True
        return True and not was_error


