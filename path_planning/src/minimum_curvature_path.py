#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import cupy as cp
import matplotlib.pyplot as plt


# todo - implement
def find_curvature(prev, curr, next):
    numerator = 2 * np.linalg.det(np.subtract(next, curr), np.subtract(prev, curr))
    return numerator



class Map:
    def __init__(self):
        # local occupancy grid
        # self.grid_map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        self.grid_map = np.loadtxt('test_track_array.txt', dtype=int)
        self.map = OccupancyGrid()
        self.border_arr = np.array([])
        self.map_width = 0
        self.map_height = 0
        self.origin = [10,4] # for the test array
        self.get_border_coordinates()

    # def __init__(self):
    #     # local occupancy grid
    #     # self.grid_map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
    #     self.grid_map = np.loadtxt('test_track_array.txt', dtype=int)
    #     self.map = OccupancyGrid()
    #     self.border_arr = np.array([])
    #     self.map_width = 0
    #     self.map_height = 0
    #     self.origin = Pose()

    def map_callback(self, new_map):
        self.map = new_map
        print "Map received!"
        self.origin = self.map.info.origin
        self.map_height = self.map.info.height
        self.map_width = self.map.info.width
        self.get_border_coordinates()

    def get_border_coordinates(self):
        # self.border_arr = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        self.border_arr = self.grid_map  # TODO - just for debugging
        print self.border_arr
        # plt.imshow(self.border_arr,cmap=cm.Greys_r)
        # plt.show()
        output = []
        start = [10, 4]
        print self.get_middle_path()
        # for i in range(0, self.map_height):
        #     for j in range(0, self.map_width):
        #         if self.border_arr[i, j] == 100:
        #             # output.append([i, j])
        #             output.append([i, j])

        for i in range(0, self.border_arr.shape[0]):
            for j in range(0, self.border_arr.shape[1]):
                if self.border_arr[i, j] == 1:
                    # output.append([i, j])
                    output.append([i, j])

        # # plt.show()
        # plt.plot(output, 'ro')
        # plt.show()

    # a very naive attempt at getting the middle path. Parts of the given path should be invalid
    def get_middle_path(self):
        start = [10, 4]
        start1= [10, 4]
        print self.find_upper_border(start)
        print self.find_lower_border(start1)

    def find_upper_border(self, point):
        temp = point
        while self.border_arr[point[0], point[1]] != 1:
            temp[0] += 1
        return point

    def find_lower_border(self, point):
        temp = point
        while self.border_arr[point[0], point[1]] != 1:
            temp[0] -= 1
        return temp




if __name__ == '__main__':
    # rospy.init_node('map_listener', anonymous=True)
    track_map = Map()
    # rospy.spin()
