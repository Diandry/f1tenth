#! /usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from path_planning.msg import drive_param
import matplotlib.pyplot as plt


def filter_array(arr, threshold):
    # trim end elements from array. Ranges outside of -90 to 90 degrees. This assumes a 270 degree lidar sweep
    num = int(len(arr) / 6)  # samples to remove from both sides
    trimmed_arr = arr[num: -num]
    # plt.subplot(1, 2, 1)
    # plt.plot(trimmed_arr)

    i = 0
    while i < len(trimmed_arr) - 1:
        if trimmed_arr[i + 1] - trimmed_arr[i] >= threshold:
            # print "Found disparity. Curr: {0}, next {1}".format(trimmed_arr[i], trimmed_arr[i + 1])
            # overwrite the points after the disparity
            new_length = trimmed_arr[i]
            # margin = find_number_of_samples(0.4, new_length)  # number of samples around the safe zone
            margin = 250
            for j in range(0, margin):
                # the number of sample that need to be replaced
                if i + j < len(trimmed_arr) and trimmed_arr[i + j] > new_length:
                    trimmed_arr[i + j] = new_length
            i = i + margin
        i = i + 1

    #plt.subplot(1, 2, 2)
    #plt.plot(trimmed_arr)
    # plt.draw()
    # plt.pause(0.01)
    # find the angle corresponding to that distance
    max_index = trimmed_arr.argmax()
    if max_index <= 360:
        angle = (max_index / -4) * (np.pi / 180)
    else:
        angle = (max_index - 360) / 4 * (np.pi / 180)

    print 'max distance at angle: ', angle * 180 / np.pi, ",index: ", max_index, ", distance: ", trimmed_arr[max_index]
    return angle


# function returns the number of samples within half the width of the car to account for the car's size
# parameters: car_width
#             len: length to obstacle

def find_number_of_samples(car_width, len):
    angle = car_width / (2.0 * len)
    angle = np.arctan(angle)
    return int(np.ceil(angle / 0.0043633))


def callback(msg):
    arr = np.array(msg.ranges)
    angle = filter_array(arr, 0.3)
    message = drive_param(velocity=1, angle=angle)
    pub = rospy.Publisher('/drive_parameters', drive_param, queue_size=10)
    pub.publish(message)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.sleep(0.1)
rospy.spin()
