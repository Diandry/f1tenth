#! /usr/bin/env python
import numpy as np
import pandas as pd
from path_planning.msg import drive_param
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import message_filters
import std_msgs.msg
from BrakingAlgorithm import BrakingAlgorithm

import rospy


def callback(laser, velocity, acceleration):
    distance_to_obstacle = laser.ranges[720]
    vel = velocity.twist.twist.linear.x * 12
    print "true distance: ", distance_to_obstacle, " vel: ", vel
    # acc = acceleration.linear_acceleration.x * 5
    brakes = BrakingAlgorithm()

    # inputs to braking algorithm
    input_arr = np.asarray([distance_to_obstacle / 12.0, vel / 4.0])
    print "input array ", input_arr
    braking_force = brakes.algo(input_arr)

    print "braking force: ", -braking_force[0][0]
    acceleration_pub = rospy.Publisher('acceleration', std_msgs.msg.Float32)
    acceleration_pub.publish(-braking_force[0][0] * 4.25)
    # drive_param_pub.publish(compute_brakes.algo())


if __name__ == '__main__':
    rospy.init_node('braking_system')
    laser_sub = message_filters.Subscriber('/scan', LaserScan)
    velocity_sub = message_filters.Subscriber('/pf/pose/odom', Odometry)
    accel_sub = message_filters.Subscriber('/imu', Imu)

    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, velocity_sub, accel_sub], queue_size=5, slop=0.1)
    ts.registerCallback(callback)
    rospy.spin()
