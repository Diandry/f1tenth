#! /usr/bin/env python
import numpy as np
import pandas as pd
from path_planning.msg import drive_param
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import message_filters
from std_msgs.msg import Float32
from BrakingAlgorithm import BrakingAlgorithm
from ackermann_msgs.msg import AckermannDriveStamped

import rospy


def callback(laser, velocity):
    distance_to_obstacle = laser.ranges[720]
    # vel = velocity.drive.speed * 1.2
    # vel = velocity.twist.twist.linear.x * 12
    vel = velocity.data * 1.2
    print "true distance: ", distance_to_obstacle, " vel: ", vel
    # acc = acceleration.linear_acceleration.x * 5
    brakes = BrakingAlgorithm()

    # inputs to braking algorithm
    input_arr = np.asarray([distance_to_obstacle / 12.0, vel / 4.0])
    print "input array ", input_arr
    braking_force = brakes.algo(input_arr)

    print "braking force: ", -braking_force[0][0]
    acceleration_pub = rospy.Publisher('acceleration', Float32, queue_size=5)
    acceleration_pub.publish(-braking_force[0][0] * 5.2)
    # drive_param_pub.publish(compute_brakes.algo())


if __name__ == '__main__':
    rospy.init_node('braking_system')
    laser_sub = message_filters.Subscriber('/scan', LaserScan)
    # velocity_sub = message_filters.Subscriber('/pf/pose/odom', Odometry)
    # vel_sub = message_filters.Subscriber('/vesc/low_level/ackermann_cmd_mux/output', AckermannDriveStamped)
    vel_sub = message_filters.Subscriber('current_velocity', Float32)
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, vel_sub], queue_size=5, slop=0.1,
                                                     allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()
