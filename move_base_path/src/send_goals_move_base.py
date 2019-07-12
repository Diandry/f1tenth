#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal

waypoints = [(0.0, 0.0, 0.0, 1.0)]


def go_to_goal(x, y, z, w):
    global client
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    wait = client.wait_for_result(rospy.Duration(10.0))
    rospy.loginfo(str(wait))
    if not wait:
        rospy.logerr("Action server not available or goal did not complete")
    # The rospy signal shutdown command shuts down a node the argument is a string describing the reason for shutting the node down
    # rospy.signal_shutdown("Action sever not available!")
    else:
        return client.get_result()


def movebase_client():
    global client
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available")
        rospy.signal_shutdown("Action sever not available!")
    else:
        return client.get_result()


if __name__ == "__main__":
    try:
        rospy.init_node("movebase_client_py")
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        for i in range(len(waypoints)):
            goal = waypoints[i]
            result = go_to_goal(goal[0], goal[1], goal[2], goal[3])
            if result:
                rospy.loginfo("Goal Execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished")
