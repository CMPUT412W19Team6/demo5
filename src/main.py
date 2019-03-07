#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
from geometry_msgs.msg import Twist, Pose
from smach import State, StateMachine
import smach_ros
from dynamic_reconfigure.server import Server
from comp2.cfg import Comp2Config
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent, Sound, Led
from tf.transformations import decompose_matrix, compose_matrix
from ros_numpy import numpify
from sensor_msgs.msg import Joy, LaserScan, Image
import numpy as np
import angles as angles_lib
import math
import random
from std_msgs.msg import Bool, String, Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

TAGS_FOUND = []
START_POSE = None
TAG_POSE = None
CURRENT_POSE = None
client = None
TAGS_IN_TOTAL = 3


class Turn(State):
    def __init__(self):
        State.__init__(self, outcomes=["find_far", "find_close"])
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        pass


class MoveCloser(State):
    def __init__(self):
        State.__init__(self, outcomes=["close_enough"], output_keys=["goal"])
        self.rate = rospy.Rate(10)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def execute(self, userdata):
        global TAG_POSE, CURRENT_POSE

        D_vector = [TAG_POSE.position.x - CURRENT_POSE.position.x,
                    TAG_POSE.position.y - CURRENT_POSE.position.y]
        D = np.linalg.norm(D_vector)

        while D > 1.5:
            D_vector = [TAG_POSE.position.x - CURRENT_POSE.position.x,
                        TAG_POSE.position.y - CURRENT_POSE.position.y]
            D = np.linalg.norm(D_vector)
            Y = TAG_POSE.position.y - CURRENT_POSE.position.y

            t = Twist()
            t.linear.x = 0.5
            t.angular.z = Y

            self.vel_pub.publish(t)

            self.rate.sleep()

        s = np.sign(CURRENT_POSE.position.x - TAG_POSE.position.x)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = TAG_POSE.position.x + s * 0.5
        goal.target_pose.pose.position.y = TAG_POSE.position.y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0

        userdata.goal = goal

        return "close_enough"


class Navigate(State):
    def __init__(self, stage):
        State.__init__(self, outcomes=["find_all", "done"], input_keys=[
                       "goal"], output_keys=["goal"])
        self.rate = rospy.Rate(10)
        self.stage = stage

    def execute(self, userdata):
        global client, TAGS_FOUND, START_POSE, TAGS_IN_TOTAL

        # client send move base to userdata.goal

        # wait until done

        # after done, check do we need to go back to
        if len(TAGS_FOUND) >= TAGS_IN_TOTAL:
            return "find_all"

        return "done"


def odom_callback(msg):
    global CURRENT_POSE, START_POSE

    CURRENT_POSE = msg.pose.pose
    if START_POSE == None:
        START_POSE = CURRENT_POSE


if __name__ == "__main__":
    rospy.init_node('demo5')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    sm = StateMachine(outcomes=['success', 'failure'])
    sm.userdata.goal = None

    rospy.Subscriber("odom", Odometry, callback=odom_callback)

    with sm:

        StateMachine.add("Turn", Turn(), transitions={
                         "find_far": "MoveCloser", "find_close": "MoveToGoal"})

        StateMachine.add("MoveCloser", MoveCloser(), transitions={
                         "close_enough": "MoveToGoal"})

        StateMachine.add("MoveToGoal", Navigate("toGoal"), transitions={
                         "find_all": "success", "done": "BackToStart"})

        StateMachine.add("BackToStart", Navigate("toStart"),
                         transitions={"done": "Turn"})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
