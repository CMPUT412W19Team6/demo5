#!/usr/bin/env python

import rospy
import cv2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cv_bridge
from geometry_msgs.msg import Twist, Pose, PoseStamped, PointStamped
from smach import State, StateMachine
import smach_ros
from dynamic_reconfigure.server import Server
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
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
import tf2_ros
import tf2_geometry_msgs
import tf

TAGS_FOUND = []
START_POSE = None
TAG_POSE = None
CURRENT_POSE = None
client = None
TAGS_IN_TOTAL = 3
CURRENT_STATE = None


class Turn(State):
    def __init__(self):
        State.__init__(self, outcomes=["find_far"],
                       output_keys=["current_marker"])
        self.rate = rospy.Rate(10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.marker_sub = rospy.Subscriber(
            'ar_pose_marker_base', AlvarMarkers, self.marker_callback)
        self.marker_detected = False
        self.rate = rospy.Rate(30)

    def execute(self, userdata):
        global CURRENT_STATE
        CURRENT_STATE = "turn"

        self.marker_detected = False
        while not self.marker_detected:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = -0.4
            self.cmd_pub.publish(msg)
            self.rate.sleep()

        userdata.current_marker = TAGS_FOUND[-1]
        self.marker_detected = False
        self.cmd_pub.publish(Twist())

        return "find_far"

    def marker_callback(self, msg):
        global TAG_POSE, TAGS_FOUND
        global CURRENT_STATE

        if CURRENT_STATE == "turn" and len(msg.markers) > 0:
            msg = msg.markers[0]

            if msg.id not in TAGS_FOUND and msg.id in [2, 3, 4]:
                TAGS_FOUND.append(msg.id)
                TAG_POSE = msg.pose.pose
                self.marker_detected = True


class MoveCloser(State):
    def __init__(self):
        State.__init__(self, outcomes=["close_enough"], output_keys=[
                       "goal"], input_keys=["current_marker"])
        self.rate = rospy.Rate(10)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.marker_sub = rospy.Subscriber(
            'ar_pose_marker_base', AlvarMarkers, self.marker_callback_base)

        self.current_marker = None
        self.tag_pose_base = None
        self.distance_from_marker = 0.2

        self.listener = tf.TransformListener()

    def execute(self, userdata):
        global CURRENT_POSE
        global CURRENT_STATE, START_POSE
        CURRENT_STATE = "move_closer"

        self.tag_pose_base = None
        self.current_marker = userdata.current_marker
        max_angular_speed = 0.8
        min_angular_speed = 0.0
        max_linear_speed = 0.8
        min_linear_speed = 0.0

        while True:
            if self.tag_pose_base is not None and self.tag_pose_base.position.x < 0.5:
                break
            elif self.tag_pose_base is not None and self.tag_pose_base.position.x > 0.5:
                move_cmd = Twist()

                if self.tag_pose_base.position.x > 0.6:  # goal too far
                    move_cmd.linear.x += 0.1
                elif self.tag_pose_base.position.x > 0.5:  # goal too close
                    move_cmd.linear.x -= 0.1
                else:
                    move_cmd.linear.x = 0

                if self.tag_pose_base.position.y < 1e-3:  # goal to the left
                    move_cmd.angular.z -= 0.1
                elif self.tag_pose_base.position.y > -1e-3:  # goal to the right
                    move_cmd.angular.z += 0.1
                else:
                    move_cmd.angular.z = 0

                move_cmd.linear.x = math.copysign(max(min_linear_speed, min(
                    abs(move_cmd.linear.x), max_linear_speed)), move_cmd.linear.x)
                move_cmd.angular.z = math.copysign(max(min_angular_speed, min(
                    abs(move_cmd.angular.z), max_angular_speed)), move_cmd.angular.z)

                move_cmd.linear.x = abs(move_cmd.linear.x)

                self.vel_pub.publish(move_cmd)
            self.rate.sleep()

        pose = PointStamped()
        pose.header.frame_id = "ar_marker_" + str(self.current_marker)
        pose.header.stamp = rospy.Time(0)
        pose.point.z = self.distance_from_marker

        self.listener.waitForTransform("odom", pose.header.frame_id, rospy.Time(0),rospy.Duration(4))
        
        pose_transformed = self.listener.transformPoint("odom", pose)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = pose_transformed.point.x
        goal.target_pose.pose.position.y = pose_transformed.point.y
        goal.target_pose.pose.orientation = START_POSE.orientation
        userdata.goal = goal

        return "close_enough"

    def marker_callback_base(self, msg):
        global CURRENT_STATE
        if CURRENT_STATE == "move_closer" and self.current_marker is not None:
            for marker in msg.markers:
                if marker.id == self.current_marker:
                    self.tag_pose_base = marker.pose.pose


class Navigate(State):
    def __init__(self, stage):
        State.__init__(self, outcomes=["find_all", "done"], input_keys=[
                       "goal"], output_keys=["goal"])
        self.rate = rospy.Rate(10)
        self.stage = stage

        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

    def execute(self, userdata):
        global client, TAGS_FOUND, START_POSE, TAGS_IN_TOTAL, CURRENT_POSE
        global CURRENT_STATE
        CURRENT_STATE = "navigate"

        if len(TAGS_FOUND) >= TAGS_IN_TOTAL and self.stage == "toStart":
            return "find_all"
        else:
            userdata.goal.target_pose.header.stamp = rospy.Time.now()
            result = self.move_base_client.send_goal_and_wait(userdata.goal)

            # turn_goal = MoveBaseGoal()
            # turn_goal.target_pose.header.frame_id = "odom"
            # turn_goal.target_pose.pose = CURRENT_POSE

            # euler = euler_from_quaternion((CURRENT_POSE.orientation.x, CURRENT_POSE.orientation.y, CURRENT_POSE.orientation.z, CURRENT_POSE.orientation.w))
            # quaternion = quaternion_from_euler(euler[0], euler[1], euler[2] * (-1))

            # turn_goal.target_pose.pose.orientation.x = quaternion[0]
            # turn_goal.target_pose.pose.orientation.y = quaternion[1]
            # turn_goal.target_pose.pose.orientation.z = quaternion[2]
            # turn_goal.target_pose.pose.orientation.w = quaternion[3]

            # self.move_base_client.send_goal_and_wait(turn_goal)
            # print(result)

            userdata.goal = MoveBaseGoal()
            userdata.goal.target_pose.header.frame_id = "odom"
            userdata.goal.target_pose.pose = START_POSE

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
                         "find_far": "MoveCloser"})

        StateMachine.add("MoveCloser", MoveCloser(), transitions={
                         "close_enough": "MoveToGoal"})

        StateMachine.add("MoveToGoal", Navigate("toGoal"), transitions={
                         "find_all": "success", "done": "BackToStart"})

        StateMachine.add("BackToStart", Navigate("toStart"),
                         transitions={"done": "Turn", "find_all": "success"})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
