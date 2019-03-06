#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
from geometry_msgs.msg import Twist
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

TAGS_FOUND = []
START_POSE = []


if __name__ == "__main__":
    rospy.init_node('demo5')

    sm = StateMachine(outcomes=['success', 'failure'])
    with sm:

        StateMachine.add("Turn", transitions={
                         "find_far": "MoveCloser", "find_close": "MoveToGoal"})

        StateMachine.add("MoveCloser", transitions={
                         "close_enough": "MoveToGoal"})

        StateMachine.add("MoveToGoal", transitions={
                         "find_all": "success", "done": "BackToStart"})

        StateMachine.add("BackToStart", transitions={"done": "Turn"})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
