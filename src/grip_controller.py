#! /usr/bin/env python

import rospy
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from actionlib import SimpleActionClient, SimpleGoalState