#! /usr/bin/env python

import roslib
roslib.load_manifest('billiards_control')
import time
import sys
import rospy
from actionlib_msgs.msg import *
from billiards_msgs.msg import *
import actionlib.action_client

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

NODE = 'test_play'
rospy.init_node(NODE, disable_signals=True)

if __name__ == '__main__':
    args = rospy.myargv()
    client = actionlib.SimpleActionClient('play_fixed', PlayAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("Never heard from the action server")
        sys.exit(1)

    g = PlayGoal()
    client.send_goal(g)
    client.wait_for_result()

    if client.get_state() != GoalStatus.SUCCEEDED:
        print "Failed:", client.get_state()
        sys.exit(1)
    sys.exit(0)
