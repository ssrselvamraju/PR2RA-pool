#! /usr/bin/env python

import roslib
roslib.load_manifest('billiards_executive')
import time
import sys
import rospy
from actionlib_msgs.msg import *
from billiards_msgs.msg import *
import actionlib.action_client

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

NODE = 'test_posestamped_to_tf'
rospy.init_node(NODE, disable_signals=True)

if __name__ == '__main__':
    args = rospy.myargv()
    client = actionlib.SimpleActionClient('posestamped_to_tf', PoseStampedToTFAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("Never heard from the action server")
        sys.exit(1)

    g = PoseStampedToTFGoal()
    g.pose.pose.position.x = 42
    g.pose.pose.orientation.z = 24
    g.pose.header.frame_id = 'foo'
    g.child_frame = 'bar'

    client.send_goal(g)
    client.wait_for_result()

    if client.get_state() != GoalStatus.SUCCEEDED:
        print "Failed:", client.get_state()
        sys.exit(1)
    sys.exit(0)
