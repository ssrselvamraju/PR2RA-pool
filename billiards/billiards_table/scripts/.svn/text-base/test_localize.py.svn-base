#!/usr/bin/env python

import roslib
roslib.load_manifest('billiards_table')
import time
import sys
import rospy
from actionlib_msgs.msg import *
from billiards_msgs.msg import *
import actionlib.action_client

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

NODE = 'test_localize'
rospy.init_node(NODE, disable_signals=True)

if __name__ == '__main__':
    if '--debug' in sys.argv:
        action = 'localize_table_debug'
    else:
        action = 'localize_table'
    print 'Connecting to %s server...' % action

    client = actionlib.SimpleActionClient(action, LocalizeTableAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr('never heard from the localize_table action server')
        sys.exit(1)
        
    print 'Connected to %s server' % action

    g = LocalizeTableGoal()

    print 'Sending goal'
    client.send_goal(g)

    print 'Waiting for result'
    client.wait_for_result()
    
    if client.get_state() != GoalStatus.SUCCEEDED:
        print 'Failed:', client.get_state()
        sys.exit(1)
    else:
        print 'Success'

    sys.exit(0)
