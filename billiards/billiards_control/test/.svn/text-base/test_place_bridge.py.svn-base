#! /usr/bin/env python

import roslib
roslib.load_manifest('billiards_control')
import time
import sys
import rospy
from actionlib_msgs.msg import *
from billiards_msgs.msg import *
from geometry_msgs.msg import *
import actionlib.action_client
from math import pi, sin, cos

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

NODE = 'test_place_bridge'
rospy.init_node(NODE, disable_signals=True)

def PS(frame, x, y, z, qx=0, qy=0, qz=0, qw=1):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position = Point(x,y,z)
    ps.pose.orientation = Quaternion(qx,qy,qz,qw)
    return ps

def str_is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def a_placement(th):
    #assert abs(Constants.BRIDGE_IN_BASE_QZ - 0.7071) < 0.001
    #assert abs(Constants.BRIDGE_IN_BASE_QW - 0.7071) < 0.001
    return PS('base_link',
              Constants.BRIDGE_IN_BASE_X,
              Constants.BRIDGE_IN_BASE_Y,
              Constants.BRIDGE_IN_BASE_Z,
              0,0, sin(th/2.), cos(th/2.))

# The orientation differences matter way more than the translational differences
placements = {
    'a': a_placement(pi/2 + 0.08),
    's': a_placement(pi/2 + 0.03),
    'd': a_placement(pi/2 + 0.0),
    'f': a_placement(pi/2 + -0.03),
    'g': a_placement(pi/2 + -0.08),
}

placements = {
    'a': a_placement(pi/2 + 0.08),
    's': a_placement(pi/2 + 0.03),
    'd': a_placement(pi/2 + 0.0),
    'f': a_placement(pi/2 + -0.03),
    'g': a_placement(pi/2 + -0.08),
    'h': a_placement(pi/2 + 10./180*pi),
}


if __name__ == '__main__':
    args = rospy.myargv()
    client = actionlib.SimpleActionClient('place_bridge_action', PlaceBridgeAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("Never heard from place_bridge_action action server")
        sys.exit(1)

    if len(args) == 1:
        print "Lifting up"
        g = PlaceBridgeGoal()
        g.down = False
        client.send_goal(g)
        client.wait_for_result()
        if client.get_state() != GoalStatus.SUCCEEDED:
            print "Failed:", client.get_state()
            sys.exit(1)
        sys.exit(0)
    else:
        placement = None
        if str_is_number(args[1]):
            placement = a_placement(pi/2 + float(args[1]))
        elif args[1] in placements:
            placement = placements[args[1]]
        else:
            print "Invalid placement:", args[1]
            args[1] = 'd'
        
        print "Placing"
        g = PlaceBridgeGoal()
        g.down = True
        g.pose = placement
        client.send_goal(g)
        client.wait_for_result()
        if client.get_state() != GoalStatus.SUCCEEDED:
            print "Failed:", client.get_state()
            sys.exit(1)
        sys.exit(0)
        
