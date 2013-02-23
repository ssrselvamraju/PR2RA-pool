#! /usr/bin/env python


import roslib
roslib.load_manifest('billiards_control')

import time, copy
import rospy
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from actionlib_msgs.msg import *
from billiards_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from std_msgs.msg import *
import actionlib.action_client
from math import pi, sin, cos
import tf_conversions.posemath as pm
import arm

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

NODE = 'place_bridge_action'
rospy.init_node(NODE, disable_signals=True)
arm.init()

marker_pub = rospy.Publisher('/visualization_marker', Marker)

server = None
def execute_cb(goal):
    try:
        succeeded = False
        if goal.down:

            #### FIXED BRIDGE POSE ####
            goal.pose = arm.PS('base_link',
                               Constants.BRIDGE_IN_BASE_X,
                               Constants.BRIDGE_IN_BASE_Y,
                               Constants.TABLE_HEIGHT - 0.051,
                               Constants.BRIDGE_IN_BASE_QX,
                               Constants.BRIDGE_IN_BASE_QY,
                               Constants.BRIDGE_IN_BASE_QZ,
                               Constants.BRIDGE_IN_BASE_QW)



            b_marker = Marker()
            b_marker.ns = "place_bridge_action"
            b_marker.id = 1;
            b_marker.type = Marker.ARROW
            b_marker.header.frame_id = goal.pose.header.frame_id
            b_marker.header.stamp = rospy.get_rostime()
            b_marker.pose = goal.pose.pose
            b_marker.scale = Vector3(0.2, 0.2, 0.2)
            b_marker.color = ColorRGBA(1, 0, 1, 1)
            print "=== Publishing bridge desi marker:"
            print b_marker
            marker_pub.publish(b_marker)

            bridge_t = pm.fromMsg(goal.pose.pose)
            #r_in_bridge = arm.fromRaw(-1.093, 0.180, -0.425,   0.157, -0.177, -0.630, 0.740)
            #r_in_bridge = arm.fromRaw(-1.148, 0.333, -0.300,  0.076, -0.048, -0.721, 0.687)

            # First one we used in driving around tests.  Too high (angle puts cue into table)
            r_in_bridge = arm.fromRaw(-1.236, 0.260, -0.170,  0, 0, -0.7071, 0.7071)
            
            r_in_bridge = arm.fromRaw(-1.236, 0.260, -0.250,  0, 0, -0.7071, 0.7071)

            # This one is from the measuring session, trying to get it flat
            r_in_bridge = arm.fromRaw(-1.0, 0.17, -0.300,  0, 0, -0.7071, 0.7071)
            r_pose_t = bridge_t * r_in_bridge
            r_pose = PoseStamped()
            r_pose.header = goal.pose.header
            r_pose.pose = pm.toMsg(r_pose_t)


            #r_pose = arm.PS('bridge_frame', -1.148, 0.333, -0.300,  0.076, -0.048, -0.721, 0.687)
            #r_pose = arm.PS('bridge_frame', -1.1, 0.333, -0.300,  0, 0, -0.7071, 0.7071)

            
            rospy.loginfo("Lining up right arm for shot at: " + str(r_pose))
            r_marker = Marker()
            r_marker.ns = "place_bridge_action"
            r_marker.id = 0;
            r_marker.type = Marker.ARROW
            r_marker.header.frame_id = r_pose.header.frame_id
            r_marker.header.stamp = rospy.get_rostime()
            r_marker.pose = r_pose.pose
            r_marker.scale = Vector3(0.2, 0.2, 0.2)
            r_marker.color = ColorRGBA(1, 0, 1, 1)
            marker_pub.publish(r_marker)
            q_r = arm.ik_r(r_pose, arm.q_stage)
            
            if not q_r:
                raise Exception("IK failed for lining up right arm")
            
            if not arm.gotor(q_r):
                raise Exception("Failed to line up right arm")
            #arm.gotor(arm.q_stage)
            
            if not goal.pose.header.frame_id:
                arm.gotol(arm.BRIDGE)
                succeeded = arm.gotol(arm.BRIDGE_PUSH)
            else:
                #goal.pose.pose.position.z -= 0.025
                goal_above = copy.deepcopy(goal)
                goal_above.pose.pose.position.z += 0.03
                tool = arm.PS('bridge_frame', 0,0,0, 0,0,0,1)
                
                rospy.loginfo("Moving the bridge above its goal: " + str(goal_above))
                q = arm.ik_l(goal_above.pose, arm.BRIDGE, tool)
                if not q:
                    raise Exception("IK failed for moving above: " + str(goal_above))
                if not arm.gotol(q):
                    raise Exception("Moving to above failed")
                
                rospy.loginfo("Moving the bridge to its goal: " + str(goal))
                q = arm.ik_l(goal.pose, arm.BRIDGE, tool)
                if not q:
                    raise Exception("IK failed for placing bridge: " + str(goal_above))
                if not arm.gotol(q):
                    raise Exception("Moving to place the bridge failed")
                # TODO: push
        else:
            if not arm.gotol(arm.BRIDGE_UP):
                raise Exception("Moving arm to lift bridge failed")

        server.set_succeeded()
        
    except Exception as ex:
        server.set_aborted()
        rospy.logerr("Place bridge failed:\n" + str(ex))


if __name__ == '__main__':
    global server
    server = actionlib.simple_action_server.SimpleActionServer(NODE, PlaceBridgeAction, execute_cb)
    rospy.spin()
