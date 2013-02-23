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

NODE = 'swing_arms_action'
rospy.init_node(NODE, disable_signals=True)
arm.init()

server = None
def execute_cb(goal):
    try:

        traj_l = JointTrajectory()
        traj_l.joint_names = arm.L_JOINTS
        traj_l.points = \
            [JointTrajectoryPoint(positions = qs, velocities = [0.0]*7, time_from_start = rospy.Duration(2.0))
             for qs in arm.SWING_L]

        traj_r = JointTrajectory()
        traj_r.joint_names = arm.R_JOINTS
        traj_r.points = \
            [JointTrajectoryPoint(positions = qs, velocities = [0.0]*7, time_from_start = rospy.Duration(2.0))
             for qs in arm.SWING_R]

        traj_l.header.stamp = traj_r.header.stamp = rospy.get_rostime() + rospy.Duration(1.0)

        if not goal.away:
            traj_l.points.reverse()
            traj_r.points.reverse()
            
        tfs = 2.0
        for pl, pr in zip(traj_l.points, traj_r.points):
            pl.time_from_start = rospy.Duration(tfs)
            pr.time_from_start = rospy.Duration(tfs)
            tfs += 2.0


        arm.l_arm_client.send_goal(JointTrajectoryGoal(trajectory = traj_l))
        arm.r_arm_client.send_goal(JointTrajectoryGoal(trajectory = traj_r))
        arm.l_arm_client.wait_for_result()
        arm.r_arm_client.wait_for_result()
        if arm.l_arm_client.get_state() == GoalStatus.SUCCEEDED and arm.r_arm_client.get_state() == GoalStatus.SUCCEEDED:
            server.set_succeeded()
        else:
            server.set_aborted()
        
    except Exception as ex:
        server.set_aborted()
        rospy.logerr("Swing arms failed:\n" + str(ex))


if __name__ == '__main__':
    global server
    server = actionlib.simple_action_server.SimpleActionServer(NODE, SwingArmsAction, execute_cb)
    rospy.spin()
