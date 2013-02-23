#! /usr/bin/env python


import roslib
roslib.load_manifest('billiards_control')

import time
import rospy
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from actionlib_msgs.msg import *
from billiards_msgs.msg import *
from geometry_msgs.msg import *
import actionlib.action_client
import tf
from math import pi
import arm

import atexit
atexit.register(rospy.signal_shutdown, 'exit')

NODE = 'strike_action'
rospy.init_node(NODE, disable_signals=True)
arm.init()

SPEED = -3.6 / 2.

traj_strike = JointTrajectory()
traj_strike.joint_names = arm.R_JOINTS
traj_strike.points = [
    JointTrajectoryPoint(positions = arm.q_stage,
                         velocities = [0.0]*7,
                         accelerations = [],
                         time_from_start = rospy.Duration(1.0)),
    JointTrajectoryPoint(positions = arm.q_stage,
                         velocities = [0.0]*6 + [SPEED],
                         accelerations = [],
                         time_from_start = rospy.Duration(1.0)),
    JointTrajectoryPoint(positions = arm.q_through,
                         velocities = [0.0]*6 + [SPEED],
                         accelerations = [],
                         time_from_start = rospy.Duration(1.0 + (arm.q_through[6] - arm.q_stage[6])/ SPEED))]


def generate_strike_traj(speed = -1.8):
    state = arm.one_message('/r_arm_controller/state', JointTrajectoryControllerState)
    q = arm.posr()
    q_stage = q[:]
    q_through = q[:]

    # Are we at the end of the strike or at the beginning?
    wrist_up = Vector3Stamped()
    wrist_up.header.frame_id = 'r_wrist_roll_link'
    wrist_up.vector = Vector3(0,0,1)
    wrist_up_in_base = arm.tfl.transformVector3('base_link', wrist_up)


    if abs(wrist_up_in_base.vector.y) < 0.001:  # Flat
        q_stage[6] += arm.WRIST_BACK_OFF
        q_through[6] += arm.WRIST_FORWARD_OFF
        rospy.loginfo("Striking from flat: [%.3f, %.3f]" % (q_stage[6], q_through[6]))
    elif wrist_up_in_base.vector.y >= 0:  # Already forward
        q_stage[6] = q_through[6] + arm.WRIST_BACK_OFF - arm.WRIST_FORWARD_OFF
        rospy.loginfo("Striking from forward [%.3f, %.3f]" % (q_stage[6], q_through[6]))
    else:  # Assume we're backed off
        q_through[6] = q_stage[6] - arm.WRIST_BACK_OFF + arm.WRIST_FORWARD_OFF
        rospy.loginfo("Striking from back [%.3f, %.3f]" % (q_stage[6], q_through[6]))
    print q, q_stage, q_through
    
    traj_strike = JointTrajectory()
    traj_strike.joint_names = arm.R_JOINTS
    traj_strike.points = [
        JointTrajectoryPoint(positions = q_stage,
                             velocities = [0.0]*7,
                             accelerations = [],
                             time_from_start = rospy.Duration(3.0)),
        JointTrajectoryPoint(positions = q_stage,
                             velocities = [0.0]*6 + [speed],
                             accelerations = [],
                             time_from_start = rospy.Duration(3.0)),
        JointTrajectoryPoint(positions = q_through,
                             velocities = [0.0]*6 + [speed],
                             accelerations = [],
                             time_from_start = rospy.Duration(3.0 + (q_through[6] - q_stage[6])/ speed)),
        JointTrajectoryPoint(positions = q_through,
                             velocities = [0.0]*7,
                             accelerations = [],
                             time_from_start = rospy.Duration(3.0 + (q_through[6] - q_stage[6])/ speed))]


    return traj_strike

server = None
def execute_cb(goal):
    try:
        traj = generate_strike_traj()
        arm.send(traj)
        server.set_succeeded()
    except Exception as ex:
        server.set_aborted()
        rospy.logerr("Strike failed:\n" + str(ex))
        

    
if __name__ == '__main__':
    global server
    server = actionlib.simple_action_server.SimpleActionServer(NODE, StrikeAction, execute_cb)
    rospy.spin()

'''

while True:
    arm.send(generate_strike_traj())
    time.sleep(2)
    



traj_strike = JointTrajectory()
traj_strike.joint_names = arm.R_JOINTS
traj_strike.points = [
    JointTrajectoryPoint(positions = q_stage,
                         velocities = [0.0]*7,
                         accelerations = [],
                         time_from_start = rospy.Duration(3.0)),
    JointTrajectoryPoint(positions = q_stage,
                         velocities = [0.0]*6 + [speed],
                         accelerations = [],
                         time_from_start = rospy.Duration(3.0)),
    JointTrajectoryPoint(positions = q_through,
                         velocities = [0.0]*6 + [speed],
                         accelerations = [],
                         time_from_start = rospy.Duration(3.0 + (q_through[6] - q_stage[6])/ speed))]

'''
