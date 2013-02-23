#! /usr/bin/env python

import roslib; roslib.load_manifest('billiard_ball_detector')
import rospy
import actionlib
import billiards_msgs.msg

if __name__ == '__main__':
    rospy.init_node('ball_detector_client')
    rospy.loginfo("client started")
    client = actionlib.SimpleActionClient('vision_get_table_state', billiards_msgs.msg.GetTableStateAction)
    print 'waiting for server...'
    client.wait_for_server()
    #rospy.Duration(2.0).sleep()
    print 'action_server_up'
    goal = billiards_msgs.msg.GetTableStateGoal()
    if 0:
        goal.table_frame_id = '/table_nav'
        goal.filter_against_table = False
    else:
        goal.table_frame_id = '/table'
        goal.filter_against_table = True
    client.send_goal(goal)
    print 'goal sent'
    client.wait_for_result()
    result = client.get_result()
    print 'Result:'
    print result
