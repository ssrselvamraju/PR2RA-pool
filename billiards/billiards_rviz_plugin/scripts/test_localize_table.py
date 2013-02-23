#! /usr/bin/env python

import roslib; roslib.load_manifest('billiard_ball_detector')
import rospy
import actionlib
import billiards_msgs.msg

if __name__ == '__main__':
    rospy.init_node('test_rviz_localize_table')
    rospy.loginfo("client started")
    client = actionlib.SimpleActionClient('rviz_localize_table', billiards_msgs.msg.LocalizeTableAction)
    print 'waiting for server...'
    client.wait_for_server()
    #rospy.Duration(2.0).sleep()
    print 'action_server_up'
    goal = billiards_msgs.msg.GetTableStateGoal()
    client.send_goal(goal)
    print 'goal sent'
    client.wait_for_result()
    result = client.get_result()
    print 'Result:'
    print result
