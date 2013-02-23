#!/usr/bin/env python

import roslib; roslib.load_manifest('billiards_vis')
import rospy
import time

from billiards_msgs.msg import ShotPlan

if __name__ == "__main__":
    rospy.init_node("test_shot_plan")
    
    out_pub = rospy.Publisher("shot_plan", ShotPlan)
    
    while not rospy.is_shutdown():
        sp = ShotPlan()
        sp.base_pose.header.frame_id = "table"
        sp.base_pose.header.stamp = rospy.Time.now()
        sp.base_pose.pose.position.x = -1.0
        sp.base_pose.pose.position.y = -1.0
        sp.base_pose.pose.position.z = -1.0
        sp.base_pose.pose.orientation.w = 1.0
        
        sp.bridge_pose.header.frame_id = "table"
        sp.bridge_pose.header.stamp = rospy.Time.now()
        sp.bridge_pose.pose.position.x = 0.5
        sp.bridge_pose.pose.position.y = 0.5
        sp.bridge_pose.pose.orientation.w = 1.0
        
        out_pub.publish(sp)
        
        time.sleep(0.1)