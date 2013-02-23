#!/usr/bin/env python

import roslib; roslib.load_manifest('billiards_vis')
import rospy

from billiards_msgs.msg import ShotPlan, Constants
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ShotPlanToMarkers(object):
    def __init__(self):
        self._in_sub = rospy.Subscriber("in", ShotPlan, self.callback)
        self._out_pub = rospy.Publisher("out", Marker)
        self._id = 0
    
    def create_arrow(self, pose_stamped):
        m = Marker()
        m.header = pose_stamped.header
        m.ns = "billiards_shot_plan"
        m.id = self._id
        m.action = Marker.ADD
        m.type = Marker.ARROW
        m.pose = pose_stamped.pose
        m.scale.x = 0.5
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.a = 1.0
        self._id += 1
        return m
    
    def callback(self, state):
        self._id = 0
        
        m = self.create_arrow(state.base_pose)
        m.color.r = 1.0
        self._out_pub.publish(m)
        
        m = self.create_arrow(state.bridge_pose)
        m.color.g = 1.0
        m.scale.x = Constants.BRIDGE_TO_STRIKE_MIN
        self._out_pub.publish(m)
        
        m = Marker()
        m.header = state.bridge_pose.header
        m.ns = "billiards_shot_plan"
        m.id = self._id
        m.action = Marker.ADD
        m.type = Marker.CUBE
        m.pose = state.bridge_pose.pose
        m.pose.position.z += 0.055
        m.scale.x = 0.005
        m.scale.y = 0.05
        m.scale.z = 0.11
        m.color.a = 1.0
        m.color.b = 1.0
        self._out_pub.publish(m)
        
        self._id += 1
        

if __name__ == "__main__":
    rospy.init_node("shot_plan_to_markers")
    
    t = ShotPlanToMarkers()
    
    rospy.spin()

