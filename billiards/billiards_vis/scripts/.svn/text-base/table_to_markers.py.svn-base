#!/usr/bin/env python

import roslib; roslib.load_manifest('billiards_vis')
import rospy

from billiards_msgs.msg import TableState, BallState, Constants
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TableToMarkers(object):
    def __init__(self):
        self._in_sub = rospy.Subscriber("in", TableState, self.callback)
        self._out_pub = rospy.Publisher("out", Marker)
    
    def callback(self, state):
        count = 0
        for ball in state.balls:
            m = Marker()
            m.header = ball.point.header
            m.ns = "billiards_table_state"
            m.id = count
            m.action = Marker.ADD
            m.type = Marker.SPHERE
            m.pose.position.x = ball.point.point.x
            m.pose.position.y = ball.point.point.y
            #m.pose.position.z = Constants.BALL_RADIUS
            m.pose.position.z = ball.point.point.z
            m.pose.orientation.w = 1.0
            if (ball.id == 0):
                m.color.r = 1.0
                m.color.g = 1.0
                m.color.b = 1.0
            elif (ball.id < 8):
                m.color.r = 1.0
            elif (ball.id == 8):
                pass
            else:
                m.color.r = 1.0
                m.color.g = 1.0
            
            m.color.a = 0.7
                
            if (ball.pocketed):
                m.color.g = 1.0
                m.color.r = 0.0
                m.color.b = 0.0
            
            m.scale.x = m.scale.y = m.scale.z = Constants.BALL_RADIUS * 2
            self._out_pub.publish(m)
            
            m.id = m.id + 1000
            m.type = Marker.TEXT_VIEW_FACING
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.text = str(ball.id)
            m.pose.position.z = ball.point.point.z + 0.2
            m.scale.x = m.scale.y = m.scale.z = 0.2
            self._out_pub.publish(m)
            
            count += 1
            
        for id in xrange(count, 16):
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "table"
            m.ns = "billiards_table_state"
            m.id = id
            m.action = Marker.DELETE
            self._out_pub.publish(m)
            m.id = id + 1000
            self._out_pub.publish(m)
            
        # send the rails
        m = Marker()
        m.header.frame_id = "table"
        m.header.stamp = rospy.Time.now()
        m.ns = "billiards_table_state"
        m.id = 10000
        m.action = Marker.ADD
        m.type = Marker.LINE_STRIP
        m.pose.orientation.w = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.scale.x = 0.05
        p = Point(x = 0.0, y = 0.0, z = 0.0)
        m.points.append(p)
        p = Point(x = Constants.TABLE_LENGTH, y = 0.0, z = 0.0)
        m.points.append(p)
        p = Point(x = Constants.TABLE_LENGTH, y = Constants.TABLE_WIDTH, z = 0.0)
        m.points.append(p)
        p = Point(x = 0, y = Constants.TABLE_WIDTH, z = 0.0)
        m.points.append(p)
        p = Point(x = 0, y = 0, z = 0.0)
        m.points.append(p)
        self._out_pub.publish(m)

if __name__ == "__main__":
    rospy.init_node("table_to_markers")
    
    t = TableToMarkers()
    
    rospy.spin()
