#!/usr/bin/env python


import roslib; roslib.load_manifest("billiards_checkerboard")

import tf
import rospy
from tf_conversions import posemath as pm
from geometry_msgs.msg import *





class Localizer:
    def __init__(self):

        self.tfl = tf.TransformListener()
        self.sub = rospy.Subscriber("board_pose", PoseStamped, self.callback)


    def callback(self, data):

        data_map = self.tfl.transformPose("/map", data)
        print "Position in map"
        print data_map
        



if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    l = Localizer()
    print "started"
    rospy.spin()

