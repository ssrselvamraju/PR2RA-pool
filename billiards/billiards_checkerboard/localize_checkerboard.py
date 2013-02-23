#!/usr/bin/env python


import roslib; roslib.load_manifest("billiards_checkerboard")

import tf
import rospy
import math
import threading
import time
from tf_conversions import posemath as pm
from geometry_msgs.msg import *


cb0 = pm.fromMsg(Pose(Point(0.211288259977,
                             1.1586949635,
                             0.780193785588),
                      Quaternion(0.693575488633,
                                 0.720086872142,
                                 0.00685370752406,
                                 -0.0195183203427)))



cb1 = pm.fromMsg(Pose(Point(-0.829222503346,
                             0.91628121813,
                             0.75470019588),
                      Quaternion(0.666639100989,
                                 0.743640332462,
                                 0.0509050054482,
                                 -0.000213045973996)))





cb2 = pm.fromMsg(Pose(Point(-1.08121716871,
                            2.93264031484,
                            0.715232616605),
                      Quaternion(0.693575488633,#stolen form cb0 for right orientation
                                 0.720086872142,
                                 0.00685370752406,
                                 -0.0195183203427))) 
"""
                      Quaternion(0.784677879852,
                                 -0.606350719965,
                                 -0.120554439733,
                                 0.0456733656571)))
"""
cb3 = pm.fromMsg(Pose(Point(0.316599758072,
                            2.86272678925,
                            0.757206296245),
                      Quaternion(0.693575488633,#stolen form cb0 for right orientation
                                 0.720086872142,
                                 0.00685370752406,
                                 -0.0195183203427))) 
"""
old needed to be rotated
                      Quaternion(0.676953165258,
                                 0.735986939112,
                                 0.00755496547167,
                                 0.000748332203636)))
"""                 

checkerboards = [cb0, cb1, cb2, cb3]


rotation_correction = pm.fromMsg(Pose(Point(0,0,0),Quaternion(0,0,1,0)))

class Broadcaster:
    def __init__(self, parent, child):
        self.running = False
        self.tfb = tf.TransformBroadcaster()
        self.transform = None
        self.parent = parent
        self.child = child
        
    def start(self):
        self.running = True
        thread = threading.Thread(target = self._run)
        thread.daemon = True
        thread.start()

    def stop(self):
        self.running = False

    def _run(self):
        while self.running:
            if self.transform:
                self.tfb.sendTransform(self.transform[0], self.transform[1], rospy.Time.now(), self.child, self.parent)
            time.sleep(0.01)

class Localizer:
    def __init__(self):

        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber("board_pose", PoseStamped, self.callback)
        self.sub = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose_cb)
        self.broad = Broadcaster('/map', '/odom_combined')
        self.broad.start()


    def callback(self, data):

        self.tfl.waitForTransform("/map", data.header.frame_id, data.header.stamp, rospy.Duration().from_sec(1.0))
        map_pose = self.tfl.transformPose("/map", data)
        cb_in_map = pm.fromMsg(map_pose.pose)
        print "CB IN MAP\n", cb_in_map

        # correct for ambiguous checkerboard positions
        eulers = cb_in_map.M.GetEulerZYX()
        print eulers
        reversing = False
        if math.cos(eulers[0] - math.pi/2) < 0:
            cb_in_map.M =  rotation_correction.M * cb_in_map.M 
            print "reversing", cb_in_map
            reversing = True
            
        

        

        
        best_distance = 10000000
        best_cb = checkerboards[0]

        for cb in checkerboards:
            distance = (cb_in_map.p - cb.p).Norm()
            if distance < best_distance:
                best_distance = distance
                best_cb = cb
        print "closest checkerboard %d"%checkerboards.index(best_cb)



        # compute cb to footprint
        data_out = self.tfl.transformPose("/base_footprint", data)
        #print "now", data_out
        cb_bl = pm.fromMsg(data_out.pose)
        if reversing:
            cb_bl.M = cb_bl.M * rotation_correction.M

        #compute base pose from known tranform and measured one
        base_pose = best_cb * cb_bl.Inverse()        
        base_pose_msg = pm.toMsg(base_pose)
        print "Base Pose "
        print base_pose_msg
        
        self.compute_net_transform(base_pose)



    def compute_net_transform(self, base_pose):

        base_to_odom = self.tfl.lookupTransform("/base_footprint", "/odom_combined", rospy.Time())

        bto = pm.fromTf(base_to_odom)
        

        net_t = base_pose * bto

        print "Setting "
        print pm.toMsg(net_t)
        self.broad.transform = pm.toTf(net_t)
        
        
    def initial_pose_cb(self, data):
        base_pose = pm.fromMsg(data.pose.pose)
        self.compute_net_transform(base_pose)


if __name__ == '__main__':
    rospy.init_node('checkerboard_localization', anonymous=True)
    l = Localizer()
    print "started"
    rospy.spin()

