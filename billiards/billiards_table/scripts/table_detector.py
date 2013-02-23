#!/usr/bin/env python

import roslib; roslib.load_manifest('billiards_table')
import rospy

import itertools
import math
import optparse
import sys
import threading
import time

import Image
import numpy
import scipy
import scipy.linalg

import actionlib
import cv
import cv_bridge
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Point32
import image_geometry
import message_filters
import numpy
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
import sensor_msgs.msg
import stereo_msgs.msg
import tf
from visualization_msgs.msg import Marker

from billiards_msgs.msg import Constants, LocalizeTableAction, LocalizeTableResult

import table_score

from tf import transformations 

class TableDetector(object):
    def __init__(self, broadcast):
        self._msg_lock = threading.Lock()
        
        self._left_image = None
        self._disparity  = None
        self._left_info  = None
        self._right_info = None
        
        self._stereo_model = None

        self._cv_bridge = cv_bridge.CvBridge()

        self._tf_listener    = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()

        self._current_estimate   = (0.0, 0.0, 0.0)      # initialize estimate to /table_nav frame
        self._broadcast_estimate = None

        #

        self._narrow_left_image_sub = message_filters.Subscriber('/narrow_stereo/left/image_rect',   sensor_msgs.msg.Image)
        self._narrow_disparity_sub  = message_filters.Subscriber('/narrow_stereo/disparity',         stereo_msgs.msg.DisparityImage)
        self._narrow_left_info_sub  = message_filters.Subscriber('/narrow_stereo/left/camera_info',  sensor_msgs.msg.CameraInfo)
        self._narrow_right_info_sub = message_filters.Subscriber('/narrow_stereo/right/camera_info', sensor_msgs.msg.CameraInfo)
        self._narrow_sync = message_filters.TimeSynchronizer([self._narrow_left_image_sub, self._narrow_disparity_sub, self._narrow_left_info_sub, self._narrow_right_info_sub], 10)
        self._narrow_sync.registerCallback(self._narrow_callback)

        self._markers_pub = rospy.Publisher('visualization_marker', Marker)

        action = 'localize_table'

        print 'Launching %s server...' % action
        self._localize_table_server = actionlib.SimpleActionServer(action, LocalizeTableAction, self.localize_table_action)

        print 'Connecting to point_head_action...'
        self._point_head_client     = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        self._point_head_client.wait_for_server()

        print 'Connected to point_head_action.'

        if rospy.has_param('table_frame_pose_x'):
            x = rospy.get_param('table_frame_pose_x')
            y = rospy.get_param('table_frame_pose_y')
            th = rospy.get_param('table_frame_pose_theta')

            print 'Found table frame pose on the parameter server: %.2f %.2f %.2f' % (x, y, th
)
            self._broadcast_estimate = (x, y, th)
            self._current_estimate = self._broadcast_estimate

        if broadcast:
            print 'Broadcasting table_nav to table tf...'
            self._broadcast_table_thread = threading.Thread(target=self._broadcast_table, args=(self,))
            self._broadcast_table_thread.setDaemon(True)
            self._broadcast_table_thread.start()

    def _narrow_callback(self, left_image, disparity, left_info, right_info):
        self._left_image = left_image
        self._disparity  = disparity
        self._left_info  = left_info
        self._right_info = right_info

    # Broadcast the table frame at 20Hz.
    def _broadcast_table(self, table_detector):
        while not rospy.is_shutdown():
            if self._broadcast_estimate is not None:
                self._tf_broadcaster.sendTransform((self._broadcast_estimate[0], self._broadcast_estimate[1], -Constants.RAIL_HEIGHT), tf.transformations.quaternion_from_euler(0, 0, -self._broadcast_estimate[2]), rospy.Time.now(), '/table', '/table_nav')
                time.sleep(0.05)

    def localize_table_action(self, goal):
        print 'Localizing table...'

        diamond_positions = []

        inset = 0.1

        import numpy

        base_link_tn = self._transform_points('/base_link', '/table_nav', [(0.0, 0.0, 0.0)])[0]
        if base_link_tn[0] > 2.0:
            look_pts = []
            for y in numpy.arange(inset, Constants.TABLE_WIDTH - inset, 0.15):
                look_pts.append((Constants.TABLE_LENGTH - inset, y))
        elif base_link_tn[0] < 0.0:
            look_pts = []
            for y in numpy.arange(inset, Constants.TABLE_WIDTH - inset, 0.15):
                look_pts.append((inset, y))
        elif base_link_tn[1] < 0.0:
            look_pts = []
            for x in numpy.arange(inset, Constants.TABLE_LENGTH - inset, 0.3):
                look_pts.append((x, inset))
        else:
            look_pts = []
            for x in numpy.arange(inset, Constants.TABLE_LENGTH - inset, 0.3):
                look_pts.append((x, Constants.TABLE_WIDTH - inset))

        #look_pts = [(inset,                          inset),
        #            (inset,                          Constants.TABLE_WIDTH / 2),
        #            (inset,                          Constants.TABLE_WIDTH - inset),
        #            (Constants.TABLE_LENGTH / 2,     inset),
        #            (Constants.TABLE_LENGTH / 2,     Constants.TABLE_WIDTH / 2),
        #            (Constants.TABLE_LENGTH / 2,     Constants.TABLE_WIDTH - inset),
        #            (Constants.TABLE_LENGTH - inset, inset),
        #            (Constants.TABLE_LENGTH - inset, Constants.TABLE_WIDTH / 2),
        #            (Constants.TABLE_LENGTH - inset, Constants.TABLE_WIDTH - inset)]

        #for p in numpy.arange(0.0, Constants.TABLE_WIDTH - 2 * inset, 0.3):
        #    look_pts.append((inset, inset + p))
        #for p in numpy.arange(0.0, Constants.TABLE_LENGTH - 2 * inset, 0.3):
        #    look_pts.append((inset + p, inset))

        for i, look_pt in enumerate(look_pts):
            print '  - looking at point %d of %d' % (i + 1, len(look_pts))
            self.look_at_table_point((look_pt[0], look_pt[1], -0.25))
            time.sleep(1)

            print '  - detecting diamonds'
            look_pt_diamond_positions = self.detect_diamonds(self._left_image, self._left_info, self._right_info, self._disparity)
            self._publish_diamonds_to_rviz('all_diamonds_%d' % i, 0, look_pt_diamond_positions, '/table_nav', 0, 1, 0, 0.001)
            time.sleep(0.5)

            if len(look_pt_diamond_positions) > 0:
                self.estimate_table_pose(self._current_estimate, look_pt_diamond_positions)

            # Filter points close to balls
            min_allowable_dist_to_ball = 0.06
            look_pt_diamonds_far_from_balls = []
            for p in look_pt_diamond_positions:
                min_dist = None
                for b in goal.state.balls:
                    bx, by = b.point.point.x, b.point.point.y
                    dist = math.sqrt((p[1] - by)**2 + (p[0] - bx)**2)
                    if min_dist is None or dist < min_dist:
                        min_dist = dist
                if min_dist is None or min_dist > min_allowable_dist_to_ball:
                    look_pt_diamonds_far_from_balls.append(p)
            look_pt_diamond_positions = look_pt_diamonds_far_from_balls

            print '  - found %d diamonds' % len(look_pt_diamond_positions)
            
            diamond_positions.extend(look_pt_diamond_positions)

        self._publish_diamonds_to_rviz('all_diamonds', 0, diamond_positions, '/table_nav', 0, 1, 0, 0.04)

        if len(diamond_positions) == 0:
            self._localize_table_server.set_aborted(None, 'no diamonds found')
        else:
            table_pose = self.estimate_table_pose(self._current_estimate, diamond_positions)

            self._current_estimate   = table_pose
            self._broadcast_estimate = table_pose
        
            rospy.set_param('table_frame_pose_x',     table_pose[0])
            rospy.set_param('table_frame_pose_y',     table_pose[1])
            rospy.set_param('table_frame_pose_theta', table_pose[2])

            result = LocalizeTableResult()
            result.child_frame = '/table'
            result.pose = PoseStamped()
            result.pose.header.stamp = rospy.Time.now()
            result.pose.header.frame_id = '/table_nav'
            result.pose.pose.position.x = table_pose[0]
            result.pose.pose.position.y = table_pose[1]
            result.pose.pose.position.z = -Constants.RAIL_HEIGHT
            q = tf.transformations.quaternion_from_euler(0, 0, -table_pose[2])
            result.pose.pose.orientation.x = q[0]
            result.pose.pose.orientation.y = q[1]
            result.pose.pose.orientation.z = q[2]
            result.pose.pose.orientation.w = q[3]

            print 'Succeeded...'
            self._localize_table_server.set_succeeded(result)

    def look_at_table_point(self, pt):
        point_head_goal = PointHeadGoal()
        point = PointStamped()
        point.header.frame_id = '/table_nav'
        point.point.x = pt[0]
        point.point.y = pt[1]
        point.point.z = pt[2]
        point_head_goal.target = point

        self._point_head_client.send_goal(point_head_goal)
        self._point_head_client.wait_for_result()

    def detect_diamonds(self, left_image, left_info, right_info, disparity):
        # Parameters
        corner_count       = 500                         # number of corners to detect
        min_z              = -0.08                       # min Z coord of diamond in /table_nav frame
        max_z              =  0.02                       # max Z coord of diamond in /table_nav frame
        localization_error = 0.05                         # uncertainty in the /table_nav frame
        max_cluster_size   = 2                           # clusters of corners greater than this are discarded
        diamond_spacing    = Constants.TABLE_WIDTH / 4   # spacing between diamonds
        cluster_dist       = 0.8 * diamond_spacing       # diamonds nearer than this are considered to be a cluster

        # Step 0: initialize the stereo model
        stereo_model = image_geometry.StereoCameraModel()
        stereo_model.fromCameraInfo(left_info, right_info)

        # Step 1a: detect corners in /narrow_stereo/left/image_rect
        left_cv = self._cv_bridge.imgmsg_to_cv(left_image, 'passthrough')
        eig_im  = cv.CreateMat(left_cv.rows, left_cv.cols, cv.CV_8UC1)
        temp_im = cv.CreateMat(left_cv.rows, left_cv.cols, cv.CV_8UC1)
        corners = cv.GoodFeaturesToTrack(left_cv, eig_im, temp_im, corner_count, 0.25, 1.0)

        print ' - found %d corners' % len(corners)
        for c in corners:
            print '   - %d %d' % c

        # Step 1b: transform into /narrow_stereo_optical_frame
        disparity_orig = self._cv_bridge.imgmsg_to_cv(disparity.image, 'passthrough')
        disparity = cv.CreateMat(disparity_orig.rows, disparity_orig.cols, cv.CV_8UC1)
        cv.Convert(disparity_orig, disparity)
        corners_ns = []
        for corner in corners:
            u, v = int(corner[0]), int(corner[1])
            corner_ns = stereo_model.projectPixelTo3d((u, v), disparity[v, u])
            corners_ns.append(corner_ns)

        # Step 1c: transform into /table_nav frame
        corners_tn = self._transform_points('/narrow_stereo_optical_frame', '/table_nav', corners_ns)

        for c in corners_tn:
            print '   - %.2f %.2f %.2f' % c

        self._publish_diamonds_to_rviz('corners', 0, corners_tn, '/table_nav', 1, 0, 0, 0.03)

        # Step 1d: discard points outside of Z-bounds
        corners_z_filtered_tn = [(x, y, z) for (x, y, z) in corners_tn if z >= min_z and z <= max_z]

        print ' - found %d points inside bounds' % len(corners_z_filtered_tn)

        # Step 1e: discard points outside of rail bounds
        corners_rail_filtered_tn = self._filter_off_rails(corners_z_filtered_tn, localization_error)

        # Step 2: discard clusters of >2 corners
        small_clusters_tn = self._discard_big_clusters(corners_rail_filtered_tn, max_cluster_size, cluster_dist)

        print ' - found %d clusters with less than 3 corners' % len(small_clusters_tn)

        # Step 3: merge clusters of 2 corners, gives { d_0, d_1, d_2, ... , d_m }
        merged_clusters_tn = self._merge_clusters(small_clusters_tn, cluster_dist)

        print ' - found %d merged clusters' % len(merged_clusters_tn)

        #self._publish_diamonds_to_rviz('small_clusters',  0, small_clusters_tn,        '/table_nav', 0, 0, 1, 0.04)
        #self._publish_diamonds_to_rviz('merged_clusters', 0, merged_clusters_tn,       '/table_nav', 0, 1, 0, 0.04)

        return merged_clusters_tn

    def estimate_table_pose(self, current_pose, diamond_positions):
        # Parameters
        x_stdev        = 0.75
        y_stdev        = 0.75
        theta_stdev    = 0.13
        num_hypotheses = 12000
        max_hypothesis_dist_from_table_nav = 0.25

        #

        observed_diamonds = [(x, y) for (x, y, z) in diamond_positions]
        observed_z = sum([z for (x, y, z) in diamond_positions]) / len(diamond_positions)

        # Generate hypothetical poses P = { ( x^(k), y^(k), z^(k) ), ... }
        hypotheses = table_score.generate_hypotheses(self._current_estimate[0] * 0.5, self._current_estimate[1] * 0.5, self._current_estimate[2] * 0.5, x_stdev, y_stdev, theta_stdev, num_hypotheses)
        #hypotheses = table_score.generate_hypotheses(self._current_estimate[0], self._current_estimate[1], self._current_estimate[2], x_stdev, y_stdev, theta_stdev, num_hypotheses)
        #hypotheses = table_score.generate_hypotheses(0.0, 0.0, 0.0, x_stdev, y_stdev, theta_stdev, num_hypotheses)

        hypotheses_filtered = [p for p in hypotheses if abs(p[0]) + abs(p[1]) + abs(p[2]) < max_hypothesis_dist_from_table_nav]

        # Find best hypothesis
        (best_hypothesis, best_error) = table_score.find_best_hypothesis(hypotheses_filtered, observed_diamonds)
        print 'Estimated pose: x=%.2f y=%.2f th=%.2f (error=%.6f)' % (best_hypothesis[0], best_hypothesis[1], best_hypothesis[2], best_error)

        # Visualize hypothesis
        for i, candidate in enumerate(hypotheses_filtered):
            if i % 50 != 0:
                continue
            diamond_set = table_score.computed_hypothesized_diamonds(candidate)
            self._publish_diamonds_to_rviz('diamond_hypothesis', i, [(x, y, observed_z) for (x, y) in diamond_set], '/table_nav', 1, 0, 0, 0.006)

        # Visualize observed
        self._publish_diamonds_to_rviz('diamond_positions', 0, diamond_positions, '/table_nav', 0, 1, 0, 0.03)

        # Visualize /table_nav
        #table_nav_diamonds = [(x, y, 0.0) for (x, y) in table_score.computed_hypothesized_diamonds((0.0, 0.0, 0.0))]
        #self._publish_diamonds_to_rviz('table_nav_diamonds', 0, table_nav_diamonds, '/table_nav', 1, 0, 0, 0.04)

        # Visualize current_estimate
        current_estimate_diamonds = [(x, y, observed_z) for (x, y) in table_score.computed_hypothesized_diamonds(self._current_estimate)]
        self._publish_diamonds_to_rviz('current_estimate_diamonds', 0, current_estimate_diamonds, '/table_nav', 1, 0, 1, 0.03)

        # Visualize best estimate
        new_estimate_points = table_score.computed_hypothesized_diamonds(best_hypothesis)
        self._publish_diamonds_to_rviz('new_estimate', 0, [(x, y, observed_z) for (x, y) in new_estimate_points], '/table_nav', 0, 0, 1, 0.03)

        # Debugging: visualize table in rviz
        self._publish_table_to_rviz('table', '/table', 0, 0, 1)

        return best_hypothesis

    def _render_table_nav_points(self, im, stereo_model, pts):
        font = cv.InitFont(cv.CV_FONT_HERSHEY_PLAIN, 1.0, 1.0)

        pts_ns  = self._transform_points('/table_nav', '/narrow_stereo_optical_frame', pts)   # transform back into the /narrow_stereo_optical_frame
        pts_uvs = [stereo_model.project3dToPixel(pt)[0] for pt in pts_ns]                     # /narrow_stereo_optical_frame to pixel coords

        render_im = cv.CloneMat(im)
        for i, (x, y) in enumerate(pts_uvs):
            x, y = int(x), int(y)
            cv.Rectangle(render_im, (x-3,y-3), (x+3,y+3), 255, 1)
            text = '%.2f %.2f %.2f' % pts_ns[i]
            cv.PutText(render_im, text, (x-5, y-10), font, 255)

        return render_im

    def _transform_points(self, frame_id, target_frame_id, pts):
        # Transform points
        try:
            (trans, rot) = self._tf_listener.lookupTransform(frame_id, target_frame_id, rospy.Time())
        except Exception, ex:
            print 'Error transforming points:', str(ex)

        transformed = []
        for pt in pts:
            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            msg.point.x = pt[0]
            msg.point.y = pt[1]
            msg.point.z = pt[2]

            p2 = None
            for i in range(5):
                try:
                    self._tf_listener.waitForTransform(frame_id, target_frame_id, rospy.Time.now(), rospy.Duration(1.0))
                    p2 = self._tf_listener.transformPoint(target_frame_id, msg).point
                    break
                except Exception, ex:
                    print >> sys.stderr, 'Error transforming points: %s. Trying again.' % str(ex)

            if p2 is not None:
                transformed.append((p2.x, p2.y, p2.z))

        return transformed

    def _filter_off_rails(self, pts, localization_error):
        out_min_x = -Constants.RAIL_DEPTH - localization_error
        out_min_y = -Constants.RAIL_DEPTH - localization_error
        out_max_x = Constants.TABLE_LENGTH + Constants.RAIL_DEPTH + localization_error
        out_max_y = Constants.TABLE_WIDTH  + Constants.RAIL_DEPTH + localization_error
        in_min_x  = localization_error
        in_min_y  = localization_error
        in_max_x  = Constants.TABLE_LENGTH - localization_error
        in_max_y  = Constants.TABLE_WIDTH  - localization_error
        out_sw_tn = (out_min_x, out_min_y, 0.0)
        out_nw_tn = (out_min_x, out_max_y, 0.0)
        out_se_tn = (out_max_x, out_min_y, 0.0)
        out_ne_tn = (out_max_x, out_max_y, 0.0)
        in_sw_tn  = (in_min_x,  in_min_y,  0.0)
        in_nw_tn  = (in_min_x,  in_max_y,  0.0)
        in_se_tn  = (in_max_x,  in_min_y,  0.0)
        in_ne_tn  = (in_max_x,  in_max_y,  0.0)
        sw_tn     = (-Constants.RAIL_DEPTH / 2, -Constants.RAIL_DEPTH / 2,                        0.0)
        nw_tn     = (-Constants.RAIL_DEPTH / 2, Constants.TABLE_WIDTH + Constants.RAIL_DEPTH / 2, 0.0)
        se_tn     = (Constants.TABLE_LENGTH + Constants.RAIL_DEPTH / 2, -Constants.RAIL_DEPTH / 2,                        0.0)
        ne_tn     = (Constants.TABLE_LENGTH + Constants.RAIL_DEPTH / 2, Constants.TABLE_WIDTH + Constants.RAIL_DEPTH / 2, 0.0)

        rail_s, rail_w, rail_n, rail_e = [], [], [], []
        for (x, y, z) in pts:
            if (x >= out_min_x and x <= in_min_x and y >= in_min_y and y <= in_max_y):
                rail_w.append((x, y, z)) # west rail
            elif (x >= in_max_x and x <= out_max_x and y >= in_min_y and y <= in_max_y):
                rail_e.append((x, y, z)) # east rail
            elif (x >= in_min_x and x <= in_max_x and y >= in_max_y and y <= out_max_y):
                rail_n.append((x, y, z)) # north rail
            elif (x >= in_min_x and x <= in_max_x and y >= out_min_y and y <= in_min_y):
                rail_s.append((x, y, z)) # south rail

        filtered_pts = []
        for pt in itertools.chain(rail_s, rail_w, rail_n, rail_e):
            filtered_pts.append(pt)

        return filtered_pts

    def _discard_big_clusters(self, pts, max_cluster_size, cluster_dist):
        filtered_pts = []
        
        for i in range(len(pts)):
            cluster_size = 0
            x1, y1, z1 = pts[i]
            for j in range(len(pts)):
                if j == i:
                    continue
                
                x2, y2, z2 = pts[j]

                dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
                if dist < cluster_dist:
                    cluster_size += 1

            if cluster_size <= max_cluster_size:
                filtered_pts.append((x1, y1, z1))

        return filtered_pts

    def _merge_clusters(self, pts, cluster_dist):
        merged_pts = []
        
        skip = set()
        for i in range(len(pts)):
            if i in skip:
                continue

            x1, y1, z1 = pts[i]
            merged = False
            for j in range(i + 1, len(pts)):
                if j in skip:
                    continue

                x2, y2, z2 = pts[j]

                dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
                if dist < cluster_dist:
                    c = ((x1 + x2) * 0.5, (y1 + y2) * 0.5, (z1 + z2) * 0.5)
                    merged_pts.append(c)
                    skip.add(j)
                    merged = True
                    break

            if not merged:
                merged_pts.append((x1, y1, z1))
                
        return merged_pts

    def _publish_diamonds_to_rviz(self, ns, marker_id, pts, frame_id, r, g, b, radius):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = 'billiard_table_markers_' + ns
        m.type = Marker.SPHERE_LIST
        m.id = marker_id
        m.action = Marker.ADD

        for pt in pts:
            pose = Point()
            pose.x = pt[0]
            pose.y = pt[1]
            pose.z = pt[2]
            m.points.append(pose)

        m.scale.x = radius
        m.scale.y = radius
        m.scale.z = radius
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        m.lifetime = rospy.Duration(90.0)
        self._markers_pub.publish(m)

    def _publish_table_to_rviz(self, ns, frame_id, r, g, b):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now() + rospy.Duration(2)
        m.ns = 'billiard_table_markers_' + ns
        m.type = Marker.CUBE
        m.id = 0
        m.action = Marker.ADD

        m.pose.position.x = Constants.TABLE_LENGTH / 2
        m.pose.position.y = Constants.TABLE_WIDTH / 2
        m.pose.position.z = 0.0
        m.scale.x = Constants.TABLE_LENGTH
        m.scale.y = Constants.TABLE_WIDTH
        m.scale.z = 0.01
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.2
        m.lifetime = rospy.Duration(360.0)
        self._markers_pub.publish(m)

if __name__ == '__main__':
    broadcast = '--broadcast' in sys.argv

    rospy.init_node('table_detector')
    table_detector = TableDetector(broadcast)
    rospy.spin()
