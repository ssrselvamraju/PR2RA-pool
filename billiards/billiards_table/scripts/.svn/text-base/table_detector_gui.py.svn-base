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
from geometry_msgs.msg import PointStamped, Point
import image_geometry
import message_filters
import numpy
from pr2_controllers_msgs.msg import PointHeadGoal
import sensor_msgs.msg
import stereo_msgs.msg
import tf
from visualization_msgs.msg import Marker

from billiards_msgs.msg import Constants

import table_score

# GUI debugging
import wxversion
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)
import wx
import wx.lib.wxcairo
if 'wxGTK' in wx.PlatformInfo:
    import ctypes
    gdkLib = wx.lib.wxcairo._findGDKLib()
    gdkLib.gdk_cairo_create.restype = ctypes.c_void_p

from rxbag_plugins import image_helper

from tf import transformations

class TableDetectorApp(wx.App):
    def __init__(self):
        wx.App.__init__(self)

    def OnInit(self):
        self._frame = wx.Frame(None, title='TableDetector', pos=wx.DefaultPosition, size=(1280, 960), style=wx.DEFAULT_FRAME_STYLE)
        self._frame.Center()
        self._frame.Show()
        self.SetTopWindow(self._frame)

        self._image_panel0 = ImagePanel(self._frame, -1, size=(640, 480))
        self._image_panel1 = ImagePanel(self._frame, -1, size=(640, 480))
        self._image_panel2 = ImagePanel(self._frame, -1, size=(640, 480))
        self._image_panel3 = ImagePanel(self._frame, -1, size=(640, 480))
        
        self._image_panels = [self._image_panel0, self._image_panel1, self._image_panel2, self._image_panel3]
        
        self._image_panel0.Position = (  0,   0)
        self._image_panel1.Position = (640,   0)
        self._image_panel2.Position = (  0, 480)
        self._image_panel3.Position = (640, 480)

        self._detector = TableDetector(self._image_panels)
        return True

class ImagePanel(wx.Window):
    def __init__(self, *args, **kwargs):
        wx.Window.__init__(self, *args, **kwargs)

        self._image_lock    = threading.Lock()
        self._image         = None
        self._pil_image     = None
        self._image_surface = None

        self.Bind(wx.EVT_PAINT, self._on_paint)
        self.Bind(wx.EVT_IDLE,  self._on_idle)

        self.ClientSize = (640, 480)

    def _get_image(self):
        return self._image
    
    def _set_image(self, im):
        with self._image_lock:
            self._image = im
            wx.CallAfter(self.Refresh)

    image = property(_get_image, _set_image)

    def _on_idle(self, event):
        self.Refresh()
        event.RequestMore()

    def _on_paint(self, event):
        with self._image_lock:
            if self._image is None:
                return

            dc = wx.lib.wxcairo.ContextFromDC(wx.PaintDC(self))

            ix, iy, iw, ih = 0, 0, self.Size[0], self.Size[1]

            self._pil_image     = self._cv_to_pil(self._image)
            self._image_surface = image_helper.pil_to_cairo(self._pil_image.resize((iw, ih), Image.NEAREST))

            dc.set_source_surface(self._image_surface, ix, iy)
            dc.rectangle(ix, iy, iw, ih)
            dc.fill()

    def _cv_to_pil(self, cv_im):
        pil_im = Image.fromstring('L', cv.GetSize(cv_im), cv_im.tostring())
        return pil_im.convert('RGBA')

class TableDetector(threading.Thread):
    def __init__(self, image_panels):
        threading.Thread.__init__(self)

        self._table_detector_server = actionlib.SimpleActionServer('localize_table',                          LocalizeTableAction, self.localize_table_action)
        self._point_head_client     = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        self._point_head_client.wait_for_server()

        self._image_panels = image_panels
        
        self._msg_lock = threading.Lock()
        
        self._left_image = None
        self._disparity  = None
        self._left_info  = None
        self._right_info = None
        
        self._stereo_model = None

        self._cv_bridge = cv_bridge.CvBridge()

        self._tf_listener    = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()

        self._current_estimate = (0.0, 0.0, 0.0)      # initialize estimate to /table_nav frame

        #

        self._narrow_left_image_sub = rospy.Subscriber('/narrow_stereo/left/image_rect',   sensor_msgs.msg.Image,          self._narrow_left_image_cb)
        self._narrow_disparity_sub  = rospy.Subscriber('/narrow_stereo/disparity',         stereo_msgs.msg.DisparityImage, self._narrow_disparity_cb)
        self._narrow_left_info_sub  = rospy.Subscriber('/narrow_stereo/left/camera_info',  sensor_msgs.msg.CameraInfo,     self._narrow_left_info_cb)
        self._narrow_right_info_sub = rospy.Subscriber('/narrow_stereo/right/camera_info', sensor_msgs.msg.CameraInfo,     self._narrow_right_info_cb)

        #self._narrow_left_image_sub = message_filters.Subscriber('/narrow_stereo/left/image_rect',   sensor_msgs.msg.Image)
        #self._narrow_disparity_sub  = message_filters.Subscriber('/narrow_stereo/disparity',         stereo_msgs.msg.DisparityImage)
        #self._narrow_left_info_sub  = message_filters.Subscriber('/narrow_stereo/left/camera_info',  sensor_msgs.msg.CameraInfo)
        #self._narrow_right_info_sub = message_filters.Subscriber('/narrow_stereo/right/camera_info', sensor_msgs.msg.CameraInfo)
        #self._narrow_sync = message_filters.TimeSynchronizer([self._narrow_left_image_sub, self._narrow_left_info_sub, self._narrow_right_info_sub], 10)
        #self._narrow_sync.registerCallback(self._narrow_callback)

        self._markers_pub = rospy.Publisher('visualization_marker', Marker)

        self._broadcast_table_thread = threading.Thread(target=self._broadcast_table, args=(self,))
        self._broadcast_table_thread.setDaemon(True)
        self._broadcast_table_thread.start()

        self.setDaemon(True)
        self.start()

    def _narrow_disparity_cb(self, disparity):   self._disparity  = disparity
    def _narrow_left_image_cb(self, left_image): self._left_image = left_image
    def _narrow_left_info_cb(self, left_info):   self._left_info  = left_info
    def _narrow_right_info_cb(self, right_info): self._right_info = right_info

    def run(self):
        while not rospy.is_shutdown():
            self.detect_table()

            time.sleep(2.0)

    def localize_table_action(self, goal):
        print 'Localizing table...'
        
        corners = [(0.0, 0.0), (0.0, Constants.TABLE_WIDTH), (Constants.TABLE_LENGTH, 0.0), (Constants.TABLE_LENGTH, Constants.TABLE_WIDTH)]
        for corner in corners:
            self.look_at_table_point(corner[0], corner[1], 0.0)

            time.sleep(2.0)
            
            #corner_diamond_positions = self.detect_table(self._left_image, self._left_info, self._right_info, self._disparity)
            
            print '  - found %d diamonds'

        #table_pose = self.estimate_table_pose(self._current_estimate, diamond_positions)

    def look_at_table_point(self, pt):
        point_head_goal = PointHeadGoal()
        point = PointStamped()
        point.header.frame_id = '/table_nav'
        point.point.x = pt[0]
        point.point.y = pt[1]
        point.point.z = pt[2]
        goal.target = point

        self._point_head_client.send_goal(goal)
        self._point_head_client.wait_for_result()

    def detect_diamonds(self, left_image, left_info, right_info, disparity, debug=False):
        # Parameters
        corner_count       = 500                         # number of corners to detect
        min_z              = -0.05                       # min Z coord of diamond in /table_nav frame
        max_z              =  0.05                       # max Z coord of diamond in /table_nav frame
        localization_error = 0.3                         # uncertainty in the /table_nav frame
        max_cluster_size   = 2                           # clusters of corners greater than this are discarded
        diamond_spacing    = Constants.TABLE_WIDTH / 4   # spacing between diamonds
        cluster_dist       = 0.8 * diamond_spacing       # diamonds nearer than this are considered to be a cluster

        # Step 0: initialize the stereo model
        stereo_model = image_geometry.StereoCameraModel()
        stereo_model.fromCameraInfo(left_info, right_info)

        # Step 1a: detect corners in /narrow_stereo/left/image_rect
        left_cv = self._cv_bridge.imgmsg_to_cv(self._left_image, 'passthrough')
        eig_im  = cv.CreateMat(left_cv.rows, left_cv.cols, cv.CV_8UC1)
        temp_im = cv.CreateMat(left_cv.rows, left_cv.cols, cv.CV_8UC1)
        corners = cv.GoodFeaturesToTrack(left_cv, eig_im, temp_im, corner_count, 0.4, 1.0)

        # Step 1b: transform into /narrow_stereo_optical_frame
        disparity_orig = self._cv_bridge.imgmsg_to_cv(self._disparity.image, 'passthrough')
        disparity = cv.CreateMat(disparity_orig.rows, disparity_orig.cols, cv.CV_8UC1)
        cv.Convert(disparity_orig, disparity)
        corners_ns = []
        for corner in corners:
            u, v = int(corner[0]), int(corner[1])
            corner_ns = stereo_model.projectPixelTo3d((u, v), disparity[v, u])
            corners_ns.append(corner_ns)

        # Step 1c: transform into /table_nav frame
        corners_tn = self._transform_points('/narrow_stereo_optical_frame', '/table_nav', corners_ns)

        # Step 1d: discard points outside of Z-bounds
        corners_z_filtered_tn = [(x, y, z) for (x, y, z) in corners_tn if z >= min_z and z <= max_z]

        # Step 1e: discard points outside of rail bounds
        #corners_rail_filtered_tn = self._filter_off_rails(corners_z_filtered_tn, localization_error)
        corners_rail_filtered_tn = corners_z_filtered_tn[:]

        # Step 2: discard clusters of >2 corners
        small_clusters_tn = self._discard_big_clusters(corners_rail_filtered_tn, max_cluster_size, cluster_dist)

        # Step 3: merge clusters of 2 corners, gives { d_0, d_1, d_2, ... , d_m }
        merged_clusters_tn = self._merge_clusters(small_clusters_tn, cluster_dist)

        if debug:
            corners_im = cv.CloneMat(left_cv)
            for i, (x, y) in enumerate(corners):
                x, y = int(x), int(y)
                cv.Rectangle(corners_im, (x-2,y-2), (x+2,y+2), 255, 1)

            self._image_panels[0].image = corners_im
            self._image_panels[1].image = self._render_table_nav_points(left_cv, stereo_model, corners_rail_filtered_tn)
            self._image_panels[2].image = self._render_table_nav_points(left_cv, stereo_model, small_clusters_tn)
            self._image_panels[3].image = self._render_table_nav_points(left_cv, stereo_model, merged_clusters_tn)

        return merged_clusters_tn

    def estimate_table_pose(self, current_pose, diamond_positions, debug=False):
        # Parameters
        x_stdev        = 0.07
        y_stdev        = 0.07
        theta_stdev    = 0.08
        num_hypotheses = 5000
        max_hypothesis_dist_from_table_nav = 0.15

        #

        observed_diamonds = [(x, y) for (x, y, z) in diamond_positions]
        observed_z = sum([z for (x, y, z) in merged_clusters_tn]) / len(merged_clusters_tn)

        # Generate hypothetical poses P = { ( x^(k), y^(k), z^(k) ), ... }
        #hypotheses = table_score.generate_hypotheses(self._current_estimate[0], self._current_estimate[1], self._current_estimate[2], x_stdev, y_stdev, theta_stdev, num_hypotheses)
        hypotheses = table_score.generate_hypotheses(0.0, 0.0, 0.0, x_stdev, y_stdev, theta_stdev, num_hypotheses)

        hypotheses_filtered = [p for p in hypotheses if abs(p[0]) + abs(p[1]) + abs(p[2]) < max_hypothesis_dist_from_table_nav]

        # Find best hypothesis
        (best_hypothesis, best_error) = table_score.find_best_hypothesis(hypotheses_filtered, observed_diamonds)
        print 'Estimated pose: x=%.2f y=%.2f th=%.2f (error=%.6f)' % (best_hypothesis[0], best_hypothesis[1], best_hypothesis[2], best_error)

        if debug:
            # Visualize hypothesis
            for i, candidate in enumerate(hypotheses_filtered):
                if i % 100 != 0:
                    continue
                diamond_set = table_score.computed_hypothesized_diamonds(candidate)
                self._publish_diamonds_to_rviz('diamond_hypothesis', i, [(x, y, observed_z) for (x, y) in diamond_set], '/table_nav', 1, 0, 0, 0.006)
    
            # Visualize observed
            self._publish_diamonds_to_rviz('merged_clusters_tn', 0, merged_clusters_tn, '/table_nav', 0, 1, 0, 0.04)
    
            # Visualize /table_nav
            table_nav_diamonds = [(x, y, 0.0) for (x, y) in table_score.computed_hypothesized_diamonds((0.0, 0.0, 0.0))]
            self._publish_diamonds_to_rviz('table_nav_diamonds', 0, table_nav_diamonds, '/table_nav', 1, 0, 0, 0.04)
    
            # Visualize current_estimate
            current_estimate_diamonds = [(x, y, observed_z) for (x, y) in table_score.computed_hypothesized_diamonds(self._current_estimate)]
            self._publish_diamonds_to_rviz('current_estimate_diamonds', 0, current_estimate_diamonds, '/table_nav', 1, 0, 1, 0.04)

            # Visualize best estimate
            new_estimate_points = table_score.computed_hypothesized_diamonds(new_estimate)
            self._publish_diamonds_to_rviz('new_estimate', 0, [(x, y, observed_z) for (x, y) in new_estimate_points], '/table_nav', 0, 0, 1, 0.04)
    
            # Debugging: visualize table in rviz
            self._publish_table_to_rviz('table', '/table', 0, 0, 1)
            
        return best_hypothesis

    # Broadcast the table frame at 20Hz.
    def _broadcast_table(self, table_detector):
        while not rospy.is_shutdown():
            self._tf_broadcaster.sendTransform((self._broadcast_estimate[0], self._broadcast_estimate[1], -Constants.RAIL_HEIGHT), tf.transformations.quaternion_from_euler(0, 0, -self._broadcast_estimate[2]), rospy.Time.now(), '/table', '/table_nav')
            time.sleep(0.05)

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
            
            self._tf_listener.waitForTransform(frame_id, target_frame_id, rospy.Time.now(), rospy.Duration(5.0))
            p2 = self._tf_listener.transformPoint(target_frame_id, msg).point
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
        m.lifetime = rospy.Duration(30.0)
        self._markers_pub.publish(m)

    def _publish_table_to_rviz(self, ns, frame_id, r, g, b):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
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
        m.lifetime = rospy.Duration(30.0)
        self._markers_pub.publish(m)

if __name__ == '__main__':    
    rospy.init_node('table_detector')
    app = TableDetectorApp()
    app.MainLoop()
