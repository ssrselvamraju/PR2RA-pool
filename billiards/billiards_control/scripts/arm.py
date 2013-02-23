#! /usr/bin/env python

import roslib
roslib.load_manifest('billiards_control')

import time
import rospy
import tf
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from actionlib_msgs.msg import *
from kinematics_msgs.msg import *
from kinematics_msgs.srv import *
from geometry_msgs.msg import *
import actionlib.action_client
from math import pi, sin, cos
import tf_conversions.posemath as pm

R_JOINTS = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
            'r_elbow_flex_joint', 'r_forearm_roll_joint',
            'r_wrist_flex_joint', 'r_wrist_roll_joint']
L_JOINTS = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
            'l_elbow_flex_joint', 'l_forearm_roll_joint',
            'l_wrist_flex_joint', 'l_wrist_roll_joint']

tfl = None
r_arm_client = None
l_arm_client = None
get_ik_r_service = None
get_ik_l_service = None

def one_message(topic, Msg):
    msg = []
    def cb(m):
        if not msg:
            msg.append(m)
    sub = rospy.Subscriber(topic, Msg, cb)
    try:
        while not msg:
            time.sleep(0.002)
        return msg[0]
    finally:
        sub.unregister()

def posr():
    state = one_message('/r_arm_controller/state', JointTrajectoryControllerState)
    return list(state.desired.positions)
def posl():
    state = one_message('/l_arm_controller/state', JointTrajectoryControllerState)
    return list(state.desired.positions)

def init():
    global tfl, r_arm_client, l_arm_client, get_ik_r_service, get_ik_l_service
    if not tfl:
        tfl = tf.TransformListener()
        r_arm_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action',
                                                    JointTrajectoryAction)
        l_arm_client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action',
                                                    JointTrajectoryAction)
        get_ik_r_service = rospy.ServiceProxy('/pr2_right_arm_kinematics/get_ik', GetPositionIK)
        get_ik_l_service = rospy.ServiceProxy('/pr2_left_arm_kinematics/get_ik', GetPositionIK)
  
def send_to_client(t, client):
    g = JointTrajectoryGoal()
    g.trajectory = t
    try:
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    if client.get_state() == GoalStatus.SUCCEEDED:
        print "Success"
        return True
    else:
        print "Failure"
        return False
def send(t):
    if 'l_shoulder_pan_joint' in t.joint_names:
        return send_to_client(t, l_arm_client)
    elif 'r_shoulder_pan_joint' in t.joint_names:
        return send_to_client(t, r_arm_client)
    else:
        print "Don't know which client to send to"


def gotol(qs):
    traj = JointTrajectory()
    traj.joint_names = L_JOINTS
    traj.points = [JointTrajectoryPoint(positions = qs,
                                        velocities = [0.0]*7,
                                        time_from_start = rospy.Duration(4.0))]
    return send(traj)
    
def gotor(qs):
    traj = JointTrajectory()
    traj.joint_names = R_JOINTS
    traj.points = [JointTrajectoryPoint(positions = qs,
                                        velocities = [0.0]*7,
                                        time_from_start = rospy.Duration(4.0))]
    return send(traj)



def validate_quat(q):
    if q.x == q.y == q.z == q.w == 0.0:
        q.w = 1.0

def PS(frame, x, y, z, qx=0, qy=0, qz=0, qw=1):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position = Point(x,y,z)
    ps.pose.orientation = Quaternion(qx,qy,qz,qw)
    return ps
        
def get_ik_simple(service, goal, seed, joint_names, tip_frame, tool=None):
    validate_quat(goal.pose.orientation)
    
    # Transforms the goal due to the tip frame
    if tool:
        tool_in_tip = tfl.transformPose(tip_frame, tool)
        tool_in_tip_t = pm.fromMsg(tool_in_tip.pose)
        goal_t = pm.fromMsg(goal.pose)
        #goal.pose = pm.toMsg(tool_in_tip_t.Inverse() * goal_t)
        #goal = PoseStamped(header = goal.header, pose = pm.toMsg(tool_in_tip_t.Inverse() * goal_t))
        goal = PoseStamped(header = goal.header, pose = pm.toMsg(goal_t * tool_in_tip_t.Inverse()))

    req = PositionIKRequest()
    req.ik_link_name = tip_frame
    req.pose_stamped = goal
    req.ik_seed_state.joint_state.name = joint_names
    req.ik_seed_state.joint_state.position = seed

    error_code = 0
    for i in range(10):
        resp = service(req, rospy.Duration(10.0))
        if resp.error_code.val != 1:
            rospy.logerr("Error in IK: %d" % resp.error_code.val)
        else:
            return list(resp.solution.joint_state.position)
    return []


def ik_r(goal, seed, tool = None):
    resp = get_ik_simple(get_ik_r_service, goal,
                         seed, R_JOINTS,
                         'r_wrist_roll_link', tool)
    return resp
    return resp.solution.joint_state.position

def ik_l(goal, seed, tool = None):
    resp = get_ik_simple(get_ik_l_service, goal,
                         seed, L_JOINTS,
                         'l_wrist_roll_link', tool)
    return resp
    return resp.solution.joint_state.position

def fromRaw(x, y, z, qx, qy, qz, qw):
    p = Pose(Point(x,y,z), Quaternion(qx,qy,qz,qw))
    return pm.fromMsg(p)


BRIDGE = [1.15499, 0.01531, 2.62625, -0.14865, -5.76226, -1.47351, -2.187361]
BRIDGE_PUSH = [1.15499, 0.02031, 2.62625, -0.14865, -5.76226, -1.47351, -2.187361]
BRIDGE_UP = [1.15499, -0.30, 2.62625, -0.14865, -5.76226, -1.47351, -2.187361]

STRIKE = [-1.24669, 1.2772, -2.5141, -0.65491, -4.96495, -1.21409, -0.65823]
WRIST_BACK = -0.21492273102985748
WRIST_FORWARD = -0.90384297188683493


STRIKE = [-0.46788174130538751, 1.0167579575097525, -2.5944045783888496, -1.2225998194625165, 8.4406507647638396, -1.769841790609731, -0.72584855829847428]

q_stage = STRIKE[:]
q_stage[6] = WRIST_BACK
q_through = STRIKE[:]
q_through[6] = WRIST_FORWARD


WRIST_FORWARD = -0.60384297188683493

# Moving to offsets
WRIST_BACK_OFF = 0.4
WRIST_FORWARD_OFF = -0.3

LIFTED_L = [1.15499, -0.300, 2.626, -0.1486, 0.5209, -1.473, 4.0958]
LIFTED_R = [-0.5140, 1.0396, -2.594, -1.3428, 8.4215, -1.8501, -0.8748]

SWING_L = [
    LIFTED_L,
    [0.4372, -0.3332, 2.0, -0.1492, 0.4228, -1.5079, 3.8166],
    [-0.3600, -0.3362, 1.9, -0.8520, 1.2846, -1.0702, 2.7683]
]
SWING_R = [
    LIFTED_R,
    [-1.0892, 0.7472, -2.4318, -0.9058, 8.3842, -0.9088, -0.3284],
    [-1.7548, 0.7786, -2.4824, -0.3570, 8.3095, -0.8363, 0.12792]
]


'''

import roslib
roslib.load_manifest('billiards_control')
import time
import rospy
rospy.init_node('temp', disable_signals=True, anonymous=True)
%run arm.py
init()


bpd = PS('base_link', 0.313, 0.955, 0.833,  0.019, -0.056, 0.749, 0.660)
bpd = PS('base_link', 0.313, 0.855, 0.833,  0.019, -0.056, 0.749, 0.660)
bf = PoseStamped()
bf.pose.orientation.w = 1
bf.header.frame_id = 'bridge_frame'

ik_l(bpd, BRIDGE, bf)



bpd = PS('base_link', 0.7, 0.4, 0.8,  0,0,0,1)


f = PoseStamped()
f.header.frame_id = 'l_wrist_roll_link'
f.pose.orientation.w = 1.0
f.pose.position.x = 0.0
ik_l(bpd, [0]*7, f)






goal = PS('base_link', 0.301, 0.925, 1.102,  -0.007, 0.686, 0.082, 0.723) # of the l wrist roll
goal = PS('base_link', 0.301, 0.825, 1.102,  -0.007, 0.686, 0.082, 0.723) # of the l wrist roll
f = PoseStamped()
f.header.frame_id = 'l_wrist_roll_link'
f.pose.orientation.w = 1.0
ik_l(goal, [1.15499, -0.1499, 2.6262, -0.14865, 0.52085, -1.4734, 4.095], None)


Let's try moving the bridge about a little bit:

bgoal = PS('base_link', 0.7, 0.0, 0.8,  0,0,0,1)
f = PoseStamped()
f.header.frame_id = 'bridge_frame'#'l_wrist_roll_link'
f.pose.orientation.w = 1.0
ik_l(bgoal, [1.15499, -0.1499, 2.6262, -0.14865, 0.52085, -1.4734, 4.095], f)




# Moving the bridge around in the table frame

bgoal = PS('table', 0.22, 0.56, 0.0,  0,0,0,1)
f = PoseStamped()
f.header.frame_id = 'bridge_frame'
f.pose.orientation.w = 1.0
ik_l(bgoal, BRIDGE, f)





bgoal = PS('base_link', 0.295, 0.914, 0.714,  -0.006, 0.023, 0.750, 0.661)


# New (rotated) bridge pose
TH = 90 * pi/180.
bgoal = PS('base_link', 0.285, 0.800, 0.714,  0, 0, sin(TH/2), cos(TH/2))
f = PoseStamped()
f.header.frame_id = 'bridge_frame'#'l_wrist_roll_link'
f.pose.orientation.w = 1.0
ik_l(bgoal, [1.15499, -0.1499, 2.6262, -0.14865, 0.52085, -1.4734, 4.095], f)




# Moving the right arm around in the bridge frame
rgoal = PS('bridge_frame', -1.236, 0.260, -0.250,  0, 0, -0.7071, 0.7071)
ik_l(rgoal, STRIKE)

rgoal = PS('bridge_frame', -1.0, 0.17, -0.300,  0, 0, -0.7071, 0.7071)
ik_r(rgoal, STRIKE)



rgoal = PS('bridge_frame', -1.0, 0.17, -0.280,  0, 0, -0.7071, 0.7071)
ik_r(rgoal, STRIKE)



'''

