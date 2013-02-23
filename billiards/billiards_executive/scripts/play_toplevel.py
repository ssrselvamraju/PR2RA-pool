#! /usr/bin/env python

# The nominal sequence for a single shot is:
#   lift bridge
#   move to one end of the table
#   swing arms out of the way
#   acquire table state (starts publishing new table frame)
#   acquire ball state (returns TableState)
#   plan shot, given TableState (returns ShotPlan)
#   swing arms back into shooting position
#   move base to ShotPlan.base_pose
#   swing arms out of the way
#   acquire table state (starts publishing new table frame)
#   acquire ball state (returns TableState)
#   plan shot, given TableState and limited angular search range (returns
#   ShotPlan)
#   move arms to satisfy ShotPlan.bridge_pose
#   shoot
#   lift bridge

# Assumed to be running all the time:
#   gross localization (amcl), publishes table_nav frame

import roslib
roslib.load_manifest('billiards_executive')

import rospy
import math
import actionlib
import tf.transformations
from smach import *

from actionlib_msgs.msg import *
from billiards_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from pr2_controllers_msgs.msg import *

# HACK: this is here so that we can hardcode poses for the head and foot of
# the table.
MOVE_BASE_FIXED_FRAME = 'old_table_nav'
POINT_HEAD_FIXED_FRAME = 'table_nav'
TABLE_FRAME = 'table'

def print_result(userdata, status, result):
    print "RESULT:", result

def extract_planned_bridge_pose(ud, goal):
    return PlaceBridgeGoal(down = True,
                           pose = ud.planned_shot.bridge_pose)

def plan_to_move_base(ud, goal):
    goal.target_pose = ud.planned_shot.base_pose
    return goal

def plan_to_move_base_head_table(ud, goal):
    g = MoveBaseGoal()
    # HACK: should figure out where to go based on detected table position.
    #       For now, we hardcode particular poses in the roadmap as known
    #       ends of the table.
    g.target_pose.header.frame_id = MOVE_BASE_FIXED_FRAME
    g.target_pose.pose.position.x = 1.426641
    g.target_pose.pose.position.y = 0.056116
    g.target_pose.pose.position.z = 0.002238
    g.target_pose.pose.orientation.x = -0.000671
    g.target_pose.pose.orientation.y = 0.001071
    g.target_pose.pose.orientation.z = -0.709349
    g.target_pose.pose.orientation.w = 0.704856
    return g

def plan_to_move_base_head_table_for_observation(ud, goal):
    g = MoveBaseGoal()
    # HACK: should figure out where to go based on detected table position.
    #       For now, we hardcode particular poses in the roadmap as known
    #       ends of the table.
    g.target_pose.header.frame_id = MOVE_BASE_FIXED_FRAME
    g.target_pose.pose.position.x = 0.9
    g.target_pose.pose.position.y = 0.047
    g.target_pose.pose.position.z = 0.002238
    g.target_pose.pose.orientation.x = -0.000671
    g.target_pose.pose.orientation.y = 0.001071
    g.target_pose.pose.orientation.z = -0.709349
    g.target_pose.pose.orientation.w = 0.704856
    return g


def plan_to_move_base_foot_table(ud, goal):
    g = MoveBaseGoal()
    # HACK: should figure out where to go based on detected table position.
    #       For now, we hardcode particular poses in the roadmap as known
    #       ends of the table.
    g.target_pose.header.frame_id = MOVE_BASE_FIXED_FRAME
    g.target_pose.pose.position.x = 5.075683
    g.target_pose.pose.position.y = 0.315661
    g.target_pose.pose.position.z = 0.003264
    g.target_pose.pose.orientation.x = 0.000451
    g.target_pose.pose.orientation.y = -0.000207
    g.target_pose.pose.orientation.z = 0.743349
    g.target_pose.pose.orientation.w = 0.668904
    return g

def point_head_at_middle_of_table(ud, goal):
    g = PointHeadGoal()
    g.target.header.frame_id = POINT_HEAD_FIXED_FRAME
    g.target.point.x = 0.3
    g.target.point.y = 0.6
    return g

def point_head_at_cue_ball(ud, goal):
    # Find the cue ball
    for b in ud.observed_table_state.balls:
        if b.id == 0:
            g = PointHeadGoal()
            g.target = b.point
            g.target.header.stamp = rospy.Time(0.0)
            return g
    # What's the right way to indicate failure here?
    return None

def plan_to_plan_shot_unlimited(ud, goal):
    goal.state = ud.observed_table_state
    goal.angle_min = 0
    goal.angle_max = 2*math.pi
    return goal

def plan_to_plan_shot_limited(ud, goal):
    # HACK: use the old table state, except for the cue ball position
#    goal.state = ud.observed_table_state
    goal.state = ud.new_observed_table_state
#    new_cue_ball = None
#    for b in ud.new_observed_table_state.balls:
#        if b.id == 0:
#            new_cue_ball = b
#    if new_cue_ball is None:
#        rospy.logwarn("Failed to find cue ball on second observation; passing previous state in its entirety.")
#    else:
#        for i in range(0,len(goal.state.balls)):
#            if goal.state.balls[i].id == 0:
#                goal.state.balls[i] = new_cue_ball

    # HACK: should figure out reasonable angle limits based on the current
    #       state of the world.  For now, we hardcode a small range.
    half_angle = 0.2 # radians
    q = (ud.planned_shot.bridge_pose.pose.orientation.x,
         ud.planned_shot.bridge_pose.pose.orientation.y,
         ud.planned_shot.bridge_pose.pose.orientation.z,
         ud.planned_shot.bridge_pose.pose.orientation.w)
    yaw  = tf.transformations.euler_from_quaternion(q)[2]
    goal.angle_min = yaw - half_angle
    goal.angle_max = yaw + half_angle
    # Normalize angles
    goal.angle_min = math.atan2(math.sin(goal.angle_min), math.cos(goal.angle_min))
    goal.angle_max = math.atan2(math.sin(goal.angle_max), math.cos(goal.angle_max))
    print 'Munged state sent to planner on second call:'
    print goal
    return goal

def main(argv):
    USE_RVIZ = False
    if len(argv) > 1 and sys.argv[1] == 'rviz':
        USE_RVIZ = True

    rospy.init_node('play_toplevel')

    sm = StateMachine(outcomes = ['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.add('LIFT_TORSO',
                         SimpleActionState('/torso_controller/position_joint_action', SingleJointPositionAction,
                                           goal = SingleJointPositionGoal(position = 0.215)),
                         {'succeeded': 'INITIAL_LIFT_BRIDGE'})

        StateMachine.add('INITIAL_LIFT_BRIDGE',
                         SimpleActionState('place_bridge_action', PlaceBridgeAction,
                                           goal = PlaceBridgeGoal(down = False)),
                         {'succeeded': 'MOVE_BASE_TO_END_OF_TABLE_FOR_PREOBSERVATION'})

        StateMachine.add('MOVE_BASE_TO_END_OF_TABLE_FOR_PREOBSERVATION',
                         SimpleActionState('drive_pool', MoveBaseAction,
                                           goal_cb = plan_to_move_base_head_table_for_observation),
                         {'succeeded': 'INITIAL_SWING_ARMS_OUT'})

        StateMachine.add('INITIAL_SWING_ARMS_OUT',
                         SimpleActionState('swing_arms_action', SwingArmsAction,
                                           goal = SwingArmsGoal(away = True)),
                         {'succeeded': 'POINT_HEAD_AT_MIDDLE_OF_TABLE_PREOBSERVATION',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('POINT_HEAD_AT_MIDDLE_OF_TABLE_PREOBSERVATION',
                         SimpleActionState('/head_traj_controller/point_head_action', PointHeadAction,
                                           goal_cb = point_head_at_middle_of_table),
                         {'succeeded': 'INITIAL_PREOBSERVE_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('INITIAL_PREOBSERVE_TABLE',
                         SimpleActionState('rviz_get_table_state' if USE_RVIZ else 'vision_get_table_state', GetTableStateAction,
                                           goal = GetTableStateGoal(table_frame_id = POINT_HEAD_FIXED_FRAME, filter_against_table = False),
                                           result_slots_map = {'state': 'observed_table_state'}),
                         {'succeeded': 'MOVE_BASE_TO_END_OF_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('MOVE_BASE_TO_END_OF_TABLE',
                         SimpleActionState('drive_pool', MoveBaseAction,
                                           goal_cb = plan_to_move_base_head_table),
                         {'succeeded': 'INITIAL_LOCALIZE_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('INITIAL_LOCALIZE_TABLE',
                         SimpleActionState('rviz_localize_table' if USE_RVIZ else 'localize_table', LocalizeTableAction,
                                           goal_slots_map = {'observed_table_state' : 'state'},
                                           result_slots_map = {'pose': 'observed_table_pose', 'child_frame': 'observed_table_frame'}),
                         {'succeeded': 'INITIAL_PUBLISH_TABLE_FRAME',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('INITIAL_PUBLISH_TABLE_FRAME',
                         SimpleActionState('posestamped_to_tf', PoseStampedToTFAction,
                                           goal_slots_map = {'observed_table_pose' : 'pose', 'observed_table_frame' : 'child_frame'}),
                         {'succeeded': 'MOVE_BASE_TO_END_OF_TABLE_FOR_OBSERVATION',
                          'aborted': 'SWING_ARMS_IN_FAIL'})
        
        StateMachine.add('MOVE_BASE_TO_END_OF_TABLE_FOR_OBSERVATION',
                         SimpleActionState('drive_pool', MoveBaseAction,
                                           goal_cb = plan_to_move_base_head_table_for_observation),
                         {'succeeded': 'POINT_HEAD_AT_MIDDLE_OF_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('POINT_HEAD_AT_MIDDLE_OF_TABLE',
                         SimpleActionState('/head_traj_controller/point_head_action', PointHeadAction,
                                           goal_cb = point_head_at_middle_of_table),
                         {'succeeded': 'INITIAL_OBSERVE_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('INITIAL_OBSERVE_TABLE',
                         SimpleActionState('rviz_get_table_state' if USE_RVIZ else 'vision_get_table_state', GetTableStateAction,
                                           goal = GetTableStateGoal(table_frame_id = TABLE_FRAME, filter_against_table = True),
                                           result_slots_map = {'state': 'observed_table_state'}),
                         {'succeeded': 'INITIAL_SWING_ARMS_IN',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('INITIAL_SWING_ARMS_IN',
                         SimpleActionState('swing_arms_action', SwingArmsAction,
                                           goal = SwingArmsGoal(away = False)),
                         {'succeeded': 'INITIAL_PLAN_SHOT'})

        StateMachine.add('INITIAL_PLAN_SHOT',
                         SimpleActionState('plan_shot', PlanShotAction,
					   goal_cb = plan_to_plan_shot_unlimited,
                                           goal_slots_map = {'observed_table_state': 'state'},
                                           result_slots_map = {'shot': 'planned_shot'}),
                         {'succeeded': 'MOVE_BASE'})

        StateMachine.add('MOVE_BASE',
                         SimpleActionState('drive_pool', MoveBaseAction,
                                           goal_cb = plan_to_move_base),
                         {'succeeded': 'SWING_ARMS_OUT'})

        StateMachine.add('SWING_ARMS_OUT',
                         SimpleActionState('swing_arms_action', SwingArmsAction,
                                           goal = SwingArmsGoal(away = True)),
                         {'succeeded': 'POINT_HEAD_AT_CUE_BALL_PREOBSERVE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('POINT_HEAD_AT_CUE_BALL_PREOBSERVE',
                         SimpleActionState('/head_traj_controller/point_head_action', PointHeadAction,
                                           goal_cb = point_head_at_cue_ball),
                         {'succeeded': 'PREOBSERVE_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('PREOBSERVE_TABLE',
                         SimpleActionState('rviz_get_table_state' if USE_RVIZ else 'vision_get_table_state', GetTableStateAction,
                                           goal = GetTableStateGoal(table_frame_id = POINT_HEAD_FIXED_FRAME, filter_against_table = False),
                                           result_slots_map = {'state': 'new_observed_table_state'}),
                         {'succeeded': 'LOCALIZE_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('LOCALIZE_TABLE',
                         SimpleActionState('rviz_localize_table' if USE_RVIZ else 'localize_table', LocalizeTableAction,
                                           goal_slots_map = {'new_observed_table_state' : 'state'},
                                           result_slots_map = {'pose': 'observed_table_pose', 'child_frame': 'observed_table_frame'}),
                         {'succeeded': 'PUBLISH_TABLE_FRAME',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('PUBLISH_TABLE_FRAME',
                         SimpleActionState('posestamped_to_tf', PoseStampedToTFAction,
                                           goal_slots_map = {'observed_table_pose' : 'pose', 'observed_table_frame' : 'child_frame'}),
                         {'succeeded': 'POINT_HEAD_AT_CUE_BALL',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('POINT_HEAD_AT_CUE_BALL',
                         SimpleActionState('/head_traj_controller/point_head_action', PointHeadAction,
                                           goal_cb = point_head_at_cue_ball),
                         {'succeeded': 'OBSERVE_TABLE',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('OBSERVE_TABLE',
                         SimpleActionState('rviz_get_table_state' if USE_RVIZ else 'vision_get_table_state', GetTableStateAction,
                                           goal = GetTableStateGoal(table_frame_id = TABLE_FRAME, filter_against_table = True),
                                           result_slots_map = {'state': 'new_observed_table_state'}),
                         {'succeeded': 'SWING_ARMS_IN',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('SWING_ARMS_IN',
                         SimpleActionState('swing_arms_action', SwingArmsAction,
                                           goal = SwingArmsGoal(away = False)),
                         {'succeeded': 'PLAN_SHOT'})

        StateMachine.add('PLAN_SHOT',
                         SimpleActionState('plan_shot', PlanShotAction,
					   goal_cb = plan_to_plan_shot_limited,
                                           goal_slots_map = {'observed_table_state': 'state'},
                                           result_slots_map = {'shot': 'planned_shot'}),
                         {'succeeded': 'MOVE_BASE_AGAIN'})

        StateMachine.add('MOVE_BASE_AGAIN',
                         SimpleActionState('drive_pool', MoveBaseAction,
                                           goal_cb = plan_to_move_base),
                         {'succeeded': 'SWING_ARMS_OUT2'})

        StateMachine.add('SWING_ARMS_OUT2',
                         SimpleActionState('swing_arms_action',SwingArmsAction,
                                           goal= SwingArmsGoal(away = True)),
                         {'succeeded': 'POINT_HEAD_AT_CUE_BALL_PREOBSERVE2',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('POINT_HEAD_AT_CUE_BALL_PREOBSERVE2',
                         SimpleActionState('/head_traj_controller/point_head_action', PointHeadAction,
                                           goal_cb = point_head_at_cue_ball),
                         {'succeeded': 'PREOBSERVE_TABLE2',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('PREOBSERVE_TABLE2',
                         SimpleActionState('rviz_get_table_state' if USE_RVIZ else 'vision_get_table_state', GetTableStateAction,
                                           goal = GetTableStateGoal(table_frame_id = POINT_HEAD_FIXED_FRAME, filter_against_table = False),
                                           result_slots_map = {'state': 'new_observed_table_state'}),
                         {'succeeded': 'LOCALIZE_TABLE2',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('LOCALIZE_TABLE2',
                         SimpleActionState('rviz_localize_table' if USE_RVIZ else 'localize_table', LocalizeTableAction,
                                           goal_slots_map = {'new_observed_table_state' : 'state'},
                                           result_slots_map = {'pose': 'observed_table_pose', 'child_frame': 'observed_table_frame'}),
                         {'succeeded': 'PUBLISH_TABLE_FRAME2',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('PUBLISH_TABLE_FRAME2',
                         SimpleActionState('posestamped_to_tf', PoseStampedToTFAction,
                                           goal_slots_map = {'observed_table_pose' : 'pose', 'observed_table_frame' : 'child_frame'}),
                         {'succeeded': 'POINT_HEAD_AT_CUE_BALL2',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('POINT_HEAD_AT_CUE_BALL2',
                         SimpleActionState('/head_traj_controller/point_head_action', PointHeadAction,
                                           goal_cb = point_head_at_cue_ball),
                         {'succeeded': 'OBSERVE_TABLE2',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('OBSERVE_TABLE2',
                         SimpleActionState('rviz_get_table_state' if USE_RVIZ else 'vision_get_table_state', GetTableStateAction,
                                           goal = GetTableStateGoal(table_frame_id = TABLE_FRAME, filter_against_table = True),
                                           result_slots_map = {'state': 'new_observed_table_state'}),
                         {'succeeded': 'SWING_ARMS_IN2',
                          'aborted': 'SWING_ARMS_IN_FAIL'})

        StateMachine.add('SWING_ARMS_IN2',
                         SimpleActionState('swing_arms_action', SwingArmsAction,
                                           goal = SwingArmsGoal(away = False)),
                         {'succeeded': 'PLAN_SHOT2'})

        StateMachine.add('PLAN_SHOT2',
                         SimpleActionState('plan_shot', PlanShotAction,
                                           goal_cb = plan_to_plan_shot_limited,
                                           goal_slots_map = {'observed_table_state': 'state'},
                                           result_slots_map = {'shot': 'planned_shot'}),
                         {'succeeded': 'PLACE_BRIDGE'})

        StateMachine.add('PLACE_BRIDGE',
                         SimpleActionState('place_bridge_action', PlaceBridgeAction,
                                           goal_cb = extract_planned_bridge_pose,
                                           result_cb = print_result),
                         {'succeeded': 'STRIKE',
                          'aborted': 'LIFT_BRIDGE_FAIL'})

        StateMachine.add('STRIKE',
                         SimpleActionState('strike_action', StrikeAction,
                                           goal = StrikeGoal()),
                         {'succeeded': 'LIFT_BRIDGE',
                          'aborted': 'LIFT_BRIDGE_FAIL'})

        StateMachine.add('LIFT_BRIDGE',
                         SimpleActionState('place_bridge_action', PlaceBridgeAction,
                                           goal = PlaceBridgeGoal(down = False)),
                         {'succeeded': 'succeeded'})

        StateMachine.add('SWING_ARMS_IN_FAIL',
                         SimpleActionState('swing_arms_action', SwingArmsAction,
                                           goal = SwingArmsGoal(away = False)),
                         {'succeeded': 'aborted'})

        StateMachine.add('LIFT_BRIDGE_FAIL',
                         SimpleActionState('place_bridge_action', PlaceBridgeAction,
                                           goal = PlaceBridgeGoal(down = False)),
                         {'succeeded': 'aborted'})


    # Run state machine introspection server
    intro_server = IntrospectionServer('play_toplevel_smach',sm,'/PLAY')
    intro_server.start()

    # Run state machine action server 
    asw = ActionServerWrapper(
            'play_toplevel', PlayAction, sm,
            succeeded_outcomes = ['succeeded'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'])
    asw.run_server()

    
    rospy.spin()
    intro_server.stop()

if __name__ == '__main__':
    import sys
    main(sys.argv)
