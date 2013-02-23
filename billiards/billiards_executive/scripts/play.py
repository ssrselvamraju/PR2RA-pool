#! /usr/bin/env python

import roslib
roslib.load_manifest('billiards_executive')

import rospy
import actionlib
from smach import *

from actionlib_msgs.msg import *
from billiards_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *

def print_result(userdata, status, result):
    print "RESULT:", result

def extract_planned_bridge_pose(ud, goal):
    return PlaceBridgeGoal(down = True,
                           pose = ud.planned_shot.bridge_pose)

def plan_to_move_base(ud, goal):
    goal.target_pose = ud.planned_shot.base_pose
    return goal

def main():
    rospy.init_node('play_fixed')

    sm = StateMachine(outcomes = ['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.add('INITIAL_LIFT_BRIDGE',
                         SimpleActionState('place_bridge_action', PlaceBridgeAction,
                                           goal = PlaceBridgeGoal(down = False)),
                         {'succeeded': 'OBSERVE_TABLE'})
                         
        
        StateMachine.add('OBSERVE_TABLE',
                         SimpleActionState('rviz_get_table_state', GetTableStateAction,
                                           goal = GetTableStateGoal(),
                                           result_slots_map = {'state': 'observed_table_state'}),
                         {'succeeded': 'PLAN_SHOT'})

        StateMachine.add('PLAN_SHOT',
                         SimpleActionState('rviz_plan_shot', PlanShotAction,
                                           goal_slots_map = {'observed_table_state': 'state'},
                                           result_slots_map = {'shot': 'planned_shot'}),
                         {'succeeded': 'MOVE_BASE'})

        StateMachine.add('MOVE_BASE',
                         SimpleActionState('drive_pool', MoveBaseAction,
                                           goal_cb = plan_to_move_base),
                         {'succeeded': 'PLACE_BRIDGE'})
                         #{'succeeded': 'succeeded'})

        StateMachine.add('PLACE_BRIDGE',
                         SimpleActionState('place_bridge_action', PlaceBridgeAction,
                                           goal_cb = extract_planned_bridge_pose,
                                           result_cb = print_result),
                         {'succeeded': 'STRIKE'})

        StateMachine.add('STRIKE',
                         SimpleActionState('strike_action', StrikeAction,
                                           goal = StrikeGoal()),
                         #{'succeeded': 'LIFT_BRIDGE'}
                         {'succeeded': 'succeeded'}
                         )

        StateMachine.add('LIFT_BRIDGE',
                         SimpleActionState('place_bridge_action', PlaceBridgeAction,
                                           goal = PlaceBridgeGoal(down = False)),
                         {'succeeded': 'succeeded'})
                                           

    # Run state machine introspection server
    intro_server = IntrospectionServer('play_fixed_smach',sm,'/PLAY')
    intro_server.start()

    # Run state machine action server 
    asw = ActionServerWrapper(
            'play_fixed', PlayAction, sm,
            succeeded_outcomes = ['succeeded'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'])
    asw.run_server()

    
    rospy.spin()
    intro_server.stop()

if __name__ == '__main__':
    main()
