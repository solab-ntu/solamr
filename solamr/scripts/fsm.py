#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import actionlib
import rospy
import roslib
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
import smach
import smach_ros

from fsm_define import Single_AMR, Single_Assembled # , BehaviorState

# def cb_start(request):
#     SM.execute()  # start FSM
#     return EmptyResponse()


if __name__ == "__main__":

    rospy.init_node(name='fsm', anonymous=False)

    #-- Get parameters
    # ns_action = rospy.get_param(param_name="~ns_action")
    # path_file = rospy.get_param(param_name="~path_file")
    FILE_PATH_SINGLE_AMR = rospy.get_param(param_name="~file_path_single_amr")
    FILE_PATH_SINGLE_ASSEMBLED = rospy.get_param(param_name="~file_path_single_assembled")
    # -- Get service
    # action_move_base = actionlib.SimpleActionClient(ns=ns_action, ActionSpec=MoveBaseAction)
    # action_move_base.wait_for_server()

    # -- Get publisher
    # pub_cmd_vel = rospy.Publisher(name=top_cmd_vel, data_class=Twist, queue_size=1)
    # 
    # -- Node function
    # rospy.Service(name="~start", service_class=Empty, handler=cb_start)

    # -- FSM Config
    SM = smach.StateMachine(outcomes=['completed'])
    with SM:
        smach.StateMachine.add(
            label='Single_AMR',
            state=Single_AMR(file_path=FILE_PATH_SINGLE_AMR),
            transitions={'Single_Assembled': 'Single_Assembled', 'done': 'completed'})
        smach.StateMachine.add(
            label='Single_Assembled',
            state=Single_Assembled(file_path=FILE_PATH_SINGLE_ASSEMBLED),
            transitions={'Single_AMR': 'Single_AMR', 'done': "completed"})
        # smach.StateMachine.add(
        #     label='BehaviorState',
        #     state=BehaviorState(pub_cmd_vel=pub_cmd_vel),
        #     transitions={'done': 'PlanningState'})
        

    # -- FSM VIEWER
    sis = smach_ros.IntrospectionServer('smach_server', SM, '/ROOT')
    sis.start()
    SM.execute() # start FSM
    rospy.spin()
    sis.stop()
