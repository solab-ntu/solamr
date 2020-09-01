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

from fsm_define import Single_AMR, Single_Assembled, Find_Shelf, Go_Dock_Standby, Dock_In, Dock_Out, Double_Assembled# , BehaviorState

# def cb_start(request):
#     SM.execute()  # start FSM
#     return EmptyResponse()


if __name__ == "__main__":

    rospy.init_node(name='fsm', anonymous=False)

    #-- Get parameters
    FILE_PATH_SINGLE_AMR = rospy.get_param(param_name="~file_path_single_amr")
    FILE_PATH_DOUBLE_AMR = rospy.get_param(param_name="~file_path_double_amr")
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")
    ROLE = rospy.get_param(param_name="~role")
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
            transitions={'Find_Shelf': 'Find_Shelf',  # TODO listen to a "combine_shlef" service
                         'done': 'completed'})
        
        smach.StateMachine.add(
            label='Find_Shelf',
            state=Find_Shelf(),
            transitions={'abort': 'Single_AMR', # For abort
                         'done': 'Go_Dock_Standby'}) # If found tags

        smach.StateMachine.add(
            label='Go_Dock_Standby',
            state=Go_Dock_Standby(),
            transitions={'abort': 'Single_AMR', # For abort
                         'done': 'Dock_In'}) # If dock standby

        smach.StateMachine.add(
            label='Dock_In',
            state=Dock_In(),
            transitions={'abort': 'Single_AMR', # For abort
                         'Single_Assembled': 'Single_Assembled', # Dock succeeed, single assembled 
                         'Double_Assembled': 'Double_Assembled', }) # # Dock succeeed, double assembled 

        smach.StateMachine.add(
            label='Dock_Out',
            state=Dock_Out(),
            transitions={'abort': 'Single_Assembled', # For abort, # TODO decide?
                         # 'abort': 'Double_Assembled', # For abort
                         'done': 'Single_AMR'}) # Successfully dock out

        smach.StateMachine.add(
            label='Single_Assembled',
            state=Single_Assembled(),
            transitions={'Dock_Out': 'Dock_Out'})
        
        smach.StateMachine.add(
            label='Double_Assembled',
            state=Double_Assembled(file_path=FILE_PATH_DOUBLE_AMR),
            transitions={'Dock_Out': 'Dock_Out'})
        

    # -- FSM VIEWER
    sis = smach_ros.IntrospectionServer('smach_server', SM, '/ROOT')
    sis.start()
    SM.execute() # start FSM
    rospy.spin()
    sis.stop()
