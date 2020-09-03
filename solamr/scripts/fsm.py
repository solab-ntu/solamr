#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import actionlib
import rospy
import roslib
import yaml
import os
import time

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
import smach
import smach_ros
# ROS
import rospkg
import roslaunch
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from solamr.srv import StringSrv
import roslaunch

class Task(object):
    def __init__(self, mode, shelf_id, tag_location,
                 wait_location = None, goal_location = None, home_location = None):
        self.mode = mode
        self.shelf_id = shelf_id
        self.tag_location = tag_location
        self.wait_location = wait_location
        self.goal_location = goal_location
        self.home_location = home_location

#######################
### Global Function ###
#######################
def init_roslaunch(file_path):
    '''
    Let it able to switch roslaunch 
    Argument:
        file_path: string - the roslaunch file path you want to launch
    Return:
        parent of roslaunch process
    '''
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    return roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))

def task_cb(req):
    '''
    Load yaml file and change parameter by req.data file path
    '''
    global TASK

    # Check Task is busy
    if TASK != None:
        rospy.logerr("[fsm] Reject task, because I'm busy now.")
        return "Reject task, I'm busy now"
    
    path = rospkg.RosPack().get_path('solamr') + "/params/task/" + req.data
    try:
        with open(path) as file:
            rospy.loginfo("[shelf_detector] Load yaml file from " + path)
            params = yaml.load(file, Loader=yaml.FullLoader)
            TASK = Task(params['mode'],
                        params['shelf_id'],
                        params['tag_location'],
                        params['wait_location'],
                        params['goal_location'],
                        params['home_location']
                        )
            return "OK"
    except OSError:
        rospy.logerr("[fsm] Yaml file not found at " + req.data)
        return "Yaml file not found"

class Single_AMR(smach.State):

    def __init__(self, file_path):
        super(Single_AMR, self).__init__(outcomes=('Find_Shelf', 'done'), output_keys=[])
        self.launch_file = file_path

    def execute(self, userdata):
        roslaunch = init_roslaunch(self.launch_file)
        roslaunch.start()
        # roslaunch.shutdown()
        rospy.loginfo('[fsm] Execute Single_AMR')
        while not rospy.is_shutdown():
            if TASK != None:
                break
            time.sleep(1)
        print (TASK.mode)
        # TODO Dark Magic, Change footprint back to single AMR
        os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.22,-0.22,0.0], [-0.22,0.22,0.0], [0.22,0.22,0.0], [0.22,-0.22,0.0]]'")
        return 'Find_Shelf'

class Find_Shelf(smach.State):

    def __init__(self):
        super(Find_Shelf, self).__init__(outcomes=('abort', 'done'))

    def execute(self, userdata):
        '''
        userdata.tagID -> find shelf
        userdata.proximity_goal -> A fuszy goal that shelf prababaly nearby
        '''
        # TODO Listen to service abort
        rospy.loginfo('[fsm] Execute Find_Shelf')
        rospy.loginfo(str(TASK.mode))
        # TODO send_goal (userdata.proximity_goal)
        # Keep checking tagid, and make sure standby tf is published
        time.sleep(1)
        return 'done'

class Go_Dock_Standby(smach.State):

    def __init__(self):
        super(Go_Dock_Standby, self).__init__(outcomes=('abort','done'), output_keys=[])

    def execute(self, userdata):
        '''
        '''
        rospy.loginfo('[fsm] Execute Go_Dock_Standby')
        time.sleep(1)
        # TODO Go to standby point , if navi goal is 
        # Send_goal()
        return 'done'

class Dock_In(smach.State):

    def __init__(self):
        super(Dock_In, self).__init__(outcomes=('Single_Assembled', 'Double_Assembled', 'abort'), output_keys=["target"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Dock_In')
        # TODO # Switch launch file, You have to know you're car1/car2 at this point
        # TODO How to decide goto Single_Assembled or Double_Assembled
        # If IR said it's done or navigation goal is reached
        time.sleep(1)
        return 'Single_Assembled'

class Dock_Out(smach.State):

    def __init__(self):
        super(Dock_Out, self).__init__(outcomes=('abort', 'done'), output_keys=["target"])

    def execute(self, userdata):
        '''
        '''
        rospy.loginfo('[fsm] Execute Dock_Out')
        
        # Switch launch file 
        # TODO How to decide goto Single_Assembled or Double_Assembled
        # TODO reached dock_out navi goal
        time.sleep(1)
        return 'done'

class Single_Assembled(smach.State):

    def __init__(self):
        super(Single_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Single_Assembled')
        
        # TODO Dark Magic, Change footprint to single Assembled
        os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.37,-0.37,0.0], [-0.37,0.37,0.0], [0.37,0.37,0.0], [0.37,-0.37,0.0]]'")
        time.sleep(5)
        return 'Go_Way_Point'
        '''
        x_m, y_m, rz_deg, behavior = userdata.target
        userdata.behavior = behavior

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_m
        goal.target_pose.pose.position.y = y_m
        q = quaternion_from_euler(0.0, 0.0, radians(rz_deg))
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo('fsm/Single_Assembled : {} {} {}'.format(x_m, y_m, rz_deg))
        self.status = None
        self.action_move_base.send_goal(goal=goal, feedback_cb=None, done_cb=self.done_cb)
        self.action_move_base.wait_for_result()

        if self.status == 3:
            return 'done'
        else:
            return 'failed'
        '''

class Go_Way_Point(smach.State):
    def __init__(self):
        super(Go_Way_Point, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Go_Way_Point')
        time.sleep(1)
        return 'done'

class Go_Goal(smach.State):
    def __init__(self):
        super(Go_Goal, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Go_Goal')
        time.sleep(1)
        return 'done'

class Go_Home(smach.State):
    def __init__(self):
        super(Go_Home, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Go_Home')
        time.sleep(1)
        return 'done'

class Double_Assembled(smach.State):

    def __init__(self, file_path):
        super(Double_Assembled, self).__init__(outcomes=['Dock_Out'], output_keys=["target"])
        self.launch_file = file_path

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Double_Assembled')

        roslaunch = init_roslaunch(self.launch_file)
        roslaunch.start()
        time.sleep(5)
        # roslaunch.shutdown()
        return 'Dock_Out'

if __name__ == "__main__":

    rospy.init_node(name='fsm', anonymous=False)

    #-- Get parameters
    ROSLAUNCH_PATH_SINGLE_AMR = rospy.get_param(param_name="~roslaunch_path_single_amr")
    ROSLAUNCH_PATH_DOUBLE_AMR = rospy.get_param(param_name="~roslaunch_path_double_amr")
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")
    ROLE = rospy.get_param(param_name="~role")

    # Service
    rospy.Service(name="~task", service_class=StringSrv, handler=task_cb)

    # Global variable 
    TASK = None # store task information
    # -- Get service
    # action_move_base = actionlib.SimpleActionClient(ns=ns_action, ActionSpec=MoveBaseAction)
    # action_move_base.wait_for_server()
    
    # -- FSM Config
    SM = smach.StateMachine(outcomes=['completed'])
    with SM:
        smach.StateMachine.add(
            label='Single_AMR',
            state=Single_AMR(file_path=ROSLAUNCH_PATH_SINGLE_AMR),
            transitions={'Find_Shelf': 'Find_Shelf',
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
                         'done': 'Go_Home'}) # Successfully dock out

        smach.StateMachine.add(
            label='Single_Assembled',
            state=Single_Assembled(),
            transitions={'Dock_Out': 'Dock_Out', 'Go_Way_Point': 'Go_Way_Point'})

        smach.StateMachine.add(
            label='Go_Way_Point',
            state=Go_Way_Point(),
            transitions={'abort': 'Single_Assembled', 
                         'done': 'Go_Goal'}) # Successfully reached way point

        smach.StateMachine.add(
            label='Go_Goal',
            state=Go_Goal(),
            transitions={'abort': 'Single_Assembled',
                         'done': 'Dock_Out'}) # Successfully reached goal

        smach.StateMachine.add(
            label='Go_Home',
            state=Go_Home(),
            transitions={'abort': 'Single_AMR',
                         'done': 'Single_AMR'}) # Successfully go home

        smach.StateMachine.add(
            label='Double_Assembled',
            state=Double_Assembled(file_path=ROSLAUNCH_PATH_DOUBLE_AMR),
            transitions={'Dock_Out': 'Dock_Out'})
    
    # -- FSM VIEWER
    sis = smach_ros.IntrospectionServer('smach_server', SM, '/ROOT')
    sis.start()
    SM.execute() # start FSM
    rospy.spin()
    sis.stop()
