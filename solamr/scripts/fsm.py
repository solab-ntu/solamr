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
from geometry_msgs.msg import Twist, Point
from move_base_msgs.msg import MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from solamr.srv import StringSrv
import roslaunch
# Custom 
from lucky_utility.ros.lucky_utility import get_tf


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

def switch_launch_double():
    '''
    '''
    global ROSLAUNCH
    if ROSLAUNCH != None:
        rospy.loginfo("[fsm] Shuting down single_AMR launch file")
        ROSLAUNCH.shutdown()
        time.sleep(5) # TODO DO WE need it ?
    rospy.loginfo("[fsm] Start launch file: " + ROSLAUNCH_PATH_DOUBLE_AMR)
    ROSLAUNCH = init_roslaunch(ROSLAUNCH_PATH_DOUBLE_AMR)
    ROSLAUNCH.start()

def switch_launch_single():
    global ROSLAUNCH
    if ROSLAUNCH != None:
        rospy.loginfo("[fsm] Shuting down double_AMR launch file")
        ROSLAUNCH.shutdown()
        time.sleep(5) # TODO DO WE need it ?
    rospy.loginfo("[fsm] Start launch file: " + ROSLAUNCH_PATH_SINGLE_AMR)
    ROSLAUNCH = init_roslaunch(ROSLAUNCH_PATH_SINGLE_AMR)
    ROSLAUNCH.start()

def task_cb(req):
    '''
    Load yaml file and change parameter by req.data file path
    '''
    global TASK

    # Check Task is busy
    if TASK != None:
        rospy.logerr("[fsm] Reject task, because I'm busy now.")
        return "Reject task, I'm busy now"
    
    path = rospkg.RosPack().get_path('solamr') + "/params/task/" + req.data\
           + "_" + ROBOT_NAME + ".yaml"
    try:
        with open(path) as file:
            rospy.loginfo("[shelf_detector] Load yaml file from " + path)
            # params = yaml.load(file, Loader=yaml.FullLoader)
            params = yaml.safe_load(file)
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


class Goal_Manager(object):
    def __init__(self):
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action.wait_for_server()
        self.is_reached = False
        self.goal = None
    
    def send_goal(self, xyt, frame_id):
        self.is_reached = False
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = frame_id
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = xyt[0]
        self.goal.target_pose.pose.position.y = xyt[1]
        self.goal.target_pose.pose.position.z = 0
        quaternion = quaternion_from_euler(0.0, 0.0, xyt[2])
        (self.goal.target_pose.pose.orientation.x,
         self.goal.target_pose.pose.orientation.y,
         self.goal.target_pose.pose.orientation.z,
         self.goal.target_pose.pose.orientation.w) = quaternion
        self.action.send_goal(goal=self.goal, feedback_cb=self.feedback_cb, done_cb=self.reached_cb)

    def feedback_cb(self):
        pass
    
    def reached_cb(self):
        self.is_reached = True

class Initial_State(smach.State):
    def __init__(self):
        super(Initial_State, self).__init__(outcomes=('Single_AMR', 'Single_Assembled', 'Double_Assembled'))

    def execute(self, userdata):
        if INIT_STATE == 'Single_AMR' or INIT_STATE == 'Single_Assembled':
            switch_launch_single()
        elif INIT_STATE == 'Double_Assembled':
            switch_launch_double()
        return INIT_STATE

class Single_AMR(smach.State):
    '''
    Waiting State, do nothing
    '''
    def __init__(self, file_path):
        super(Single_AMR, self).__init__(outcomes=('Find_Shelf', 'done'), output_keys=[])
        self.launch_file = file_path

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Single_AMR')
        while not rospy.is_shutdown():
            if TASK != None:
                break
            time.sleep(1)
        return 'Find_Shelf'

class Find_Shelf(smach.State):
    def __init__(self):
        super(Find_Shelf, self).__init__(outcomes=('abort', 'done'))

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Find_Shelf')

        # TODO Send search radius

        # Send goal
        GOAL_MANAGER.send_goal(TASK.tag_location, ROBOT_NAME + "/base_link")

        while not rospy.is_shutdown():
            # Get apriltag shelf location
            if ROBOT_NAME == "car1":
                shelf_xyt = get_tf(TFBUFFER, "car1/base_link", "car1/shelf_one")
            elif ROBOT_NAME == "car2":
                shelf_xyt = get_tf(TFBUFFER, "car2/base_link", "car2/shelf_two")
            
            if shelf_xyt != None:
                return 'done'
                # goal_xyt = shelf_xyt # TODO add a distance
            
            # Check goal reached or not 
            if GOAL_MANAGER.is_reached:
                return 'done'
            time.sleep(0.1)

class Go_Dock_Standby(smach.State):

    def __init__(self):
        super(Go_Dock_Standby, self).__init__(outcomes=('abort','done'), output_keys=[])

    def execute(self, userdata):
        '''
        '''
        rospy.loginfo('[fsm] Execute Go_Dock_Standby')

        while not rospy.is_shutdown():
            # Get apriltag shelf location
            if ROBOT_NAME == "car1":
                shelf_xyt = get_tf(TFBUFFER, "car1/base_link", "car1/shelf_one")
            elif ROBOT_NAME == "car2":
                shelf_xyt = get_tf(TFBUFFER, "car2/base_link", "car2/shelf_two")
            
            
            if shelf_xyt != None:
                # Update goal by camera apriltag 
                GOAL_MANAGER.send_goal(shelf_xyt, ROBOT_NAME + "/base_link")# TODO add a distance
                # goal_xyt = shelf_xyt 

                # Update search center for shelf detector
                point = Point()
                point.x = shelf_xyt[0]
                point.y = shelf_xyt[1]
                PUB_SEARCH_CENTER.publish(point)

            # Check goal reached or not 
            if GOAL_MANAGER.is_reached:
                return 'done'
            
            time.sleep(0.1)

        time.sleep(1)
        # TODO Go to standby point , if navi goal is 
        # Send_goal()
        return 'done'

class Dock_In(smach.State):

    def __init__(self):
        super(Dock_In, self).__init__(outcomes=('Single_Assembled', 'Double_Assembled', 'abort'), output_keys=["target"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Dock_In')
        # TODO If IR said it's done or navigation goal is reached
        time.sleep(1)
        if TASK.mode == "single_AMR":
            # TODO Dark Magic, Change footprint to single Assembled
            os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.37,-0.37,0.0], [-0.37,0.37,0.0], [0.37,0.37,0.0], [0.37,-0.37,0.0]]'")
            return 'Single_Assembled'
        elif TASK.mode == "double_AMR":
            switch_launch_double()
            return 'Double_Assembled'

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
        if TASK.mode == "single_AMR":
            # TODO Dark Magic, Change footprint back to single AMR
            os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.22,-0.22,0.0], [-0.22,0.22,0.0], [0.22,0.22,0.0], [0.22,-0.22,0.0]]'")
        elif TASK.mode == "double_AMR":
            switch_launch_single()

        time.sleep(1)
        return 'done'

class Go_Home(smach.State):
    def __init__(self):
        super(Go_Home, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global TASK
        rospy.loginfo('[fsm] Execute Go_Home')
        time.sleep(1)
        TASK = None
        return 'done'

class Single_Assembled(smach.State):

    def __init__(self):
        super(Single_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Single_Assembled')
        # TODO rosservice to dock out or accept a task
        while not rospy.is_shutdown():
            if TASK == None:
                # TODO listen to service to decide
                continue
                return 'Dock_Out'
            else:
                return 'Go_Way_Point'
            time.sleep(1)

class Double_Assembled(smach.State):

    def __init__(self, file_path):
        super(Double_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point'], output_keys=["target"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Double_Assembled')
        while not rospy.is_shutdown():
            if TASK == None:
                # TODO listen to service to decide
                continue
                return 'Dock_Out'
            else:
                return 'Go_Way_Point'
            time.sleep(1)

if __name__ == "__main__":

    rospy.init_node(name='fsm', anonymous=False)

    #-- Get parameters
    ROSLAUNCH_PATH_SINGLE_AMR = rospy.get_param(param_name="~roslaunch_path_single_amr")
    ROSLAUNCH_PATH_DOUBLE_AMR = rospy.get_param(param_name="~roslaunch_path_double_amr")
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")
    ROLE = rospy.get_param(param_name="~role")
    INIT_STATE = rospy.get_param(param_name="~init_state")

    # Service
    rospy.Service(name="~task", service_class=StringSrv, handler=task_cb)

    # Publisher
    PUB_SEARCH_CENTER = rospy.Publisher(ROBOT_NAME + "/search_center", Point, queue_size = 1)

    # Global variable 
    TASK = None # store task information
    ROSLAUNCH = None
    # Goal manager
    GOAL_MANAGER = Goal_Manager()
    # For getting Tf
    TFBUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TFBUFFER)

    # -- FSM Config
    SM = smach.StateMachine(outcomes=['completed'])
    with SM:
        smach.StateMachine.add(
            label='Initial_State',
            state=Initial_State(),
            transitions={'Single_AMR': 'Single_AMR',
                         'Single_Assembled': 'Single_Assembled',
                         'Double_Assembled': 'Double_Assembled'})
        
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
            transitions={'Dock_Out': 'Dock_Out', 'Go_Way_Point': 'Go_Way_Point'})
    
    # -- FSM VIEWER
    sis = smach_ros.IntrospectionServer('smach_server', SM, '/ROOT')
    sis.start()
    SM.execute() # start FSM
    rospy.spin()
    sis.stop()
