#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

# Python 
import yaml
import os
import time
from math import pi
import threading
# ROS
import rospy
import roslib
import rospkg
import roslaunch
import tf2_ros
import actionlib
import smach
import smach_ros
# Ros message
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Point, PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from solamr.srv import StringSrv
from std_msgs.msg import String, Bool
# Custom
from lucky_utility.ros.rospy_utility import get_tf, vec_trans_coordinate, send_tf

class Task(object):
    def __init__(self):
        self.mode = None
        self.shelf_location = None
        self.wait_location = None
        self.goal_location = None
        self.home_location = None
        self.task_flow = None

#######################
### Global Function ###
#######################
# def init_roslaunch(file_path):
#     '''
#     Let it able to switch roslaunch 
#     Argument:
#         file_path: string - the roslaunch file path you want to launch
#     Return:
#         parent of roslaunch process
#     '''
#     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#     roslaunch.configure_logging(uuid=uuid)
#     return roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))

def check_running():
    global IS_RUN
    while not rospy.is_shutdown():
        time.sleep(1)
    IS_RUN = False

# def switch_launch_double():
#     '''
#     '''
#     global ROSLAUNCH
#     if not IS_DUMMY_TEST:
#         if ROSLAUNCH != None:
#             rospy.loginfo("[fsm] Shuting down single_AMR launch file")
#             ROSLAUNCH.shutdown()
#             time.sleep(5) # TODO DO WE need it ?
#         rospy.loginfo("[fsm] Start launch file: " + ROSLAUNCH_PATH_DOUBLE_AMR)
#         ROSLAUNCH = init_roslaunch(ROSLAUNCH_PATH_DOUBLE_AMR)
#         ROSLAUNCH.start()

# def switch_launch_single():
#     global ROSLAUNCH
#     if ROSLAUNCH != None:
#         rospy.loginfo("[fsm] Shuting down double_AMR launch file")
#         ROSLAUNCH.shutdown()
#         time.sleep(5) # TODO DO WE need it ?
#     rospy.loginfo("[fsm] Start launch file: " + ROSLAUNCH_PATH_SINGLE_AMR)
#     ROSLAUNCH = init_roslaunch(ROSLAUNCH_PATH_SINGLE_AMR)
#     ROSLAUNCH.start()

def switch_launch(file_path):
    global ROSLAUNCH
    rospy.loginfo("[fsm] Start launch file: " + file_path)
    if not IS_DUMMY_TEST:
        if ROSLAUNCH != None:
            rospy.loginfo("[fsm] Shuting down single_AMR launch file")
            ROSLAUNCH.shutdown()
            time.sleep(5) # TODO DO WE need it ?
        # ROSLAUNCH = init_roslaunch(file_path)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid=uuid)
        ROSLAUNCH = roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))
        ROSLAUNCH.start()
    else:
        pass

def transit_mode(from_mode, to_mode):
    if   from_mode == "Single_AMR" and to_mode == "Single_Assembled":
        # TODO Ros topic 
        os.system("rostopic pub --once /" + ROBOT_NAME + "/move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.65,-0.65,0.0], [-0.65,0.65,0.0], [0.65,0.65,0.0], [0.65,-0.65,0.0]]'")
    
    elif from_mode == "Single_AMR" and to_mode == "Double_Assembled":
        switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)
    
    elif from_mode == "Single_Assembled" and to_mode == "Single_AMR":
        os.system("rostopic pub --once /" + ROBOT_NAME + "/move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.22,-0.22,0.0], [-0.22,0.22,0.0], [0.22,0.22,0.0], [0.22,-0.22,0.0]]'")
    
    elif from_mode == "Single_Assembled" and to_mode == "Double_Assembled":
        switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)

    elif from_mode == "Double_Assembled" and to_mode == "Single_AMR":
        switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
    
    elif from_mode == "Double_Assembled" and to_mode == "Single_Assembled":
        switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        # TODO change footprint?

    else:
        pass

def task_cb(req):
    '''
    Load yaml file and change parameter by req.data file path
    '''
    global TASK

    if req.data == "abort":
        TASK = None
        # Cacelled all goal
        GOAL_MANAGER.action.cancel_all_goals()
        return 'abort OK'
    
    elif req.data[:3] == "jp2":
        # Jump to assign state
        req_list = req.data.split(' ')
        task_tmp = Task()
        task_tmp.task_flow = [CUR_STATE, req_list[1]]
        TASK = task_tmp
        return "jump to " + req_list[1] + " OK"

    # Check Task is busy
    if TASK != None:
        rospy.logerr("[fsm] Reject task, because I'm busy now.")
        return "Reject task, I'm busy now"
    

    # Load yaml from task_location.yaml
    
    # Get mode
    req_list = req.data.split('_')
    mode = req_list[0] + "_" + req_list[1]
    try:
        path = rospkg.RosPack().get_path('solamr') + "/params/task/task_location.yaml"
        with open(path) as file:
            rospy.loginfo("[shelf_detector] Load yaml file from " + path)
            params = yaml.safe_load(file)
            task_tmp = Task()
            task_tmp.mode = mode
            task_tmp.shelf_location = params[mode]['shelf_location_' + ROBOT_NAME]
            task_tmp.wait_location  = params[mode]['wait_location_' + ROBOT_NAME]
            task_tmp.goal_location  = params[mode]['goal_location_' + ROBOT_NAME]
            task_tmp.home_location  = params[mode]['home_location_' + ROBOT_NAME]

        path = rospkg.RosPack().get_path('solamr') + "/params/task/" + req.data + ".yaml"
        with open(path) as file:
            rospy.loginfo("[shelf_detector] Load yaml file from " + path)
            params = yaml.safe_load(file)
            task_tmp.task_flow = params['task_flow']
            TASK = task_tmp
            return "OK"
    except OSError:
        rospy.logerr("[fsm] Yaml file not found at " + req.data)
        return "Yaml file not found"

def gate_reply_cb(data):
    global GATE_REPLY
    GATE_REPLY = data.data

class Goal_Manager(object):
    def __init__(self):
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.action.wait_for_server()
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

    def feedback_cb(self, feedback):
        pass
    
    def reached_cb(self,status, result):
        self.is_reached = True

class Initial_State(smach.State):
    def __init__(self):
        super(Initial_State, self).__init__(outcomes=('Single_AMR', 'Single_Assembled', 'Double_Assembled'))

    def execute(self, userdata):
        if INIT_STATE == 'Single_AMR' or INIT_STATE == 'Single_Assembled':
            transit_mode('Double_Assembled', 'Single_AMR')
            # switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        elif INIT_STATE == 'Double_Assembled':
            # switch_launch_double()
            # switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)
            transit_mode('Single_AMR', 'Double_Assembled')
        return INIT_STATE

class Single_AMR(smach.State):
    '''
    Waiting State, do nothing
    '''
    def __init__(self):
        super(Single_AMR, self).__init__(outcomes=('Find_Shelf','Single_Assembled','Double_Assembled', 'done'), output_keys=[])

    def execute(self, userdata):
        global TASK, CUR_STATE
        CUR_STATE = "Single_AMR"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        while IS_RUN:
            if TASK != None:
                try:
                    next_state = TASK.task_flow[TASK.task_flow.index('Single_AMR')+1]
                except IndexError:
                    rospy.loginfo("[fsm] task done")
                    TASK = None
                else:
                    if next_state == 'Single_Assembled' or next_state == 'Double_Assembled':
                        transit_mode("Single_AMR", next_state)
                    return next_state
            time.sleep(TIME_INTERVAL)
        return 'Find_Shelf'

class Find_Shelf(smach.State):
    def __init__(self):
        super(Find_Shelf, self).__init__(outcomes=('abort', 'done'))

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Find_Shelf"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if IS_DUMMY_TEST:
            time.sleep(2)
            return 'done'

        # Send goal
        GOAL_MANAGER.send_goal(TASK.shelf_location, ROBOT_NAME + "/map")
        while IS_RUN and TASK != None:
            # Get apriltag shelf location
            shelf_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_" + ROBOT_NAME)

            if shelf_xyt != None:
                return 'done'
            
            # Check goal reached or not 
            if GOAL_MANAGER.is_reached:
                rospy.logerr("[fsm] Can't find tf " + ROBOT_NAME + "/shelf_" + ROBOT_NAME)
                return 'abort'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Dock_Standby(smach.State):

    def __init__(self):
        super(Go_Dock_Standby, self).__init__(outcomes=('abort','done'), output_keys=[])

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Go_Dock_Standby"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if IS_DUMMY_TEST:
            time.sleep(2)
            return 'done'
        
        shelf_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_" + ROBOT_NAME)
        # 
        if shelf_xyt != None:
            (x1,y1) = vec_trans_coordinate((-0.5, 0), (shelf_xyt[0], shelf_xyt[1], shelf_xyt[2] + pi/2))
            GOAL_MANAGER.send_goal((x1, y1, shelf_xyt[2] + pi/2),
                                    ROBOT_NAME + "/map")
        
        while IS_RUN and TASK != None:
            # Check goal reached or not
            if GOAL_MANAGER.is_reached:
                return 'done'
            
            send_tf((0.0, 0.0, 0.0), ROBOT_NAME + "/shelf_" + ROBOT_NAME, ROBOT_NAME + "/tag/shelf_center", z_offset=-0.3)
            xyt = get_tf(TFBUFFER, ROBOT_NAME +"/base_link", ROBOT_NAME +"/tag/shelf_center")
            
            # Send search center to shelf detector
            if xyt != None:
                point = Point()
                point.x = xyt[0]
                point.y = xyt[1]
                PUB_SEARCH_CENTER.publish(point)
            
            time.sleep(TIME_INTERVAL)

        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Dock_In(smach.State):
    def __init__(self):
        super(Dock_In, self).__init__(outcomes=('Single_Assembled', 'Double_Assembled', 'abort'), output_keys=["target"])

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Dock_In"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if IS_DUMMY_TEST:
            time.sleep(2)
            next_state = TASK.task_flow[TASK.task_flow.index('Dock_In')+1]
            return next_state

        # Open gate
        PUB_GATE_CMD.publish(Bool(False))
        # Send goal 
        GOAL_MANAGER.send_goal((0.1, 0, 0), ROBOT_NAME + "/shelf_center")
        while IS_RUN and TASK != None:
            if GOAL_MANAGER.is_reached:
                # Close gate
                PUB_GATE_CMD.publish(Bool(True))
                # Get reply, Dockin successfully
                next_state = TASK.task_flow[TASK.task_flow.index('Dock_In')+1]
                transit_mode('Single_AMR', next_state)
                return next_state

                # if next_state == 'Single_Assembled':
                #     # TODO use ros topic
                    
                #     transit_mode('Single_AMR', next_state)
                #     return next_state
                # elif next_state == 'Double_Assembled':
                #     # switch_launch_double()
                    
                #     transit_mode('Single_AMR', next_state)
                #     return next_state
                # else:
                #     rospy.logwarn('[fsm] Unreconginize next stage: ' + str(next_state))
                #     return 'abort'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Way_Point(smach.State):
    def __init__(self):
        super(Go_Way_Point, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Go_Way_Point"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if IS_DUMMY_TEST:
            time.sleep(2)
            return 'done'
        
        # TODO 
        # GOAL_MANAGER.send_goal(TASK.wait_location, ROBOT_NAME + "/map")
        while IS_RUN and TASK != None:
            if GOAL_MANAGER.is_reached:
                # TODO Wait and evaluate possiability to get to final goal
                return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Goal(smach.State):
    def __init__(self):
        super(Go_Goal, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Go_Goal"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if IS_DUMMY_TEST:
            time.sleep(2)
            return 'done'
        
        GOAL_MANAGER.send_goal(TASK.goal_location, ROBOT_NAME + "/map")
        while IS_RUN and TASK != None:
            if GOAL_MANAGER.is_reached:
                return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Dock_Out(smach.State):

    def __init__(self):
        super(Dock_Out, self).__init__(outcomes=('abort', 'done', 'Single_AMR'), output_keys=["target"])

    def execute(self, userdata):
        global GATE_REPLY, TASK, CUR_STATE
        CUR_STATE = "Dock_Out"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if IS_DUMMY_TEST:
            time.sleep(2)
            next_state = TASK.task_flow[TASK.task_flow.index('Dock_Out')+1]
            if next_state == 'Go_Home':
                return 'done'
            elif next_state == 'Single_AMR':
                TASK = None
                rospy.loginfo("[fsm] task done")
                return 'Single_AMR'
        # Open gate
        PUB_GATE_CMD.publish(Bool(False))
        GATE_REPLY = None
        
        previous_state = TASK.task_flow[TASK.task_flow.index('Dock_Out')-1]
        transit_mode(previous_state, "Single_AMR")

        # if previous_state == "Single_Assembled":
        #     # TODO Dark Magic, Change footprint back to single AMR
        #     transit_mode(previous_state, "Single_AMR")
        # elif previous_state == "Double_Assembled":
        #     # switch_launch_single()
        #     transit_mode(previous_state, "Single_AMR")
        #     #switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        
        # Split goal into 5 waypoint , in order to avoid turning
        # TODO is there another way?
        for i in range(5):
            GOAL_MANAGER.send_goal((-(0.8/5.0)*i,0,0), ROBOT_NAME + "/shelf_center")
            while IS_RUN:
                if TASK == None:
                    rospy.logwarn('[fsm] task abort')
                    return 'abort'
                if GOAL_MANAGER.is_reached:
                    break
                time.sleep(TIME_INTERVAL)
        next_state = TASK.task_flow[TASK.task_flow.index('Dock_Out')+1]
        if next_state == 'Go_Home':
            return 'done'
        elif next_state == 'Single_AMR':
            TASK = None
            rospy.loginfo("[fsm] task done")
            return 'Single_AMR'
        
class Go_Home(smach.State):
    def __init__(self):
        super(Go_Home, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global TASK, CUR_STATE
        CUR_STATE = "Dock_Out"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if IS_DUMMY_TEST:
            time.sleep(2)
            TASK = None
            return 'done'

        GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
        while IS_RUN and TASK != None:
            if GOAL_MANAGER.is_reached:
                TASK = None
                rospy.loginfo("[fsm] task done")
                return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Single_Assembled(smach.State):

    def __init__(self):
        super(Single_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point','Single_AMR', 'Double_Assembled', 'Go_Goal'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global TASK, CUR_STATE
        CUR_STATE = "Single_Assembled"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        while IS_RUN:
            if TASK != None:
                try:
                    next_state = TASK.task_flow[TASK.task_flow.index('Single_Assembled')+1]
                except IndexError:
                    rospy.loginfo("[fsm] task done")
                    TASK = None
                else:
                    if next_state == 'Single_AMR' or next_state == 'Double_Assembled':
                        transit_mode("Single_Assembled", next_state)
                    return next_state
            time.sleep(TIME_INTERVAL)

class Double_Assembled(smach.State):

    def __init__(self):
        super(Double_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point','Single_AMR', 'Single_Assembled', 'Go_Goal'], output_keys=["target"])

    def execute(self, userdata):
        global TASK, CUR_STATE
        CUR_STATE = "Double_Assembled"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        while IS_RUN:
            if TASK != None:
                try:
                    next_state = TASK.task_flow[TASK.task_flow.index('Double_Assembled')+1]
                except IndexError:
                    rospy.loginfo("[fsm] task done")
                    TASK = None
                else:
                    if next_state == 'Single_AMR' or next_state == 'Single_Assembled':
                        transit_mode("Double_Assembled", next_state)
                    return next_state
            time.sleep(TIME_INTERVAL)

if __name__ == "__main__":
    rospy.init_node(name='fsm', anonymous=False)
    
    #-- Get parameters
    ROSLAUNCH_PATH_SINGLE_AMR = rospy.get_param(param_name="~roslaunch_path_single_amr")
    ROSLAUNCH_PATH_DOUBLE_AMR = rospy.get_param(param_name="~roslaunch_path_double_amr")
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")
    ROLE = rospy.get_param(param_name="~role")
    INIT_STATE = rospy.get_param(param_name="~init_state")
    TIME_INTERVAL = 1.0/rospy.get_param(param_name="~frequency")
    IS_DUMMY_TEST = rospy.get_param(param_name="~dummy_test")
    # Service
    rospy.Service(name="~task", service_class=StringSrv, handler=task_cb)
    
    # Subscriber
    rospy.Subscriber("/gate_reply", Bool, gate_reply_cb)
    GATE_REPLY = None
    
    # Publisher
    PUB_SEARCH_CENTER = rospy.Publisher("search_center", Point, queue_size = 1)
    PUB_GATE_CMD = rospy.Publisher("/gate_cmd", Bool, queue_size = 1)
    
    # Global variable 
    TASK = None # store task information
    ROSLAUNCH = None
    IS_RUN = True
    CUR_STATE = None
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
            state=Single_AMR(),
            transitions={'Find_Shelf': 'Find_Shelf',
                         'Single_Assembled': 'Single_Assembled',
                         'Double_Assembled': 'Double_Assembled',
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
                         'Single_AMR': 'Single_AMR', 
                         'done': 'Go_Home'}) # Successfully dock out

        smach.StateMachine.add(
            label='Single_Assembled',
            state=Single_Assembled(),
            transitions={'Dock_Out': 'Dock_Out',
                         'Go_Way_Point': 'Go_Way_Point',
                         'Single_AMR': 'Single_AMR',
                         'Double_Assembled': 'Double_Assembled',
                         'Go_Goal': 'Go_Goal'})

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
            state=Double_Assembled(),
            transitions={'Dock_Out': 'Dock_Out',
                         'Go_Goal': 'Go_Goal',
                         'Single_AMR': 'Single_AMR',
                         'Single_Assembled': 'Single_Assembled',
                         'Go_Way_Point': 'Go_Way_Point'})
    
    # Start checking IS_RUN
    t_check_running = threading.Thread(target=check_running)
    t_check_running.start()

    # FSM VIEWER
    sis = smach_ros.IntrospectionServer('smach_server', SM, '/ROOT')
    sis.start()
    SM.execute() # start FSM
    
    # Wait for threads to join
    t_check_running.join()
    sis.stop()
