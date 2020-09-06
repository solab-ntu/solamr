#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

# Python 
import yaml
import os
import time
from math import pi
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

SINGLE_AMR_LEN = 0.44


class Task(object):
    def __init__(self, mode,  tag_location,
                 wait_location = None, goal_location = None, home_location = None):
        self.mode = mode
        # self.shelf_id = shelf_id
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
    global TASK, EXTER_CMD

    if req.data == "abort":
        TASK = None
        # Cacelled all goal
        GOAL_MANAGER.action.cancel_all_goals()
        return 'abort OK'
    
    elif req.data == "gateopen":
        PUB_GATE_CMD.publish(Bool(False)) # Release the gate
        return 'gateopen OK'

    elif req.data == "gateclose":
        PUB_GATE_CMD.publish(Bool(True)) # Close the gate
        return 'gateclose OK'
    
    elif req.data == "dockout":
        EXTER_CMD = "dockout"
        return 'dockout OK'
    
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
                        # params['shelf_id'],
                        params['tag_location'],
                        params['wait_location'],
                        params['goal_location'],
                        params['home_location']
                        )
            return "OK"
    except OSError:
        rospy.logerr("[fsm] Yaml file not found at " + req.data)
        return "Yaml file not found"

def gate_reply_cb(data):
    '''
    Bool
    '''
    global GATE_REPLY
    GATE_REPLY = data.data


class Goal_Manager(object):
    def __init__(self):
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.action.wait_for_server()
        self.is_reached = False
        self.goal = None
        self.pub_simple_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 1)
    
    def check_reached(self):
        pass

    def send_goal(self, xyt, frame_id):
        self.is_reached = False
        # Cancel current goal 
        # self.action.cancel_all_goals()

        # self.goal = MoveBaseGoal()
        # self.goal.target_pose.header.frame_id = frame_id
        # self.goal.target_pose.header.stamp = rospy.Time.now()
        # self.goal.target_pose.pose.position.x = xyt[0]
        # self.goal.target_pose.pose.position.y = xyt[1]
        # self.goal.target_pose.pose.position.z = 0
        # quaternion = quaternion_from_euler(0.0, 0.0, xyt[2])
        # (self.goal.target_pose.pose.orientation.x,
        #  self.goal.target_pose.pose.orientation.y,
        #  self.goal.target_pose.pose.orientation.z,
        #  self.goal.target_pose.pose.orientation.w) = quaternion
        # self.action.send_goal(goal=self.goal, feedback_cb=self.feedback_cb, done_cb=self.reached_cb)
        self.goal = PoseStamped()
        self.goal.header.frame_id = frame_id
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = xyt[0]
        self.goal.pose.position.y = xyt[1]
        self.goal.pose.position.z = 0
        quaternion = quaternion_from_euler(0.0, 0.0, xyt[2])
        (self.goal.pose.orientation.x,
         self.goal.pose.orientation.y,
         self.goal.pose.orientation.z,
         self.goal.pose.orientation.w) = quaternion
        self.pub_simple_goal.publish(self.goal)

    def feedback_cb(self, feedback):
        pass
    
    def reached_cb(self,status, result):
        rospy.logwarn("GGGGGGGGGGGGGGGGGG REcive reached callback")
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
        GOAL_MANAGER.send_goal(TASK.tag_location, ROBOT_NAME + "/map")

        while not rospy.is_shutdown() and TASK != None:
            # Get apriltag shelf location
            if ROBOT_NAME == "car1":
                shelf_xyt = get_tf(TFBUFFER, "car1/map", "car1/shelf_one")
            elif ROBOT_NAME == "car2":
                shelf_xyt = get_tf(TFBUFFER, "car2/map", "car2/shelf_two")
            
            if shelf_xyt != None:
                return 'done'
                # goal_xyt = shelf_xyt # TODO add a distance
            
            # Check goal reached or not 
            if GOAL_MANAGER.is_reached:
                return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Dock_Standby(smach.State):

    def __init__(self):
        super(Go_Dock_Standby, self).__init__(outcomes=('abort','done'), output_keys=[])

    def execute(self, userdata):
        '''
        '''
        rospy.loginfo('[fsm] Execute Go_Dock_Standby')
        GOAL_MANAGER.is_reached = False
        while not rospy.is_shutdown() and TASK != None:
            # Check goal reached or not 
            if GOAL_MANAGER.is_reached:
                
                return 'done'
            
            # Get apriltag shelf location
            if ROBOT_NAME == "car1":
                shelf_xyt = get_tf(TFBUFFER, "car1/map", "car1/shelf_one")
            elif ROBOT_NAME == "car2":
                shelf_xyt = get_tf(TFBUFFER, "car2/map", "car2/shelf_two")
                       
            if shelf_xyt != None:
                # Update goal by camera apriltag
                (x1,y1) = vec_trans_coordinate((-0.5, 0), (shelf_xyt[0], shelf_xyt[1], shelf_xyt[2] + pi/2))
                GOAL_MANAGER.send_goal((x1, y1, shelf_xyt[2] + pi/2),
                                        ROBOT_NAME + "/map")
                
                # Send search center to shelf detector
                send_tf((0.0, 0.0, 0.0), "car2/shelf_two", "car2/shelf_two_center", z_offset=-0.3)
                xyt = get_tf(TFBUFFER, ROBOT_NAME +"/base_link", ROBOT_NAME +"/shelf_two_center")
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
        # global GATE_REPLY
        rospy.loginfo('[fsm] Execute Dock_In')
        # PUB_GATE_CMD.publish(Bool(True))
        PUB_GATE_CMD.publish(Bool(False))
        # GATE_REPLY = None
        # GOAL_MANAGER.action.cancel_all_goals()
        GOAL_MANAGER.is_reached = False
        while not rospy.is_shutdown() and TASK != None:
            # # Check goal reached or not
            # if GOAL_MANAGER.is_reached:
            #     return 'done'
            
            # if GATE_REPLY == True or GOAL_MANAGER.is_reached:
            if GOAL_MANAGER.is_reached:
                PUB_GATE_CMD.publish(Bool(True))
                # Get reply, Dockin successfully
                # rospy.loginfo("[fsm] Get gate reply: " + str(GATE_REPLY))
                if TASK.mode == 'single_AMR':
                    os.system("rostopic pub --once /" + ROBOT_NAME + "/move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.65,-0.65,0.0], [-0.65,0.65,0.0], [0.65,0.65,0.0], [0.65,-0.65,0.0]]'")
                    return 'Single_Assembled'
                elif TASK.mode == 'double_AMR':
                    switch_launch_double()
                    return 'Double_Assembled'
            else:
                xyt = get_tf(TFBUFFER, ROBOT_NAME + "/base_link", ROBOT_NAME + "/shelf_center")
                if xyt != None:
                    GOAL_MANAGER.send_goal(xyt, ROBOT_NAME + "/base_link")
                    # pub_simple_goal.publish()
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Way_Point(smach.State):
    def __init__(self):
        super(Go_Way_Point, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Go_Way_Point')
        GOAL_MANAGER.send_goal(TASK.wait_location, ROBOT_NAME + "/map")
        while not rospy.is_shutdown() and TASK != None:
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
        rospy.loginfo('[fsm] Execute Go_Goal')
        GOAL_MANAGER.send_goal(TASK.goal_location, ROBOT_NAME + "/map")
        while not rospy.is_shutdown() and TASK != None:
            if GOAL_MANAGER.is_reached:
                return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Dock_Out(smach.State):

    def __init__(self):
        super(Dock_Out, self).__init__(outcomes=('abort', 'done'), output_keys=["target"])

    def execute(self, userdata):
        '''
        '''
        global GATE_REPLY, EXTER_CMD
        rospy.loginfo('[fsm] Execute Dock_Out')
        PUB_GATE_CMD.publish(Bool(False)) # Release the gate
        # TODO Dark Magic, Change footprint back to single AMR
        os.system("rostopic pub --once /" + ROBOT_NAME + "/move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.22,-0.22,0.0], [-0.22,0.22,0.0], [0.22,0.22,0.0], [0.22,-0.22,0.0]]'")
        GATE_REPLY = None
        while not rospy.is_shutdown() and (TASK != None or EXTER_CMD == 'dockout'):
            EXTER_CMD = None
            xyt = get_tf(TFBUFFER, ROBOT_NAME + "/base_link", ROBOT_NAME + "/shelf_center")
            if xyt != None:
                send_tf((-0.8, 0, 0), ROBOT_NAME + "/shelf_center", ROBOT_NAME + "/shelf_center/dock_out")
                goal_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/base_link", ROBOT_NAME + "/shelf_center/dock_out")
                GOAL_MANAGER.send_goal(goal_xyt, ROBOT_NAME + "/base_link")
            
            if GATE_REPLY == False:
                if TASK.mode == "single_AMR":
                    pass
                elif TASK.mode == "double_AMR":
                    switch_launch_single()
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Home(smach.State):
    def __init__(self):
        super(Go_Home, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global TASK
        rospy.loginfo('[fsm] Execute Go_Home')
        GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
        while not rospy.is_shutdown() and TASK != None:
            if GOAL_MANAGER.is_reached:
                return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Single_Assembled(smach.State):

    def __init__(self):
        super(Single_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global EXTER_CMD
        rospy.loginfo('[fsm] Execute Single_Assembled')
        # TODO rosservice to dock out or accept a task
        while not rospy.is_shutdown():
            if TASK == None:
                # TODO listen to service to decide
                if EXTER_CMD == "dockout":
                    
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
    TIME_INTERVAL = 1.0/rospy.get_param(param_name="~frequency")

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
    EXTER_CMD = None
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
