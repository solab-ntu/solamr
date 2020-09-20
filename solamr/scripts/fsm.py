#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

# Python 
import yaml
import os
import time
from math import pi, sqrt, atan2
import threading
# ROS
import rospy
import roslib
import rospkg
import roslaunch
import tf2_ros
import tf # For tf.transformations.euler_from_quaternion(quaternion)
import smach
import smach_ros
# Ros message
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Point, PoseStamped, Polygon, Point32, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
# ROS service 
from solamr.srv import StringSrv
# from std_srvs.srv import Empty, EmptyResponse
# Move Base
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID
import dynamic_reconfigure.client

# Custom
from lucky_utility.ros.rospy_utility import get_tf, vec_trans_coordinate, send_tf, normalize_angle, sign

class Task(object):
    def __init__(self):
        self.mode = None
        self.shelf_location = None
        self.wait_location = None
        self.goal_location = None
        self.home_location = None
        self.task_flow = None

def check_running():
    '''
    This threading run at background, checking node shutdown and passing msg to peer 
    '''
    global IS_RUN, BASE_XYT
    while not rospy.is_shutdown():
        # Package formet string = "<CUR_STATE>|<x,y,t>"
        send_str = str(CUR_STATE)
        base_xyt_tmp = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/base_link")
        if base_xyt_tmp != None:
            BASE_XYT = base_xyt_tmp
            send_str += "|"\
                     + str(BASE_XYT[0]) + ","\
                     + str(BASE_XYT[1]) + ","\
                     + str(BASE_XYT[2])
        if MEASURE_PEER_XYT != None:
            send_str += "|"\
                     + str(MEASURE_PEER_XYT[0]) + ","\
                     + str(MEASURE_PEER_XYT[1]) + ","\
                     + str(MEASURE_PEER_XYT[2])
        PUB_CUR_STATE.publish(send_str)
        time.sleep(1)
    PUB_CMD_VEL.publish(Twist())
    IS_RUN = False

def switch_launch(file_path):
    global ROSLAUNCH
    rospy.loginfo("[fsm] Start launch file: " + file_path)
    if ROSLAUNCH != None:
        rospy.loginfo("[fsm] Shuting down single_AMR launch file")
        ROSLAUNCH.shutdown()
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    ROSLAUNCH = roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))
    ROSLAUNCH.start()

def change_footprint(L_2):
    '''
    L_2: half the footprint 
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    geometry_msgs/Polygon polygon
        geometry_msgs/Point32[] points
            float32 x
            float32 y
            float32 z
    '''
    footprint = Polygon()
    footprint.points = [Point32(-L_2,-L_2,0.0), Point32(-L_2, L_2,0.0),
                        Point32( L_2, L_2,0.0), Point32( L_2,-L_2,0.0)]
    PUB_GLOBAL_FOOTPRINT.publish(footprint)
    PUB_LOCAL_FOOTPRINT.publish(footprint)
    rospy.loginfo("[fsm] change footprint to " + str(L_2*2))

def change_smart_layer_base_radius(r):
    '''
    r = base_raduis
    '''
    rospy.wait_for_service("/" + ROBOT_NAME + "/move_base/global_costmap/smartobstacle_layer/set_parameters", 30.0)
    rospy.wait_for_service("/" + ROBOT_NAME + "/move_base/local_costmap/smartobstacle_layer/set_parameters", 30.0)
    client = dynamic_reconfigure.client.Client("/" + ROBOT_NAME + "/move_base/global_costmap/smartobstacle_layer", timeout=30)
    client.update_configuration({"base_radius": r})
    client = dynamic_reconfigure.client.Client("/" + ROBOT_NAME + "/move_base/local_costmap/smartobstacle_layer", timeout=30)
    client.update_configuration({"base_radius": r})
    rospy.loginfo("[fsm] change base_radius to " + str(r))

def send_initpose(xyt):
    '''
    '''
    # big_car_init_xyt
    initpose = PoseWithCovarianceStamped()
    initpose.header.stamp = rospy.Time.now()
    initpose.header.frame_id = ROBOT_NAME + "/map"
    initpose.pose.pose.position.x = xyt[0]
    initpose.pose.pose.position.y = xyt[1]
    quaternion = quaternion_from_euler(0.0, 0.0, xyt[2])
    (initpose.pose.pose.orientation.x,
     initpose.pose.pose.orientation.y,
     initpose.pose.pose.orientation.z,
     initpose.pose.pose.orientation.w) = quaternion
    PUB_INIT_POSE.publish(initpose)

def transit_mode(from_mode, to_mode):
    '''
    Define what to do, when transiting from one mode to another.
    '''
    global PEER_BASE_XYT, MEASURE_PEER_XYT, IS_STOPPING_ROBOT
    if   from_mode == "Single_AMR" and to_mode == "Single_Assembled":
        change_footprint(0.35)
        # change_footprint(0.45)
        # change_smart_layer_base_radius(0.45*sqrt(2)) # Some times can still see the shelf
        change_smart_layer_base_radius(0.7)

    elif from_mode == "Single_AMR" and to_mode == "Double_Assembled":
        switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)
        time.sleep(5)
        # Use last_base and PEER_BASE_XYT to calculate
        if ROLE == "leader":
            
            while PEER_BASE_XYT == None: # Wait PEER_BASE_XYT
                time.sleep(1)
                rospy.loginfo("[fsm] Waiting for peer robot to send last localization infomation")

            big_car_init_xyt =\
            ((BASE_XYT[0] + PEER_BASE_XYT[0]) / 2.0,
             (BASE_XYT[1] + PEER_BASE_XYT[1]) / 2.0,
             atan2(BASE_XYT[1] - PEER_BASE_XYT[1], BASE_XYT[0] - PEER_BASE_XYT[0]))
            PEER_BASE_XYT = None
            
            # big_car_init_xyt
            send_initpose(big_car_init_xyt)
            # 
            change_smart_layer_base_radius(0.9) # TODO need test
   
    elif from_mode == "Single_Assembled" and to_mode == "Single_AMR":
        change_footprint(0.22)
        change_smart_layer_base_radius(0.22*sqrt(2))
    
    elif from_mode == "Single_Assembled" and to_mode == "Double_Assembled":
        switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)

    elif from_mode == "Double_Assembled" and to_mode == "Single_AMR":
        # Record last localization result of big car
        last_base_leader = None
        last_base_follower = None

        # Create a stopping robot thread
        IS_STOPPING_ROBOT = True
        t_stopping_robot = threading.Thread(target=stopping_thread)
        t_stopping_robot.start()

        if ROLE == "leader":
            while last_base_leader == None or last_base_follower == None:
                last_base_leader   = get_tf(TFBUFFER, "carB/map", "car1/base_link")
                last_base_follower = get_tf(TFBUFFER, "carB/map", "car2/base_link")
                time.sleep(1)
            
        # Switch launch file
        switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        time.sleep(5) # Wait

        # Send peer last base 
        if ROLE == "leader":
            MEASURE_PEER_XYT = last_base_follower
            rospy.loginfo("[fsm] Send to follower its last localization : " + str(last_base_follower))
            send_initpose(last_base_leader)
        elif ROLE == "follower":
            while MEASURE_PEER_XYT == None: # Wait for last peer base
                rospy.loginfo("[fsm] Waiting for leader sending my localization")
                time.sleep(1)
            send_initpose(MEASURE_PEER_XYT)

        # join the stopping thread
        IS_STOPPING_ROBOT = False
        t_stopping_robot.join()
        rospy.loginfo("[fsm] Join stopping thread")

    elif from_mode == "Double_Assembled" and to_mode == "Single_Assembled":
        switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
    else:
        rospy.logerr("[fsm] Undefine transit mode : " + str(from_mode) + " -> " + str(to_mode))

def get_chosest_goal(laser_center,ref_point):
    '''
    laser_center - (x,y,t) by map->shelf_center
    ref_point -(x,y)
    '''
    # Set choose point !!
    min_distance = float('inf')
    output_xyt = None
    for i in range(4): # Which direction is best
        (x_laser, y_laser) = vec_trans_coordinate((1.2, 0), 
                                        (laser_center[0], 
                                         laser_center[1], 
                                         laser_center[2] + i*pi/2))
        dis_sq = (ref_point[0] - x_laser)**2 + (ref_point[1] - y_laser)**2 
        if dis_sq < min_distance:
            min_distance = dis_sq
            output_xyt = (x_laser, y_laser, laser_center[2] + i*pi/2 + pi)
    return output_xyt

def reconfig_rap_setting(setting):
    '''
    Setting: (xy_tolerance, yaw_tolerance, use_crab)
    '''
    client = dynamic_reconfigure.client.Client("rap_planner", timeout=5)
    client.update_configuration({"xy_tolerance": setting[0],
                                 "yaw_tolerance": setting[1],
                                 "use_crab": setting[2],
                                 })
    rospy.loginfo("[fsm] reconfig rap planner setting to " + str(setting))

def stopping_thread():
    while IS_STOPPING_ROBOT:
        # rospy.loginfo("[fsm] Inside stopping thread")
        PUB_CMD_VEL.publish(Twist())    
        time.sleep(0.1)

def rap_planner_homing():
    '''
    '''
    rospy.loginfo("[fsm] Calling rap planner homing")
    PUB_RAP_HOMING.publish("homing")
    # rospy.wait_for_service("/" + ROBOT_NAME + "/rap_planner/homing")
    # rospy.loginfo("[fsm] service DO EXIST!")
    # try:
    #     rospy.ServiceProxy("/" + ROBOT_NAME + "/rap_planner/homing", Empty)
    # except rospy.ServiceException as e:
    #     rospy.logerr("Service call failed: " + str(e))

def task_cb(req):
    '''
    Load yaml file and change parameter by req.data file path
    '''
    global TASK

    if req.data == "abort":
        TASK = None
        # Cacelled all goal
        GOAL_MANAGER.cancel_goal()
        PUB_RAP_HOMING.publish("homing")
        # Zero velocity
        PUB_CMD_VEL.publish(Twist())
        return 'abort OK'
    
    elif req.data[:3] == "jp2":
        # Jump to assign state
        task_tmp = Task()
        task_tmp.task_flow = [CUR_STATE, req.data[3:]]
        TASK = task_tmp
        return "jump to " + req.data[3:]

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

def peer_robot_state_cb(data):
    global PEER_ROBOT_STATE, PEER_BASE_XYT, MEASURE_PEER_XYT
    data_list = data.data.split('|')
    PEER_ROBOT_STATE = data_list[0]
    try:
        xyt = data_list[1].split(',')
        PEER_BASE_XYT = (float(xyt[0]), float(xyt[1]), float(xyt[2]))
    except IndexError:
        pass
    
    try:
        xyt = data_list[2].split(',')
        MEASURE_PEER_XYT = (float(xyt[0]), float(xyt[1]), float(xyt[2]))
    except IndexError:
        pass

class Goal_Manager(object):
    def __init__(self):
        self.pub_simple_goal = rospy.Publisher("/" + ROBOT_NAME + "/move_base_simple/goal", PoseStamped, queue_size = 1)
        self.pub_goal_cancel = rospy.Publisher("/" + ROBOT_NAME + "/move_base/cancel", GoalID, queue_size = 1)
        rospy.Subscriber("/" + ROBOT_NAME + "/move_base/result", MoveBaseActionResult, self.simple_goal_cb)
        self.is_reached = False
        self.time_last_reached = time.time()
        self.goal = None
        self.xy_goal_tolerance = None
        self.yaw_goal_tolerance = None
        self.time_last_goal = time.time()

    def send_goal(self, xyt, frame_id, tolerance=(0.12,0.06), z_offset = 0.1, ignore_tolerance = False):
        '''
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/Pose pose
            geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
            geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w
        '''
        self.is_reached = False
        # Check goal latch time
        if time.time() - self.time_last_goal < SEND_GOAL_INTERVAL: # sec
            return
        else:
            rospy.loginfo("[fsm] Send goal to move_base")
        
        if  (not ignore_tolerance) and \
            (self.xy_goal_tolerance  != tolerance[0] or\
             self.yaw_goal_tolerance != tolerance[1]):
            # Need to set new tolerance
            try:
                rospy.wait_for_service("/" + ROBOT_NAME + "/move_base/DWAPlannerROS/set_parameters", 5.0)
                client = dynamic_reconfigure.client.Client("/" + ROBOT_NAME + "/move_base/DWAPlannerROS", timeout=30)
                # rospy.wait_for_service("/" + ROBOT_NAME + "/move_base/TebLocalPlannerROS/set_parameters", 5.0)
                # client = dynamic_reconfigure.client.Client("/" + ROBOT_NAME + "/move_base/TebLocalPlannerROS", timeout=30)
                client.update_configuration({"xy_goal_tolerance": tolerance[0]})
                client.update_configuration({"yaw_goal_tolerance": tolerance[1]})
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("[fsm] DWAPlannerROS Service call failed: %s" % (e,))
            else:
                self.xy_goal_tolerance = tolerance[0]
                self.yaw_goal_tolerance = tolerance[1]
            
        self.goal = PoseStamped()
        self.goal.header.frame_id = frame_id
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = xyt[0]
        self.goal.pose.position.y = xyt[1]
        self.goal.pose.position.z = z_offset
        quaternion = quaternion_from_euler(0.0, 0.0, xyt[2])
        (self.goal.pose.orientation.x,
         self.goal.pose.orientation.y,
         self.goal.pose.orientation.z,
         self.goal.pose.orientation.w) = quaternion
        self.pub_simple_goal.publish(self.goal)
        self.time_last_goal = time.time()
  
    def simple_goal_cb(self,data):
        '''
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        actionlib_msgs/GoalStatus status
            uint8 PENDING=0
            uint8 ACTIVE=1
            uint8 PREEMPTED=2
            uint8 SUCCEEDED=3
            uint8 ABORTED=4
            uint8 REJECTED=5
            uint8 PREEMPTING=6
            uint8 RECALLING=7
            uint8 RECALLED=8
            uint8 LOST=9
            actionlib_msgs/GoalID goal_id
                time stamp
                string id
            uint8 status
            string text
        move_base_msgs/MoveBaseResult result
        '''
        if data.status.status == 3: # SUCCEEDED
            if time.time() - self.time_last_reached > 2.5: # sec, You can't reached goal so quickly
                rospy.loginfo("[fsm] simple_goal_cb : Goal Reached")
                self.time_last_reached = time.time()
                self.is_reached = True
            else:
                rospy.logwarn("[fsm] Received reached too often, Ignore reached msg")
        elif data.status.status == 2: # PREEMPTED
            pass
            # rospy.loginfo("[fsm] Goal has been preempted by a new goal")
        elif data.status.status == 4: # ABORTED
            # TODO do something
            rospy.logerr("[fsm] Goal has been aborted !!!")

    def cancel_goal(self):
        self.pub_goal_cancel.publish()

class Initial_State(smach.State):
    def __init__(self):
        super(Initial_State, self).__init__(outcomes=('Single_AMR', 'Single_Assembled', 'Double_Assembled'))

    def execute(self, userdata):
        if INIT_STATE == 'Single_AMR':
            PUB_GATE_CMD.publish(Bool(False))
            switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        elif INIT_STATE == 'Single_Assembled':
            PUB_GATE_CMD.publish(Bool(True))
            switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        elif INIT_STATE == 'Double_Assembled':
            PUB_GATE_CMD.publish(Bool(True))
            switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)
            if ROLE == "leader":
                time.sleep(5)
                change_smart_layer_base_radius(0.9)
        return INIT_STATE

class Single_AMR(smach.State):
    '''
    Waiting State, do nothing
    '''
    def __init__(self):
        super(Single_AMR, self).__init__(outcomes=('Find_Shelf','Single_Assembled','Double_Assembled','Go_Home', 'done'), output_keys=[])

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
        
        if TASK != None:
            goal = TASK.shelf_location[0]
        GOAL_MANAGER.is_reached = False
        while IS_RUN and TASK != None:
            # Get apriltag shelf location
            shelf_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_" + ROBOT_NAME, is_warn = False)
            if shelf_xyt != None:
                return 'done'
            
            # Check goal reached or not
            if GOAL_MANAGER.is_reached:
                try:
                    goal = TASK.shelf_location[TASK.shelf_location.index(goal)+1]
                except IndexError:
                    goal = TASK.shelf_location[0]
            
            # Send goal
            GOAL_MANAGER.send_goal(goal, ROBOT_NAME + "/map", tolerance = (0.3, pi/6))

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
        
        GOAL_MANAGER.is_reached = False
        choose_point = None
        while IS_RUN and TASK != None:
            # Check goal reached or not
            if GOAL_MANAGER.is_reached:
                return 'done'

            # Update tag location
            # If chose point is not set, we won't consider laser center
            # Choose point only set, when tag and laser are both avaliable
            shelf_tag_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_" + ROBOT_NAME)
            shelf_laser_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_center")
            
            if shelf_tag_xyt != None and shelf_laser_xyt != None:
                tag_goal_xy = vec_trans_coordinate((-0.66, 0),
                                                   (shelf_tag_xyt[0], 
                                                    shelf_tag_xyt[1], 
                                                    shelf_tag_xyt[2] + pi/2))
                # Assign point to choose point
                choose_point = get_chosest_goal(shelf_laser_xyt, tag_goal_xy)
            elif  shelf_tag_xyt != None and shelf_laser_xyt == None:
                # If chose point is not set, we won't consider laser center
                # Send tag goal, 
                (x1,y1) = vec_trans_coordinate((-0.66, 0),
                                                (shelf_tag_xyt[0], 
                                                shelf_tag_xyt[1], 
                                                shelf_tag_xyt[2] + pi/2))
                choose_point = (x1,y1,shelf_tag_xyt[2] + pi/2)
            elif shelf_tag_xyt == None and shelf_laser_xyt != None:
                # Only laser
                if choose_point != None:
                    choose_point = get_chosest_goal(shelf_laser_xyt, choose_point[:2])
            
            # Send goal
            if choose_point != None:
                GOAL_MANAGER.send_goal(choose_point, ROBOT_NAME + "/map")

            # Send search center to shelf detector
            send_tf((0.0, 0.0, 0.0), ROBOT_NAME + "/shelf_" + ROBOT_NAME, ROBOT_NAME + "/tag/shelf_center", z_offset=-0.50)
            laser_shelf_center_xyt = get_tf(TFBUFFER, ROBOT_NAME +"/base_link", ROBOT_NAME +"/shelf_center")
            if laser_shelf_center_xyt != None:
                # Use laser to publish search center instead of tag
                PUB_SEARCH_CENTER.publish(Point(laser_shelf_center_xyt[0], laser_shelf_center_xyt[1], 0))
            else:
                # if laser is not valid use tag
                tag_xyt = get_tf(TFBUFFER, ROBOT_NAME +"/base_link", ROBOT_NAME +"/tag/shelf_center")
                if tag_xyt != None:
                    PUB_SEARCH_CENTER.publish(Point(tag_xyt[0], tag_xyt[1], 0))
            
            time.sleep(TIME_INTERVAL)

        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Dock_In(smach.State):
    def __init__(self):
        super(Dock_In, self).__init__(outcomes=('Single_Assembled', 'Double_Assembled', 'abort'), output_keys=["target"])

    def execute(self, userdata):
        global CUR_STATE, GATE_REPLY
        CUR_STATE = "Dock_In"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)

        # Open gate
        GATE_REPLY = None
        PUB_GATE_CMD.publish(Bool(False))
        PUB_GATE_CMD.publish(Bool(True))
        # P controller
        KP_X = 0.3
        KP_T = 1.0
        VX_MAX = 0.1 # Max dockin velocity
        VX_MIN = 0.01 # Min dockin velocity
        XY_TOLERANCE = 0.01 # m
        twist = Twist()
        rho = float('inf')
        self.last_shelf_xyt = (None, None, None)
        while IS_RUN and TASK != None:
            # Send goal
            shelf_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/base_link", ROBOT_NAME + "/shelf_center")
            # if shelf_xyt != None:
            if shelf_xyt != None or shelf_xyt != self.last_shelf_xyt:
                rospy.loginfo("[fsm] Dockin error angle: " + str(atan2(shelf_xyt[1], shelf_xyt[0])))
                rho = sqrt(shelf_xyt[0]**2 + shelf_xyt[1]**2)
                twist.linear.x = KP_X*rho
                # Saturation velocity
                if abs(twist.linear.x) > VX_MAX:
                    twist.linear.x = VX_MAX * sign(twist.linear.x)
                elif abs(twist.linear.x) < VX_MIN:
                    twist.linear.x = VX_MIN * sign(twist.linear.x)
                if rho > 0.05: # To avoid strang things
                    twist.angular.z = KP_T*atan2(shelf_xyt[1], shelf_xyt[0])
                else:
                    twist.angular.z = 0.0
                self.last_shelf_xyt = shelf_xyt
            else:
                rospy.logerr("[fsm] Docking can't find shelf center")
                twist.linear.x = 0.0
                twist.linear.z = 0.0 
                # return 'abort'
            PUB_CMD_VEL.publish(twist)
            
            if GATE_REPLY == True:
                # Get reply, Dockin successfully
                # Send zero velocity
                PUB_CMD_VEL.publish(Twist())
                
                if TASK.mode == "single_AMR":
                    # SIngle AMR in-place rotation
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.6
                    t_start = rospy.get_rostime().to_sec()
                    while IS_RUN and TASK != None and\
                        rospy.get_rostime().to_sec() - t_start < pi/twist.angular.z: # sec
                        PUB_CMD_VEL.publish(twist)
                        time.sleep(TIME_INTERVAL)
                
                next_state = TASK.task_flow[TASK.task_flow.index('Dock_In')+1]
                transit_mode('Single_AMR', next_state)
                return next_state
            
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
        
        current_goal = TASK.wait_location[0]

        GOAL_MANAGER.send_goal(current_goal, ROBOT_NAME + "/map", tolerance = (0.3,  pi/6))
        while IS_RUN and TASK != None:
            # GOAL_MANAGER.send_goal(TASK.wait_location, ROBOT_NAME + "/map", tolerance = (0.3,  pi/6))
            if current_goal == TASK.wait_location[-1]: # Wait peer robot here
                if  PEER_ROBOT_STATE == "Single_Assembled" or\
                    PEER_ROBOT_STATE == "Single_AMR" or\
                    PEER_ROBOT_STATE == "Find_Shelf" or\
                    PEER_ROBOT_STATE == "Dock_In" or\
                    PEER_ROBOT_STATE == "Go_Dock_Standby":
                    rospy.loginfo("[fsm] Waiting for peer robot state ....")
                    time.sleep(1)
                    continue
            
            if GOAL_MANAGER.is_reached:
                GOAL_MANAGER.is_reached = False
                try:
                    rospy.loginfo("[fsm] Finish current goal : " + str(current_goal))
                    current_goal = TASK.wait_location[ TASK.wait_location.index(current_goal) + 1 ]
                except IndexError:
                    rospy.loginfo("[fsm] Finish all goal list!")
                    return 'done'
            else:
                GOAL_MANAGER.send_goal(current_goal, ROBOT_NAME + "/map", tolerance = (0.3,  pi/6))
            
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
        
        GOAL_MANAGER.send_goal(TASK.goal_location, ROBOT_NAME + "/map")
        
        while IS_RUN and TASK != None:
            # Tag navigation, TODO need to exchange shelf
            if  ROBOT_NAME == "car1":
                goal_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/B_site")
            elif ROBOT_NAME == "car2":
                goal_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/A_site")
            
            if GOAL_MANAGER.is_reached:
                GOAL_MANAGER.is_reached = False
                return 'done'
            else:
                if goal_xyt != None:
                    goal_xy = vec_trans_coordinate((0.7,0), (goal_xyt[0], goal_xyt[1], goal_xyt[2]-pi/2))
                    GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], goal_xyt[2]+pi/2), ROBOT_NAME + "/map")

            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Double_Goal(smach.State):
    def __init__(self):
        super(Go_Double_Goal, self).__init__(outcomes=['Dock_Out', 'abort', 'Double_Assembled', 'done'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global CUR_STATE, TASK
        CUR_STATE = "Go_Double_Goal"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if ROLE == "leader":
            # rap_planner_homing() # TODO is this must to have?
            # Change goal tolerance
            current_goal_set = TASK.goal_location[0]
            current_goal = current_goal_set[0]
            reconfig_rap_setting(current_goal_set[1])
            GOAL_MANAGER.is_reached = False
            seen_tag = False
        
        while IS_RUN and TASK != None:
            if ROLE == "leader":
                # Tag navigation
                '''
                goal_xyt = get_tf(TFBUFFER, "carB/map", ROBOT_NAME + "/B_site", is_warn = False)
                if goal_xyt != None and current_goal == TASK.goal_location[-1][0]: # Last goal
                    goal_xy = vec_trans_coordinate((1.5,0), (goal_xyt[0], goal_xyt[1], goal_xyt[2]-pi/2))
                    GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], goal_xyt[2]), "carB/map")
                    seen_tag = True
                '''
                # Wait goal reached
                if GOAL_MANAGER.is_reached:
                    GOAL_MANAGER.is_reached = False
                    try:
                        rospy.loginfo("[fsm] Finish current goal " + \
                            str(TASK.goal_location.index(current_goal_set)) + ": " + str(current_goal))
                        current_goal_set = TASK.goal_location[ TASK.goal_location.index(current_goal_set) + 1 ]
                        current_goal = current_goal_set[0]
                        reconfig_rap_setting(current_goal_set[1])
                    except IndexError:
                        rospy.loginfo("[fsm] Finish all goal list!")
                        rap_planner_homing() # rap_planner homing
                        GOAL_MANAGER.cancel_goal() # Cancel gaol
                        next_state = TASK.task_flow[TASK.task_flow.index('Go_Double_Goal')+1]
                        if next_state == TASK.task_flow[-1]: # For double_AMR_goal
                            TASK = None # finish mission
                        return next_state
                else:
                    if not seen_tag:
                        GOAL_MANAGER.send_goal(current_goal, "carB/map", ignore_tolerance = True)
            elif ROLE == "follower":
                # Listen to car1 state
                #if PEER_ROBOT_STATE == "Dock_Out": # If peer dockout, you dockout too
                #     return 'done'
                next_state = TASK.task_flow[TASK.task_flow.index('Go_Double_Goal')+1]
                if PEER_ROBOT_STATE == next_state:
                    return next_state
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
        
        twist = Twist()
        KP = 1.0
        #------------  in-place rotation -------------# 
        if TASK.mode == "single_AMR":
            error_theta = float('inf')
            while IS_RUN and TASK != None and abs(error_theta) > 0.01:
                # pid cmd_vel control, TODO choose a direction to dockout
                shelf_laser_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_center")
                base_link_xyt   = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/base_link")
                if shelf_laser_xyt != None and base_link_xyt != None:
                    # Chooose the point near the map center TODO don't pick inner direction
                    choose_point = get_chosest_goal(shelf_laser_xyt, (2.83, 1.15))
                    error_theta = normalize_angle( normalize_angle(choose_point[2]) - base_link_xyt[2])
                    rospy.loginfo("ERROR theta: " + str(error_theta))
                    twist.angular.z = KP*error_theta
                else:
                    # init search center
                    PUB_SEARCH_CENTER.publish(Point(0, 0, 0))
                PUB_CMD_VEL.publish(twist)
                time.sleep(TIME_INTERVAL)
            '''
            elif TASK.mode == "double_AMR" and ROLE == "leader":
                # NEED TODO test, give up this one, because rap planner will also publish cmd_vel 
                # at thsi point, tow cmd will collide
                
                twist_1 = Twist()
                twist_2 = Twist()
                error_1 = float('inf')
                error_2 = float('inf')
                while (abs(error_1) > 0.01 and abs(error_2) > 0.01):
                    carB_xyt = get_tf(TFBUFFER, "carB/map", "carB/base_link")
                    theta_1 = get_tf(TFBUFFER, "carB/map", "car1/base_link")
                    theta_2 = get_tf(TFBUFFER, "carB/map", "car2/base_link")
                    if theta_1 != None and theta_2 != None and carB_xyt != None:
                        error_1 = normalize_angle(carB_xyt[2] + pi/2) - theta_1[2]
                        error_2 = normalize_angle(carB_xyt[2] + pi/2) - theta_2[2]
                        rospy.loginfo("ERROR error_1: " + str(error_1))
                        rospy.loginfo("ERROR error_2: " + str(error_2))
                        twist_1.angular.z = KP*error_1
                        twist_2.angular.z = KP*error_2
                        PUB_CMD_VEL.publish(twist_1)
                        PUB_CMD_VEL_PEER.publish(twist_2)
            '''
        elif TASK.mode == "double_AMR":
            PUB_RAP_HOMING.publish("homing")
            time.sleep(3) # Wait rap_controller homing
        # Switch launch file
        if TASK.mode == "single_AMR":
            transit_mode("Single_Assembled", "Single_AMR")
            time.sleep(2) # wait shelf become static
        elif TASK.mode == "double_AMR":
            transit_mode("Double_Assembled", "Single_AMR")

        # Double AMR in-place rotation 
        rospy.loginfo("[fsm] Start in-place rotation")
        if TASK.mode == "double_AMR":
            twist.linear.x = 0.0
            if ROLE == "leader":
                twist.angular.z = 0.6
            elif ROLE == "follower":
                twist.angular.z = -0.6
            t_start = rospy.get_rostime().to_sec()
            while IS_RUN and TASK != None and\
                rospy.get_rostime().to_sec() - t_start < (2*pi/3.0)/abs(twist.angular.z): # sec
                rospy.loginfo("[fsm] Dockout, inplace rotation: (" +\
                                str(rospy.get_rostime().to_sec() - t_start) + "/"\
                                str((2*pi/3.0)/abs(twist.angular.z)) + ")")
                PUB_CMD_VEL.publish(twist)
                PUB_SEARCH_CENTER.publish(Point(0, 0, 0))
                time.sleep(TIME_INTERVAL)
        rospy.loginfo("[fsm] Finish in-place rotation")
        
        # Wait open gate
        GATE_REPLY = None
        PUB_GATE_CMD.publish(Bool(False))
        while GATE_REPLY != False:
            rospy.loginfo("[fsm] Waiting gate open...")
            PUB_SEARCH_CENTER.publish(Point(0, 0, 0))
            time.sleep(TIME_INTERVAL)

        #------------  dock out -------------# 
        t_start = rospy.get_rostime().to_sec()
        twist.linear.x = -0.1
        while IS_RUN and TASK != None and\
            rospy.get_rostime().to_sec() - t_start < 8.0: # sec
            xyt = get_tf(TFBUFFER, ROBOT_NAME + "/base_link", ROBOT_NAME + "/shelf_center")
            if xyt != None:
                twist.angular.z = KP*xyt[2] # KP*error
            else:
                twist.angular.z = 0.0
            PUB_CMD_VEL.publish(twist)
            time.sleep(TIME_INTERVAL)
        
        #------------  in-place rotation -------------# 
        twist.linear.x = 0.0
        twist.angular.z = 0.6
        t_start = rospy.get_rostime().to_sec()
        while IS_RUN and TASK != None and\
            rospy.get_rostime().to_sec() - t_start < pi/twist.angular.z: # sec
            PUB_CMD_VEL.publish(twist)
            time.sleep(TIME_INTERVAL)

        # Send zero velocity
        twist = Twist()
        PUB_CMD_VEL.publish(twist)

        # Go to next state
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

        GOAL_MANAGER.is_reached = False
        GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
        while IS_RUN and TASK != None:
            # Check goal reached
            if GOAL_MANAGER.is_reached:
                GOAL_MANAGER.is_reached = False
                TASK = None
                rospy.loginfo("[fsm] task done")
                return 'done'
            else:
                # GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
                # Send goal
                home_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/home")
                if home_xyt != None:
                    if ROBOT_NAME == "car1":
                        goal_xy = vec_trans_coordinate((0.6, 0.75), (home_xyt[0], home_xyt[1], home_xyt[2]-pi/2))
                    elif ROBOT_NAME == "car2":
                        goal_xy = vec_trans_coordinate((0.6,-0.75), (home_xyt[0], home_xyt[1], home_xyt[2]-pi/2))
                    GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], home_xyt[2]-pi/2), ROBOT_NAME + "/map")
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
        super(Double_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point','Single_AMR', 'Single_Assembled', 'Go_Double_Goal'], output_keys=["target"])

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
                    
                    if TASK.mode == "double_AMR":
                        # Wait for peer robot finish dock_in
                        if next_state == "Go_Double_Goal":
                            if PEER_ROBOT_STATE == "Double_Assembled" or PEER_ROBOT_STATE == "Go_Double_Goal":
                                return next_state
                        else:
                            return next_state
                    else:
                        return next_state
            time.sleep(TIME_INTERVAL)

if __name__ == "__main__":
    rospy.init_node(name='fsm', anonymous=False)
    
    #-- Get parameters
    ROSLAUNCH_PATH_SINGLE_AMR = rospy.get_param(param_name="~roslaunch_path_single_amr")
    ROSLAUNCH_PATH_DOUBLE_AMR = rospy.get_param(param_name="~roslaunch_path_double_amr")
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")
    ROBOT_PEER_NAME = rospy.get_param(param_name="~robot_peer_name")
    ROLE = rospy.get_param(param_name="~role")
    INIT_STATE = rospy.get_param(param_name="~init_state")
    TIME_INTERVAL = 1.0/rospy.get_param(param_name="~frequency")
    SEND_GOAL_INTERVAL = 2
    # IS_DUMMY_TEST = rospy.get_param(param_name="~dummy_test")
    # Service
    rospy.Service(name="~task", service_class=StringSrv, handler=task_cb)
    
    # Subscriber
    rospy.Subscriber("/gate_reply", Bool, gate_reply_cb)
    GATE_REPLY = None
    rospy.Subscriber( "/" + ROBOT_PEER_NAME + "/current_state", String, peer_robot_state_cb)
    PEER_ROBOT_STATE = None
    
    # Publisher
    PUB_SEARCH_CENTER = rospy.Publisher("search_center", Point, queue_size = 1)
    PUB_GATE_CMD = rospy.Publisher("/gate_cmd", Bool, queue_size = 1)
    PUB_CMD_VEL = rospy.Publisher("/" + ROBOT_NAME + "/cmd_vel", Twist, queue_size = 1)
    PUB_CUR_STATE = rospy.Publisher("/" + ROBOT_NAME + "/current_state", String, queue_size = 1)
    PUB_INIT_POSE = rospy.Publisher("/" + ROBOT_NAME + "/initialpose", PoseWithCovarianceStamped, queue_size = 1)
    PUB_GLOBAL_FOOTPRINT = rospy.Publisher("/" + ROBOT_NAME + "/move_base/local_costmap/footprint", Polygon, queue_size = 1)
    PUB_LOCAL_FOOTPRINT = rospy.Publisher("/" + ROBOT_NAME + "/move_base/global_costmap/footprint", Polygon, queue_size = 1)
    PUB_RAP_HOMING = rospy.Publisher("/" + ROBOT_NAME + "/rap_planner/homing", String, queue_size = 1)
    if ROLE == "leader":
        PUB_CMD_VEL_PEER = rospy.Publisher("/" + ROBOT_PEER_NAME + "/cmd_vel", Twist, queue_size = 1)
    
    # Global variable 
    TASK = None # store task information
    ROSLAUNCH = None
    IS_RUN = True
    CUR_STATE = None
    BASE_XYT = None
    PEER_BASE_XYT = None 
    MEASURE_PEER_XYT = None
    # Flag
    IS_STOPPING_ROBOT = None
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
                         'Go_Home': 'Go_Home',
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
            transitions={'abort': 'Find_Shelf', # For abort
                         'Single_Assembled': 'Single_Assembled', # Dock succeeed, single assembled 
                         'Double_Assembled': 'Double_Assembled', }) # # Dock succeeed, double assembled 

        smach.StateMachine.add(
            label='Dock_Out',
            state=Dock_Out(),
            transitions={'abort': 'Single_Assembled', # For abort
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
            label='Go_Double_Goal',
            state=Go_Double_Goal(),
            transitions={'abort': 'Double_Assembled',
                         'Double_Assembled': 'Double_Assembled',
                         'Dock_Out': 'Dock_Out',
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
                         'Go_Double_Goal': 'Go_Double_Goal',
                         'Single_AMR': 'Single_AMR',
                         'Single_Assembled': 'Single_Assembled',
                         'Go_Way_Point': 'Go_Way_Point'})
    
    # Start checking IS_RUN
    t_check_running = threading.Thread(target=check_running)
    t_check_running.start()

    # FSM VIEWER
    sis = smach_ros.IntrospectionServer("/" + ROBOT_NAME + '/smach_server', SM, '/' + ROBOT_NAME + '/ROOT')
    sis.start()
    SM.execute() # start FSM
    
    # Wait for threads to join
    t_check_running.join()
    sis.stop()
