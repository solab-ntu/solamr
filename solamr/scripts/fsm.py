#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

# Python 
import yaml
import os
import time
from math import pi,sqrt, atan2
import threading
# ROS
import rospy
import roslib
import rospkg
import roslaunch
import tf2_ros
import tf # For tf.transformations.euler_from_quaternion(quaternion)
import actionlib
import smach
import smach_ros
# Ros message
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Point, PoseStamped, Polygon, Point32, PoseWithCovarianceStamped
from solamr.srv import StringSrv
from std_msgs.msg import String, Bool
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
    global IS_RUN
    while not rospy.is_shutdown():
        # Package formet string = "<CUR_STATE>|<x,y,t>"
        send_str = str(CUR_STATE)
        last_base = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/base_link")
        if last_base != None:
            send_str += "|"\
                     + str(last_base[0]) + ","\
                     + str(last_base[1]) + ","\
                     + str(last_base[2])
        if MEASURE_PEER_XYT != None:
            send_str += "|"\
                     + str(MEASURE_PEER_XYT[0]) + ","\
                     + str(MEASURE_PEER_XYT[1]) + ","\
                     + str(MEASURE_PEER_XYT[2])
        PUB_CUR_STATE.publish(send_str)
        # rospy.loginfo("[fsm] Send to leader my last localization : " + str(last_base))
        time.sleep(1)
    IS_RUN = False

def switch_launch(file_path):
    global ROSLAUNCH
    rospy.loginfo("[fsm] Start launch file: " + file_path)
    if not IS_DUMMY_TEST:
        if ROSLAUNCH != None:
            rospy.loginfo("[fsm] Shuting down single_AMR launch file")
            ROSLAUNCH.shutdown()
            # time.sleep(5) # TODO DO WE need it ?
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid=uuid)
        ROSLAUNCH = roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))
        ROSLAUNCH.start()
    else:
        pass

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
    rospy.wait_for_service("/" + ROBOT_NAME + "/move_base/global_costmap/smartobstacle_layer/set_parameters", 5.0)
    rospy.wait_for_service("/" + ROBOT_NAME + "/move_base/local_costmap/smartobstacle_layer/set_parameters", 5.0)
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
    global PEER_BASE_XYT, MEASURE_PEER_XYT
    if   from_mode == "Single_AMR" and to_mode == "Single_Assembled":
        change_footprint(0.45)
        change_smart_layer_base_radius(0.45*sqrt(2))

    elif from_mode == "Single_AMR" and to_mode == "Double_Assembled":
        # # Record the last base_link localization
        # last_base = None
        # while last_base == None:
        #     last_base = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/base_link")
        #     time.sleep(1)
        
        # if ROLE == "follower":
        #     # Send last_base to peer robot
        #     PUB_CUR_STATE.publish("peer_last_base:" +
        #                            str(last_base[0]) + ":" +
        #                            str(last_base[1]) + ":" + 
        #                            str(last_base[2]))
        #     rospy.loginfo("[fsm] Send to leader my last localization : " + str(last_base))

        switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)
        time.sleep(5)
        # Use last_base and PEER_BASE_XYT to calculate , TODO need test
        if ROLE == "leader":
            while PEER_BASE_XYT == None: # Wait PEER_BASE_XYT
                time.sleep(1)
                rospy.loginfo("[fsm] Waiting for peer robot to send last localization infomation")

            big_car_init_xyt =\
            ((last_base[0] + PEER_BASE_XYT[0]) / 2.0,
             (last_base[1] + PEER_BASE_XYT[1]) / 2.0,
             atan2(last_base[1] - PEER_BASE_XYT[1], last_base[0] - PEER_BASE_XYT[0]))
            PEER_BASE_XYT = None
            
            # big_car_init_xyt
            send_initpose(big_car_init_xyt)
            
    
    elif from_mode == "Single_Assembled" and to_mode == "Single_AMR":
        change_footprint(0.22)
        change_smart_layer_base_radius(0.22*sqrt(2))
    
    elif from_mode == "Single_Assembled" and to_mode == "Double_Assembled":
        switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)

    elif from_mode == "Double_Assembled" and to_mode == "Single_AMR":
        
        # Record last localization result of big car
        last_base_leader = None
        last_base_follower = None
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
            # PUB_CUR_STATE.publish("peer_last_base:" +
            #                        str(last_base_follower[0]) + ":" +
            #                        str(last_base_follower[1]) + ":" +
            #                        str(last_base_follower[2]))
            MEASURE_PEER_XYT = last_base_follower
            rospy.loginfo("[fsm] Send to follower its last localization : " + str(last_base_follower))
            send_initpose(last_base_leader)
        elif ROLE == "follower":
            while MEASURE_PEER_XYT == None: # Wait for last peer base
                time.sleep(1)
                rospy.loginfo("[fsm] Waiting for leader sending my localization")
            send_initpose(MEASURE_PEER_XYT)
    
    elif from_mode == "Double_Assembled" and to_mode == "Single_Assembled":
        switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
    else:
        rospy.logerr("[fsm] Undefine transit mode : " + str(from_mode) + " -> " + str(to_mode))
        pass

def get_chosest_goal(laser_center,ref_point):
    '''
    laser_center - (x,y,t) by map->shelf_center
    ref_point -(x,y)
    '''
    # Set choose point !!
    min_distance = float('inf')
    output_xyt = None
    for i in range(4): # Which direction is best
        (x_laser, y_laser) = vec_trans_coordinate((1.0, 0), 
                                        (laser_center[0], 
                                         laser_center[1], 
                                         laser_center[2] + i*pi/2))
        dis_sq = (ref_point[0] - x_laser)**2 + (ref_point[1] - y_laser)**2 
        if dis_sq < min_distance:
            min_distance = dis_sq
            output_xyt = (x_laser, y_laser, laser_center[2] + i*pi/2 + pi)
    return output_xyt

def task_cb(req):
    '''
    Load yaml file and change parameter by req.data file path
    '''
    global TASK

    if req.data == "abort":
        TASK = None
        # Cacelled all goal
        GOAL_MANAGER.cancel_goal()
        # Zero velocity
        twist = Twist()
        PUB_CMD_VEL.publish(twist)
        return 'abort OK'
    
    elif req.data[:3] == "jp2":
        # Jump to assign state
        #  = req.data.split(' ')
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
        self.goal = None
        # self.goal_xyt = [None, None, None]
        self.xy_goal_tolerance = None
        self.yaw_goal_tolerance = None
        self.time_last_goal = time.time()

    def send_goal(self, xyt, frame_id, tolerance=(0.12,0.06), z_offset = 0.1):
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
        # Check goal latch time
        if time.time() - self.time_last_goal < SEND_GOAL_INTERVAL: # sec
            return

        self.is_reached = False
        if  self.xy_goal_tolerance != tolerance[0] or\
            self.yaw_goal_tolerance != tolerance[1]:
            # Need to set new tolerance
            try:
                # TODO 
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
            rospy.loginfo("[fsm] Goal Reached")
            self.is_reached = True
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
            switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        elif INIT_STATE == 'Single_Assembled':
            PUB_GATE_CMD.publish(Bool(True))
            switch_launch(ROSLAUNCH_PATH_SINGLE_AMR)
        elif INIT_STATE == 'Double_Assembled':
            PUB_GATE_CMD.publish(Bool(True))
            switch_launch(ROSLAUNCH_PATH_DOUBLE_AMR)
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
        if IS_DUMMY_TEST:
            time.sleep(2)
            return 'done'

        # TODO use yaml.
        find_points = [(2.6, 0.4, -pi/2), (2, 1.2, pi), (2.6, 2.1, pi/2), (3.5, 1.2, 0.0)]
        if ROBOT_NAME == "car1":
            goal = find_points[3]
        elif ROBOT_NAME == "car2":
            goal = find_points[1]
        GOAL_MANAGER.is_reached = False
        while IS_RUN and TASK != None:
            # Check goal reached or not 
            if GOAL_MANAGER.is_reached:
                try:
                    goal = find_points[find_points.index(goal)+1]
                except IndexError:
                    goal = find_points[0]
                # Wait goal to calm down
                time.sleep(1)
            
            # Send a serial of goal
            GOAL_MANAGER.send_goal(goal, ROBOT_NAME + "/map", tolerance = (0.3, pi/6))
            
            # Get apriltag shelf location
            shelf_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_" + ROBOT_NAME, is_warn = False)

            if shelf_xyt != None:
                return 'done'
            
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
        
        GOAL_MANAGER.is_reached = False
        choose_point = None
        while IS_RUN and TASK != None:
            # Check goal reached or not
            if GOAL_MANAGER.is_reached:
                return 'done'

            # Update tag location
            # After choose point is set, we won't consider tag anymore
            # If chose point is not set, we won't consider laser center
            # Choose point only set, when tag and laser are both avaliable
            shelf_tag_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_" + ROBOT_NAME)
            shelf_laser_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_center")
            
            if shelf_tag_xyt != None and shelf_laser_xyt != None:
                tag_goal_xy = vec_trans_coordinate((-0.5, 0),
                                    (shelf_tag_xyt[0], shelf_tag_xyt[1], shelf_tag_xyt[2] + pi/2))
                # Assign point to choose point
                choose_point = get_chosest_goal(shelf_laser_xyt, tag_goal_xy)
            elif  shelf_tag_xyt != None and shelf_laser_xyt == None:
                # If chose point is not set, we won't consider laser center
                # Send tag goal, 
                (x1,y1) = vec_trans_coordinate((-0.5, 0),
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
            send_tf((0.0, 0.0, 0.0), ROBOT_NAME + "/shelf_" + ROBOT_NAME, ROBOT_NAME + "/tag/shelf_center", z_offset=-0.3)
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
        if IS_DUMMY_TEST:
            time.sleep(2)
            next_state = TASK.task_flow[TASK.task_flow.index('Dock_In')+1]
            return next_state

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
        while IS_RUN and TASK != None:
            # Send goal
            shelf_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/base_link", ROBOT_NAME + "/shelf_center")
            if shelf_xyt != None:
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
            else:
                rospy.logerr("[fsm] Docking fail")
                return 'abort'
            PUB_CMD_VEL.publish(twist)
            
            if GATE_REPLY == True:
                # Get reply, Dockin successfully
                # Send zero velocity
                twist = Twist()
                PUB_CMD_VEL.publish(twist)
                
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
        if IS_DUMMY_TEST:
            time.sleep(2)
            return 'done'

        GOAL_MANAGER.send_goal(TASK.wait_location, ROBOT_NAME + "/map", tolerance = (0.3,  pi/6))
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
        if TASK.mode == "single_AMR":
            GOAL_MANAGER.send_goal(TASK.goal_location, ROBOT_NAME + "/map")
        elif TASK.mode == "double_AMR":
            GOAL_MANAGER.send_goal(TASK.goal_location, "carB/map")
        
        while IS_RUN and TASK != None:
            # Tag navigation, TODO need to exchange shelf
            if TASK.mode == "single_AMR":
                if  ROBOT_NAME == "car1":
                    goal_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/B_site")
                elif ROBOT_NAME == "car2":
                    goal_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/A_site")
            elif TASK.mode == "double_AMR":
                goal_xyt = get_tf(TFBUFFER, "carB/map", ROBOT_NAME + "/B_site")
            
            if goal_xyt != None:
                goal_xy = vec_trans_coordinate((1,0), (goal_xyt[0], goal_xyt[1], goal_xyt[2]-pi/2))
                GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], goal_xyt[2]+pi/2), ROBOT_NAME + "/map")

            if GOAL_MANAGER.is_reached:
                return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Double_Goal(smach.State):
    def __init__(self):
        super(Go_Double_Goal, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Go_Double_Goal"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        
        # Send goal 
        if ROLE == "leader":
            GOAL_MANAGER.send_goal(TASK.goal_location, "carB/map")
        
        while IS_RUN and TASK != None:
            if ROLE == "leader":
                # Tag navigation
                goal_xyt = get_tf(TFBUFFER, "carB/map", ROBOT_NAME + "/B_site")
                if goal_xyt != None:
                    goal_xy = vec_trans_coordinate((1,0), (goal_xyt[0], goal_xyt[1], goal_xyt[2]-pi/2))
                    GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], goal_xyt[2]+pi/2), "carB/map")

                # Wait goal reached
                if GOAL_MANAGER.is_reached:
                    return 'done'
            elif ROLE == "follower":
                # Listen to car1 state
                if PEER_ROBOT_STATE == "Dock_Out": # If peer dockout, you dockout too
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
        time.sleep(2) # wait shelf become static
        GATE_REPLY = None
        
        if TASK.mode == "single_AMR":
            transit_mode("Single_Assembled", "Single_AMR")
        elif TASK.mode == "double_AMR":
            transit_mode("Double_Assembled", "Single_AMR")
        
        PUB_SEARCH_CENTER.publish(Point(0, 0, 0))
        twist = Twist()
        KP = 1.0
        #------------  in-place rotation -------------# 
        error_theta = float('inf')
        while IS_RUN and TASK != None:
            # pid cmd_vel control, TODO choose a direction to dockout
            shelf_laser_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_center")
            base_link_xyt   = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/base_link")
            if shelf_laser_xyt != None and base_link_xyt != None and\
                error_theta > 0.01:
                # Chooose the point near the map center
                choose_point = get_chosest_goal(shelf_laser_xyt, (2.83, 1.15))
                error_theta = normalize_angle( normalize_angle(choose_point[2]) - base_link_xyt[2])
                rospy.loginfo("ERROR theta: " + str(error_theta))
                twist.angular.z = KP*error_theta
            PUB_CMD_VEL.publish(twist)
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
        # TODO 


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
        if IS_DUMMY_TEST:
            time.sleep(2)
            TASK = None
            return 'done'

        GOAL_MANAGER.is_reached = False
        GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
        while IS_RUN and TASK != None:
            print ("go home")
            # Check goal reached
            if GOAL_MANAGER.is_reached:
                TASK = None
                rospy.loginfo("[fsm] task done")
                return 'done'
            
            # TODO using tag??????
            # GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
            # Send goal
            home_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/home")
            if home_xyt != None:
                if ROBOT_NAME == "car1":
                    goal_xy = vec_trans_coordinate((1, 1), (home_xyt[0], home_xyt[1], home_xyt[2]-pi/2))
                elif ROBOT_NAME == "car2":
                    goal_xy = vec_trans_coordinate((1,-1), (home_xyt[0], home_xyt[1], home_xyt[2]-pi/2))
                GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], home_xyt[2]+pi/2), ROBOT_NAME + "/map")
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
                        if PEER_ROBOT_STATE == "Double_Assembled" or PEER_ROBOT_STATE == "Go_Double_Goal":
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
    IS_DUMMY_TEST = rospy.get_param(param_name="~dummy_test")
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
    
    # Global variable 
    TASK = None # store task information
    ROSLAUNCH = None
    IS_RUN = True
    CUR_STATE = None
    PEER_BASE_XYT = None 
    MEASURE_PEER_XYT = None
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
            label='Go_Double_Goal',
            state=Go_Double_Goal(),
            transitions={'abort': 'Double_Assembled',
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
