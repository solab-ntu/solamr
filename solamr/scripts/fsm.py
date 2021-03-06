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
from geometry_msgs.msg import Twist, Point, PoseStamped, Polygon, Point32, PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import String, Bool
# ROS service 
from solamr.srv import StringSrv
# Move Base
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID
import dynamic_reconfigure.client
# Custom import 
from rospy_tool.rospy_tool_lib import get_tf, vec_trans_coordinate, send_tf, normalize_angle, sign

class Task(object):
    '''
    Record the task information
    '''
    def __init__(self):
        self.mode = None # 'single_AMR' or 'double_AMR'
        self.shelf_location = None
        self.wait_location = None
        self.goal_location = None
        self.home_location = None
        self.task_flow = None

def check_running():
    '''
    This thread run at background, checking node shutdown and passing msg to peer robot.
    '''
    global IS_RUN, BASE_XYT
    while not rospy.is_shutdown():
        # Package formet string = "<CUR_STATE>|<x,y,t>"
        # Send current state
        send_str = str(CUR_STATE)
        
        # Send map->base_link to peer robot.
        base_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/base_link")
        if base_xyt != None:
            BASE_XYT = base_xyt
            send_str += "|"\
                     + str(BASE_XYT[0]) + ","\
                     + str(BASE_XYT[1]) + ","\
                     + str(BASE_XYT[2])
        # Send peer robot's map->base_link. This will be only use at 'double_AMR' DockOut.
        # Leader needs to inform where follower be after docking out.
        if MEASURE_PEER_XYT != None:
            send_str += "|"\
                     + str(MEASURE_PEER_XYT[0]) + ","\
                     + str(MEASURE_PEER_XYT[1]) + ","\
                     + str(MEASURE_PEER_XYT[2])
        PUB_CUR_STATE.publish(send_str)
        time.sleep(1)
    IS_RUN = False

def expand_footprint(polygon):
    '''
    Transform points -> line 
    Input : 
        polygon : 
            geometry_msgs/Polygon polygon
                geometry_msgs/Point32[] points
                    float32 x
                    float32 y
                    float32 z
    output: 
        pos_arr : PoseArray that publish to fakeObstacleLayer
    '''
    output_poseArray = PoseArray()
    for i in range(len(polygon.points)): # For every polygon edge
        # Get vertex end and start point
        vertex_start  = polygon.points[i]
        if i == len(polygon.points) - 1 : # Last line 
            vertex_end = polygon.points[0]
        else:
            vertex_end = polygon.points[i+1]
        
        #####################################
        ###   Bresenham's line algorithm  ###
        #####################################
        deltax = vertex_end.x - vertex_start.x
        deltay = vertex_end.y - vertex_start.y
        if deltax == 0: # To avoid inf slope
            deltax = 0.01

        deltaerr = abs(deltay / deltax) # slope
        error = 0.0 # No error at start
        resolution = 0.05 # m
        if deltaerr <= 1: # 0~45 degree
            y = vertex_start.y
            for i in range(int(round(abs(deltax) / resolution, 0))):
                x = vertex_start.x + i * resolution * sign(deltax)
                tmp_pose = Pose()
                tmp_pose.position.x = x 
                tmp_pose.position.y = y 
                output_poseArray.poses.append(tmp_pose) 
                error = error + deltaerr * resolution
                if error >= 0.5 * resolution:
                    y = y + sign(deltay) * resolution
                    error = error - 1 * resolution
        else:  # 45~90 degree
            x = vertex_start.x
            deltaerr = 1 / deltaerr
            for i in range(int(round(abs(deltay) / resolution, 0))):
                y = vertex_start.y + i * resolution * sign(deltay)
                tmp_pose = Pose()
                tmp_pose.position.x = x 
                tmp_pose.position.y = y 
                output_poseArray.poses.append(tmp_pose) 
                error = error + deltaerr * resolution
                if error >= 0.5 * resolution:
                    x = x + sign(deltax) * resolution
                    error = error - 1 * resolution
    return output_poseArray

def switch_launch(file_path):
    '''
    shutdown ROSLAUNCH if it's already launched, and switch on a new launch file
    '''
    global ROSLAUNCH
    if ROSLAUNCH != None:
        rospy.loginfo("[fsm] Shuting down roslaunch file")
        ROSLAUNCH.shutdown()
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    ROSLAUNCH = roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))
    rospy.loginfo("[fsm] Start launch file: " + file_path)
    ROSLAUNCH.start()

def change_footprint(L_2):
    '''
    Change move_base footprint size to L_2
    Argument:
        L_2: half of the robot perimeter length

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
    Every laser point inside radius r will be ignored by smart_obstacle_layer
    Argument:
        r - float: base_raduis
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
    Send initial pose to localize robot
    Argument:
        xyt - (float, float, float): coordinate relative to map frame
    '''
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
    Define what to do when transiting from one mode to another.
    '''
    global PEER_BASE_XYT, MEASURE_PEER_XYT, IS_STOPPING_ROBOT
    if   from_mode == "Single_AMR" and to_mode == "Single_Assembled":
        change_footprint(0.35)
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
            change_smart_layer_base_radius(0.9)
   
    elif from_mode == "Single_Assembled" and to_mode == "Single_AMR":
        change_footprint(SINGLE_AMR_L_2)
        change_smart_layer_base_radius(SINGLE_AMR_L_2*sqrt(2))
    
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
        (x_laser, y_laser) = vec_trans_coordinate((1.0, 0), 
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
    '''
    During roslaunch switch, we must asure robot stay still
    This thread will be awaken when robot is switching roslaunch.
    '''
    while IS_STOPPING_ROBOT:
        PUB_CMD_VEL.publish(Twist())    
        time.sleep(TIME_INTERVAL)

def rap_planner_homing():
    '''
    Send homing command to rap_planner 
    '''
    rospy.loginfo("[fsm] Calling rap planner homing")
    PUB_RAP_HOMING.publish("homing")

def task_cb(req):
    '''
    Callback function for rosservice, user may assign task by calling roservice.
    Argument:
        req.data - string
    '''
    global TASK

    # Task abort
    if req.data == "abort":
        TASK = None
        GOAL_MANAGER.cancel_goal()
        PUB_RAP_HOMING.publish("homing")
        PUB_CMD_VEL.publish(Twist())# Zero velocity
        return 'abort OK'
    
    # Jump from one state to another
    elif req.data[:3] == "jp2":
        # Jump to assign state
        task_tmp = Task()
        task_tmp.task_flow = [CUR_STATE, req.data[3:]]
        TASK = task_tmp
        return "jump to " + req.data[3:]

    # Reject Task if is busy
    if TASK != None:
        rospy.logerr("[fsm] Reject task, because I'm busy now.")
        return "Reject task, I'm busy now"
    
    # Load yaml from task_location.yaml
    req_list = req.data.split('_')
    # Get mode
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

        # TODO need test
        path = rospkg.RosPack().get_path('solamr') + "/params/task/task_flow.yaml"
        with open(path) as file:
            rospy.loginfo("[shelf_detector] Load yaml file from " + path)
            params = yaml.safe_load(file)
            task_tmp.task_flow = params[req.data]
            TASK = task_tmp
            return "OK"
    except OSError:
        rospy.logerr("[fsm] Yaml file not found at " + req.data)
        return "Yaml file not found"

def gate_reply_cb(data):
    '''
    Callback function for /gate_replay, indicate gate state
    '''
    global GATE_REPLY
    GATE_REPLY = data.data

def peer_robot_state_cb(data):
    '''
    Callback function for /peer_robot_state, get peer_robot_state, peer_robot_localization .etc.
    Argument:
        data.data - string
    '''
    global PEER_ROBOT_STATE, PEER_BASE_XYT, MEASURE_PEER_XYT
    data_list = data.data.split('|')
    PEER_ROBOT_STATE = data_list[0]
    
    # Get PEER_BASE_XYT
    try:
        xyt = data_list[1].split(',')
        PEER_BASE_XYT = (float(xyt[0]), float(xyt[1]), float(xyt[2]))
    except IndexError:
        pass
    
    # Get MEASURE_PEER_XYT
    try:
        xyt = data_list[2].split(',')
        MEASURE_PEER_XYT = (float(xyt[0]), float(xyt[1]), float(xyt[2]))
    except IndexError:
        pass
    
    # Project PEER_BASE_XYT to fake_obstacle_layer(costmap)
    if  PEER_BASE_XYT != None and\
        PEER_ROBOT_STATE != None and\
        PEER_ROBOT_STATE != "Double_Assembled" and\
        PEER_ROBOT_STATE != "Go_Double_Goal" and\
        CUR_STATE != "Double_Assembled" and\
        CUR_STATE != "Go_Double_Goal":
        
        # Get footprint polygon
        footprint_list = [(-SINGLE_AMR_L_2, -SINGLE_AMR_L_2), ( SINGLE_AMR_L_2, -SINGLE_AMR_L_2),
                          ( SINGLE_AMR_L_2,  SINGLE_AMR_L_2), (-SINGLE_AMR_L_2,  SINGLE_AMR_L_2)]
        poly = Polygon()
        for i in footprint_list:
            (x,y) = vec_trans_coordinate(i, PEER_BASE_XYT)
            poly.points.append(Point(x,y,0))

        # expand_footprint
        pose_array = expand_footprint(poly)

        # Publish obstacle
        PUB_FAKE_OBSTACLE.publish(pose_array)

class Goal_Manager(object):
    '''
    Send goal, Receive result, Cancel goal to move_base
    '''
    def __init__(self):
        self.pub_simple_goal = rospy.Publisher("/" + ROBOT_NAME + "/move_base_simple/goal", PoseStamped, queue_size = 1)
        self.pub_goal_cancel = rospy.Publisher("/" + ROBOT_NAME + "/move_base/cancel", GoalID, queue_size = 1)
        rospy.Subscriber("/" + ROBOT_NAME + "/move_base/result", MoveBaseActionResult, self.simple_goal_cb)
        self.is_reached = False # goal reach flag
        self.time_last_reached = time.time()
        self.goal = None
        self.xy_goal_tolerance = None # float
        self.yaw_goal_tolerance = None # float
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
    '''
    The first state fsm will be, and then transit to other useful state
    '''
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
    This state should be AMR alone, not combined.
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
    '''
    Find shelf location by apriltag
    '''
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
            #if shelf_xyt != None:
            #    return 'done'
            
            # Check goal reached or not
            if GOAL_MANAGER.is_reached:
                return 'done'
                '''
                try:
                    goal = TASK.shelf_location[TASK.shelf_location.index(goal)+1]
                except IndexError:
                    goal = TASK.shelf_location[0]
                '''
            # Send goal
            # GOAL_MANAGER.send_goal(goal, ROBOT_NAME + "/map", tolerance = (0.3, pi/6))
            GOAL_MANAGER.send_goal(goal, ROBOT_NAME + "/map")

            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Dock_Standby(smach.State):
    '''
    navigate to dockin standby point
    '''
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
                GOAL_MANAGER.cancel_goal()
                return 'done'

            # Update tag location
            # If chose point is not set, we won't consider laser center
            # Choose point only set, when tag and laser are both avaliable
            shelf_tag_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_" + ROBOT_NAME)
            shelf_laser_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/shelf_center")
            
            if shelf_tag_xyt != None and shelf_laser_xyt != None:
                tag_goal_xy = vec_trans_coordinate((-0.46, 0),
                                                   (shelf_tag_xyt[0], 
                                                    shelf_tag_xyt[1], 
                                                    shelf_tag_xyt[2] + pi/2))
                # Assign point to choose point
                choose_point = get_chosest_goal(shelf_laser_xyt, tag_goal_xy)
            elif  shelf_tag_xyt != None and shelf_laser_xyt == None:
                # If chose point is not set, we won't consider laser center
                # Send tag goal, 
                (x1,y1) = vec_trans_coordinate((-0.46, 0),
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
            send_tf((0.0, 0.0, 0.0), ROBOT_NAME + "/shelf_" + ROBOT_NAME, ROBOT_NAME + "/tag/shelf_center", z_offset=-0.46)
            # if laser is not valid use tag
            tag_xyt = get_tf(TFBUFFER, ROBOT_NAME +"/base_link", ROBOT_NAME +"/tag/shelf_center")
            if tag_xyt != None:
                PUB_SEARCH_CENTER.publish(Point(tag_xyt[0], tag_xyt[1], 0))
            time.sleep(TIME_INTERVAL)

        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Dock_In(smach.State):
    '''
    navigation to shelf center and combine with it.
    '''
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
            # can't just equeal 
            # if shelf_xyt != None:
            if shelf_xyt != None:
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
    '''
    After single_AMR combination, go to task way point and wait until another AMR
    has leave goal_location
    '''
    def __init__(self):
        super(Go_Way_Point, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Go_Way_Point"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        
        current_goal = TASK.wait_location[0]

        GOAL_MANAGER.send_goal(current_goal, ROBOT_NAME + "/map", tolerance = (0.3,  pi/6))
        while IS_RUN and TASK != None:
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
    '''
    Navigate to final goal location
    '''
    def __init__(self):
        super(Go_Goal, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global CUR_STATE
        CUR_STATE = "Go_Goal"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        
        seen_tag = False
        GOAL_MANAGER.is_reached = False
        
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
                    seen_tag = True
                    goal_xy = vec_trans_coordinate((0.7,0), (goal_xyt[0], goal_xyt[1], goal_xyt[2]-pi/2))
                    GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], goal_xyt[2]+pi/2), ROBOT_NAME + "/map")
                else:
                    if not seen_tag:
                        GOAL_MANAGER.send_goal(TASK.goal_location, ROBOT_NAME + "/map")

            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Go_Double_Goal(smach.State):
    '''
    After double_AMR combination, navigate to a serial of goals.
    '''
    def __init__(self):
        super(Go_Double_Goal, self).__init__(outcomes=['Dock_Out', 'abort', 'Double_Assembled', 'done'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global CUR_STATE, TASK
        CUR_STATE = "Go_Double_Goal"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        if ROLE == "leader":
            # Clear fake obstacle layer
            PUB_FAKE_OBSTACLE.publish(PoseArray())
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
                        # Prevent crab mode fail at last goal reached
                        if current_goal_set == TASK.goal_location[-1]: # TODO need test
                            PUB_RAP_HOMING.publish("homing")
                            rospy.loginfo("[fsm] wait homing for 2.5 sec")
                            time.sleep(2.5) # Wait homing!!
                            twist = Twist()
                            twist.linear.x = -0.01
                            rospy.loginfo("[fsm] Preventing crab mode fail")
                            PUB_CMD_VEL.publish(twist)
                            time.sleep(0.1)
                        if current_goal_set[1][2] == False: # Use diff mode
                            rospy.loginfo("[fsm] Publish fake obstacle to prevent A* oscillation")
                            obstacle_line_start = (2.82, 1.63)
                            obstacle_line_end = (2.82, 3.38)
                            poseArray = PoseArray()
                            for i in range(int((obstacle_line_end[1] - obstacle_line_start[1])/0.05)):
                                tmp_pose = Pose()
                                tmp_pose.position.x = obstacle_line_start[0]
                                tmp_pose.position.y = obstacle_line_start[1] + 0.05*i
                                poseArray.poses.append(tmp_pose)
                            PUB_FAKE_OBSTACLE.publish(poseArray)

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
    '''
    Leave shelf
    '''
    def __init__(self):
        super(Dock_Out, self).__init__(outcomes=('abort', 'done', 'Single_AMR'), output_keys=["target"])

    def execute(self, userdata):
        global GATE_REPLY, TASK, CUR_STATE
        
        CUR_STATE = "Dock_Out"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)
        
        twist = Twist()
        KP = 1.0
        #------------ single_AMR in-place rotation -------------# 
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
        elif TASK.mode == "double_AMR":
            PUB_RAP_HOMING.publish("homing")
            time.sleep(3) # Wait rap_controller homin
        
        # --------- Switch launch file ------------- #
        if TASK.mode == "single_AMR":
            transit_mode("Single_Assembled", "Single_AMR")
            time.sleep(2) # wait shelf become static
        elif TASK.mode == "double_AMR":
            transit_mode("Double_Assembled", "Single_AMR")

        # ------- Wait open gate --------- #
        GATE_REPLY = None
        PUB_GATE_CMD.publish(Bool(False))
        while GATE_REPLY != False:
            rospy.loginfo("[fsm] Waiting gate open...")
            PUB_SEARCH_CENTER.publish(Point(0, 0, 0))
            time.sleep(TIME_INTERVAL)
        
        # ------- Double AMR in-place rotation --------- #
        rospy.loginfo("[fsm] Start in-place rotation")
        if TASK.mode == "double_AMR":
            twist.linear.x = 0.0
            if ROLE == "leader":
                twist.angular.z = 0.6
            elif ROLE == "follower":
                twist.angular.z = -0.6
            t_start = rospy.get_rostime().to_sec()
            while IS_RUN and TASK != None and\
                rospy.get_rostime().to_sec() - t_start < (pi/2.0)/abs(twist.angular.z): # sec
                rospy.loginfo("[fsm] Dockout, inplace rotation: (" +\
                                str(rospy.get_rostime().to_sec() - t_start) + "/" +\
                                str((2*pi/3.0)/abs(twist.angular.z)) + ")")
                PUB_CMD_VEL.publish(twist)
                PUB_SEARCH_CENTER.publish(Point(0, 0, 0))
                time.sleep(TIME_INTERVAL)
        rospy.loginfo("[fsm] Finish in-place rotation")

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

        #------------  car2 wait 5 sec -------------# 
        if TASK.mode == "double_AMR" and ROLE == "follower":
            time.sleep(5)
        
        # Go to next state
        next_state = TASK.task_flow[TASK.task_flow.index('Dock_Out')+1]
        if next_state == 'Go_Home':
            return 'done'
        elif next_state == 'Single_AMR':
            TASK = None
            rospy.loginfo("[fsm] task done")
            return 'Single_AMR'
        
class Go_Home(smach.State):
    '''
    Task finish, single_AMR navigate to home_location
    '''
    def __init__(self):
        super(Go_Home, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        global TASK, CUR_STATE
        CUR_STATE = "Dock_Out"
        rospy.loginfo('[fsm] Execute ' + CUR_STATE)

        GOAL_MANAGER.is_reached = False
        # GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
        current_goal = TASK.home_location[0]
        if current_goal == TASK.home_location[-1]:
            # Last goal, need to precise
            GOAL_MANAGER.send_goal(current_goal, ROBOT_NAME + "/map")
        else:
            # Wait point, enlarge tolerance
            GOAL_MANAGER.send_goal(current_goal, ROBOT_NAME + "/map", tolerance = (0.3,  pi/6))
        
        while IS_RUN and TASK != None:
            if GOAL_MANAGER.is_reached:
                GOAL_MANAGER.is_reached = False
                try:
                    rospy.loginfo("[fsm] Finish current goal : " + str(current_goal))
                    current_goal = TASK.home_location[ TASK.home_location.index(current_goal) + 1 ]
                    GOAL_MANAGER.send_goal(current_goal, ROBOT_NAME + "/map")
                except IndexError:
                    TASK = None
                    rospy.loginfo("[fsm] task done")
                    return 'done'
            time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

        ''' Using tag to give goal will sometime assign a goal too near to obstacle, make global_planner fail to plan
        home_xyt = get_tf(TFBUFFER, ROBOT_NAME + "/map", ROBOT_NAME + "/home")
        if home_xyt != None:
            if ROBOT_NAME == "car1":
                goal_xy = vec_trans_coordinate((0.6, 0.75), (home_xyt[0], home_xyt[1], home_xyt[2]-pi/2))
            elif ROBOT_NAME == "car2":
                goal_xy = vec_trans_coordinate((0.6,-0.75), (home_xyt[0], home_xyt[1], home_xyt[2]-pi/2))
            GOAL_MANAGER.send_goal((goal_xy[0], goal_xy[1], home_xyt[2]-pi/2), ROBOT_NAME + "/map")
        '''
        # while IS_RUN and TASK != None:
        #     # Check goal reached
        #     if GOAL_MANAGER.is_reached:
        #         GOAL_MANAGER.is_reached = False
        #         TASK = None
        #         rospy.loginfo("[fsm] task done")
        #         return 'done'
        #     else:
        #         # Send goal
        #         GOAL_MANAGER.send_goal(TASK.home_location, ROBOT_NAME + "/map")
                

        #     time.sleep(TIME_INTERVAL)
        rospy.logwarn('[fsm] task abort')
        return 'abort'

class Single_Assembled(smach.State):
    '''
    Wait state, do nothing
    In this state means AMR is combined under single shelf
    '''
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
    '''
    Wait state, do nothing
    In this state means AMR is combined under double shelf
    '''
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
    SEND_GOAL_INTERVAL = 2.0 # sec
    SINGLE_AMR_L_2 = 0.22
    
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
    PUB_FAKE_OBSTACLE = rospy.Publisher("/" + ROBOT_NAME + "/move_base/global_costmap/fake_obstacle_layer/markedPoses", PoseArray, queue_size=1)
    if ROLE == "leader":
        PUB_CMD_VEL_PEER = rospy.Publisher("/" + ROBOT_PEER_NAME + "/cmd_vel", Twist, queue_size = 1)
    
    # Global variable 
    TASK = None # store task information
    ROSLAUNCH = None # roslaunch switch
    IS_RUN = True # check if ROS is still running
    CUR_STATE = None # current robot state
    BASE_XYT = None # localization 
    PEER_BASE_XYT = None # peer robot localization
    MEASURE_PEER_XYT = None
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
                         'Double_Assembled': 'Double_Assembled', }) # Dock succeeed, double assembled 

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
