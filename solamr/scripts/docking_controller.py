#!/usr/bin/env python  
import rospy
import tf_conversions
import math
import tf2_ros
import tf
import tf.msg

import os # For dark magic
from std_msgs.msg import String, Bool
import time
from geometry_msgs.msg import PoseStamped,PolygonStamped, Point,Twist
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#### global variable
class Docking_controller():
    def __init__(self):
        # GLOBAL paramters loading from rosparam server, to determinant topic name of cmd_vel
        is_parameters_set = False
        while not is_parameters_set:
            try:
                self.robot_name = rospy.get_param("/unique_parameter/robot_name") # Find paramters in ros server
                is_parameters_set = True
            except:
                rospy.loginfo("robot_name are not found in rosparam server, keep on trying...")
                rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
                continue
        
        #---- Subscriber -----#
        self.sub_cmd        = rospy.Subscriber("docking_shelf_cmd", String, self.docking_shelf_cmd_callback)
        self.sub_gate_reply = rospy.Subscriber("gate_reply", Bool, self.gate_reply_cb)
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #----- Publisher ------# 
        self.pub_gate_cmd = rospy.Publisher("gate_cmd", Bool, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher("cmd_vel" , Twist, queue_size = 1)
        #pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
        #--- Publish move base goal ---#
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action.wait_for_server()
        
        #--- Private variable ----#
        self._cmd = "None"
        self.reply = None

        self.t_start_backword = None 
        #---- YC navigation 
        self.TF_LISTENER = tf.TransformListener()

    def iterate_once(self):
        if   self._cmd == "s_standby":
            # Find shelft by camera or by lidar
            # Publish simplpawn="true" goal 
            try:
                t_camera = self.tfBuffer.lookup_transform(self.robot_name + '/map', self.robot_name + '/s_standby_camera', rospy.Time())
                # trans = self.tfBuffer.lookup_transform('base_link', 'zed', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass 
            else:
                rospy.loginfo("[Docking_controlle] Camera seeing s_front, set s_stand_by move_base goal")
                #msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
                #msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
                '''
                goal = PoseStamped()
                goal.header.stamp = rospy.get_rostime()
                
                goal.header.frame_id = "map"
                goal.pose.position.x = t.transform.translation.x
                goal.pose.position.y = t.transform.translation.y 
                goal.pose.position.z = 0
                goal.pose.orientation.x = t.transform.rotation.x
                goal.pose.orientation.y = t.transform.rotation.y
                goal.pose.orientation.z = t.transform.rotation.z
                goal.pose.orientation.w = t.transform.rotation.w
                '''
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.robot_name+'/s_standby_camera'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = 0
                goal.target_pose.pose.position.y = 0
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = 0
                goal.target_pose.pose.orientation.y = 0
                goal.target_pose.pose.orientation.z = 0
                goal.target_pose.pose.orientation.w = 1
                self.action.send_goal(goal=goal, feedback_cb=self.feedback_cb, done_cb=self.standby_done_cb)
        elif self._cmd == "s_center":
            # Switch to docking navigation 
            # Publish simplpawn="true" goal 
            # Check IR status 
            if self.reply == True: # Finished!, door already closed 
                self.entering_done()
                self._cmd = "None"
                return 
            try:
                t_camera = self.tfBuffer.lookup_transform(self.robot_name+'/map', self.robot_name+'/s_center_laser', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass 
            else:
                rospy.loginfo("[Docking_controlle] Laser seeing s_center, set s_center move_base goal")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.robot_name+'/s_center_laser'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = 0
                goal.target_pose.pose.position.y = 0
                goal.target_pose.pose.position.z = 0
                goal.target_pose.pose.orientation.x = 0
                goal.target_pose.pose.orientation.y = 0
                goal.target_pose.pose.orientation.z = 0
                goal.target_pose.pose.orientation.w = 1
                self.action.send_goal(goal=goal, feedback_cb=self.feedback_cb, done_cb=self.standby_done_cb)
        elif self._cmd == "leave":
            # change footprint to small amr
            #fp = PolygonStamped()
            #fp.header.stamp = rospy.get_rostime()
            #fp.polygon.points.append( Point(-0.22,-0.22,0) )
            #fp.polygon.points.append( Point(-0.22,0.22,0) )
            #fp.polygon.points.append( Point(0.22,0.22,0) )
            #fp.polygon.points.append( Point(0.22,-0.22,0) )
            #pub_footprint.publish(fp)
            # Open locker
            # Set move_base goal to S_front
            if self.t_start_backword == None:
                try:
                    t_camera = self.tfBuffer.lookup_transform(self.robot_name+'/map', self.robot_name+'/s_center_laser', rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                else:
                    rospy.loginfo("[Docking_controlle] Laser seeing s_center, set s_center move_base goal")
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = self.robot_name+'/s_center_laser'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = 0
                    goal.target_pose.pose.position.y = 0
                    goal.target_pose.pose.position.z = 0
                    goal.target_pose.pose.orientation.x = 0
                    goal.target_pose.pose.orientation.y = 0
                    goal.target_pose.pose.orientation.z = 0
                    goal.target_pose.pose.orientation.w = 1
                    self.action.send_goal(goal=goal, feedback_cb=self.feedback_cb, done_cb=self.leave_heading_adj_done_cb)
            else:
                if time.time() - self.t_start_backword < 10: # Go backword
                    twist = Twist()
                    twist.linear.x = -0.1
                    self.pub_cmd_vel.publish( twist )
                    
                else:
                    rospy.loginfo("backword complete!")
                    self.pub_cmd_vel.publish( Twist() )
                    self._cmd = "None"
                    self.t_start_backword = None
            
            
        elif self._cmd == "cancel":
            pass

    def _pub_action_goal(ts, q, start_cb=None, done_cb=None, feedback_cb=None):
        if start_cb:
            start_cb()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.robot_name+'/map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = ts[0]
        goal.target_pose.pose.position.y = ts[1]
        goal.target_pose.pose.position.z = ts[2]
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.action.send_goal(goal=goal, done_cb=done_cb, feedback_cb=feedback_cb)

    def docking_shelf_cmd_callback(self, data):
        rospy.loginfo("[docking_controller] Get command "+data.data)
        self._cmd = data.data
        if self._cmd == "leave":
            # Open locker
            self.pub_gate_cmd.publish(Bool(False))
            # TODO Dark magic
            os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.22,-0.22,0.0], [-0.22,0.22,0.0], [0.22,0.22,0.0], [0.22,-0.22,0.0]]'")
        elif self._cmd == "s_center":
            # Open locker and wait 
            self.reply = None
            self.pub_gate_cmd.publish(Bool(True))
            
    def feedback_cb(self,feedback):
        pass

    def center_done_cb(self,status, result):
        rospy.loginfo("[docking_controller] Goal DONE!!!")
        self.entering_done()

    def leave_heading_adj_done_cb(self,status, result):
        self.action.cancel_all_goals()
        self.t_start_backword = time.time()
        

    def standby_done_cb(self,status, result):
        self.action.cancel_all_goals()
        self._cmd = "None"

    def gate_reply_cb(self,data):
        self.reply = data.data

    def entering_done(self):
        self.action.cancel_all_goals()
        # TODO Dark magic
        os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.37,-0.37,0.0], [-0.37,0.37,0.0], [0.37,0.37,0.0], [0.37,-0.37,0.0]]'")
        self._cmd = "None"

if __name__ == '__main__':
    rospy.init_node('docking_controller')
    docking_controller = Docking_controller()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        docking_controller.iterate_once()
        rate.sleep()



