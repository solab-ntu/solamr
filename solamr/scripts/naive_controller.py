#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import tf # conversion euler
from geometry_msgs.msg import Twist # topic /cmd_vel
from math import atan2,acos,sqrt,pi,sin,cos,tan

#########################
### Global parameters ###
#########################
TOW_CAR_LENGTH = 0.93 # meter, Length between two cars
V_MAX = 0.3 # m/s, Max velocity
W_MAX = 0.6 # rad/s, MAX angular velocity
KP_crab = 0.8 # KP for crab mode, the bigger the value, the faster it will chase reg_ang
KP_diff = 1.5 # KP fro diff mode

class Navie_controller():
    def __init__(self,robot_name, role):
        rospy.init_node('naive_controller',anonymous=False)
        # Subscriber
        rospy.Subscriber("naive_cmd", Twist, self.cmd_cb)
        # Publisher
        self.pub_cmd_vel = rospy.Publisher("/"+robot_name+'/cmd_vel', Twist,queue_size = 1,latch=False)
        # Parameters
        self.robot_name = robot_name
        self.role = role
        # Tf listner
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.base_link_xyt = None # (x,y,theta)
        self.big_car_xyt = None  # (x,y,theta)
        self.theta = None
        # Kinematics 
        self.Vc = None
        self.Wc = None
        self.ref_ang = None
        self.V_leader = None
        self.W_leader = None
        self.V_follower = None
        self.W_follower = None
        # Flags
        self.mode = "crab" # "diff"
        self.is_need_publish = False
    
    def cmd_cb(self,data):
        '''
        Topic /<robot_name>/cmd_vel callback function
        Argument: 
            data - geometry_msgs/Twist
            geometry_msgs/Vector3 linear
                float64 x
                float64 y
                float64 z
            geometry_msgs/Vector3 angular
                float64 x - reg_ang
                float64 y - mode, 1 means differtial mode, 0 means crab mode
                float64 z
        '''
        self.Vc = data.linear.x
        self.Wc = data.angular.z
        self.ref_ang = data.angular.x
        if data.angular.y == 0:
            self.mode = "crab"
        else: 
            self.mode = "diff"
    
    def normalize_angle(self,angle):
        '''
        Make angle inside range [-pi, pi]
        Arguments:
            angle - flaot
        Return:
            float
        '''
        sign = None 
        if angle >= 0:
            sign = 1
        else: 
            sign = -1 
        ans = angle % (2* pi * sign)
        if ans < -pi: # [-2pi, -pi]
            ans += 2*pi
        elif ans > pi: # [pi, 2pi]
            ans -= 2*pi
        return ans
    
    def nearest_error(self, error):# TODO merge with normalize angle?
        '''
        Make error inside range [-pi, pi]
        '''
        if error > pi:
            error -= 2*pi
        elif error < -pi:
            error += 2*pi
        return error

    def iterate_once(self):
        '''
        call by main loop, execute every loop.
        '''
        # Do nothing if command is not set yet.
        if  self.Vc == None or\
            self.Wc == None or\
            self.ref_ang == None:
            return
        
        # Execute cmd 
        self.W_leader   = self.Wc
        self.W_follower = self.Wc
        self.V_leader   = self.Vc
        self.V_follower = -self.Vc# Follower go backward

        # Get tf 
        self.get_base_link()
        self.get_big_car()
        if self.base_link_xyt == None or self.big_car_xyt == None: # If tf is invalid
            return
        #
        self.theta = self.normalize_angle(self.normalize_angle(self.base_link_xyt[2]) - self.normalize_angle(self.big_car_xyt[2]))
       
        # Pick a nearest error nagle 
        if self.mode == "crab":
            self.W_leader += KP_crab*self.nearest_error(self.ref_ang - self.theta)
        elif self.mode == "diff":
            self.W_leader += KP_diff*self.nearest_error(self.ref_ang - self.theta)
        
        if self.mode == "crab":
            self.W_follower  += KP_crab*self.nearest_error(pi + self.ref_ang - self.theta)
        elif self.mode == "diff":
            self.W_follower  += KP_diff*self.nearest_error(pi - self.ref_ang - self.theta)
        
        # Saturation velocity, for safty
        self.V_leader = self.saturation(self.V_leader, V_MAX)
        self.W_leader = self.saturation(self.W_leader, W_MAX)
        self.V_follower = self.saturation(self.V_follower, V_MAX)
        self.W_follower = self.saturation(self.W_follower, W_MAX)
        
        # Debug print
        # rospy.loginfo("[naive controller] W_Leader = "+str(KP)+"*(" + str(self.ref_ang) + " - " + str(self.theta))
        if self.role == "leader":
            rospy.loginfo("[naive controller] Leader: (" + str(self.V_leader) + ", " +str(self.W_leader) + ")")
        elif self.role == "follower":
            rospy.loginfo("[naive controller] Follower: (" + str(self.V_follower) + ", " +str(self.W_follower) + ")")
        
        # Set publish flag
        self.is_need_publish = True

    def saturation(self, value, maximum):
        '''
        Let input value can't exceed maximum
        Arguments:
            value - float
            maximum - float
        Return:
            float : saturation value
        '''
        try:
            if abs(value) > maximum:
                if value > 0: # positive
                    value = maximum
                else: # nagative
                    value = -maximum
        except TypeError:
            pass
        finally:
            return value

    def is_same_sign(self, a, b):
        '''
        Check whether a,b are same sign 
        arguments:
            a - float/int
            b - float/int
        Return: 
            Bool - True means a,b have same sign, False means they don't
        '''
        if (a >= 0 and b >= 0) or (a < 0 and b < 0):
            return True
        else:
            return False
    
    def get_base_link(self):
        '''
        Get tf, map -> base_link 
        '''
        try:
            t = self.tfBuffer.lookup_transform(self.robot_name + "/map", self.robot_name + "/base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        else:
            quaternion = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.base_link_xyt = (t.transform.translation.x, t.transform.translation.y, euler[2])
    
    def get_big_car(self):
        '''
        Get tf, map -> center_big_car 
        '''
        try:
            t = self.tfBuffer.lookup_transform(self.robot_name + "/map", self.robot_name + "/center_big_car", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        else:
            quaternion = (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.big_car_xyt = (t.transform.translation.x, t.transform.translation.y, euler[2])

def main(args):
    # Get global parameters
    is_parameters_set = False
    while (not is_parameters_set and not rospy.is_shutdown() ):
        try:
            robot_name = rospy.get_param("/unique_parameter/robot_name") # Find paramters in ros server
            role       = rospy.get_param("/unique_parameter/role") # Find paramters in ros server
            is_parameters_set = True
        except:
            rospy.loginfo("robot_name are not found in rosparam server, keep on trying...")
            rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
            continue
    # Init naive controller
    navie_controller = Navie_controller(robot_name, role)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        navie_controller.iterate_once()
        if navie_controller.is_need_publish:
            if role == "leader" and navie_controller.V_leader != None and navie_controller.W_leader != None:
                cmd_vel = Twist()
                cmd_vel.linear.x  = navie_controller.V_leader
                cmd_vel.angular.z = navie_controller.W_leader
                navie_controller.pub_cmd_vel.publish(cmd_vel)
            elif role == "follower" and navie_controller.V_follower != None and navie_controller.W_follower != None:
                cmd_vel = Twist()
                cmd_vel.linear.x  = navie_controller.V_follower
                cmd_vel.angular.z = navie_controller.W_follower
                navie_controller.pub_cmd_vel.publish(cmd_vel)
            navie_controller.is_need_publish = False 
        rate.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass