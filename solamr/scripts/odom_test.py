#!/usr/bin/env python  
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,PolygonStamped, Point, Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path,Odometry
from math import pi,sin,cos,tan,acos,asin,atan2
import tf_conversions
import time # For time.time()
# Encoder
x_en = 0 
y_en = 0
t_en = 0 # Theta
time_en = None
en_odom = None
# IMU
x_imu = 0 
y_imu = 0
q_imu = 0
#
v_imu = 0 
w_imu = 0
time_imu = None
# Publish
en_is_need_pub  = False
imu_is_need_pub = False

def imu_callback(data):
    '''
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
            float64[9] orientation_covariance
        geometry_msgs/Vector3 angular_velocity
            float64 x
            float64 y
            float64 z
            float64[9] angular_velocity_covariance
        geometry_msgs/Vector3 linear_acceleration
            float64 x
            float64 y
            float64 z
        float64[9] linear_acceleration_covariance
    '''
    global time_imu,x_imu,y_imu,v_imu,w_imu,imu_is_need_pub,q_imu
    try:
        t = time.time()
        DT = t - time_imu
    except TypeError as e:
        time_imu = t
        return
    q_imu = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    angles = tf_conversions.transformations.euler_from_quaternion( [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w] )
    yaw = angles[2]
    v_imu += data.linear_acceleration.x*DT
    w_imu  = data.angular_velocity.z
    try: 
        r = v_imu/w_imu # divide by zero
    except ZeroDivisionError:# Go straigh
        # r = float('inf')
        x_imu = x_imu + v_imu*cos(yaw)*DT
        y_imu = y_imu + v_imu*sin(yaw)*DT
    else:
        # update x,y,theta
        x_imu = x_imu - r*sin(yaw) + r*sin(yaw + w_imu*DT)
        y_imu = y_imu + r*cos(yaw) - r*cos(yaw + w_imu*DT)
    imu_is_need_pub = True

def encoder_callback(data):
    '''
    data:
        geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
        geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
            float64 w
    '''
    global x_en,y_en,t_en,time_en,en_is_need_pub,en_odom
    
    # Encoder Twist
    try:
        t = time.time()
        DT = t - time_en
    except TypeError as e :
        time_en = t
        return
    time_en = t
    v = data.linear.x
    w = data.angular.z
    try: 
        r = v/w # divide by zero
    except ZeroDivisionError:# Go straigh
        # r = float('inf')
        x_en = x_en + v*cos(t_en)*DT
        y_en = y_en + v*sin(t_en)*DT
        # t_en = normalize_angle (t_en + w*DT)
    else:
        # update x,y,theta
        x_en = x_en - r*sin(t_en) + r*sin(t_en + w*DT)
        y_en = y_en + r*cos(t_en) - r*cos(t_en + w*DT)
        # t_en = normalize_angle (t_en + w*DT)
        t_en = t_en + w*DT
    
    #--- update encoder Twist -----#
    en_odom.twist.twist.linear  = data.linear
    en_odom.twist.twist.angular = data.angular
    en_odom.twist.covariance = [1e-09, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 1e-09, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 1e-09, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 1e-09, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 1e-09, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 1e-09]
    
    en_is_need_pub = True
    
if __name__ == '__main__':
    rospy.init_node('odom_test')
    # Publisher
    pub_imu_path      = rospy.Publisher("/rviz/imu_path"    , Path, queue_size = 1)
    pub_encoder_path  = rospy.Publisher("/rviz/encoder_path", Path, queue_size = 1)
    pub_imu_pose      = rospy.Publisher("/rviz/imu_pose"    , PoseStamped, queue_size = 1)
    # pub_encoder_pose  = rospy.Publisher("/rviz/encoder_pose", PoseStamped, queue_size = 1)
    pub_encoder_odom = rospy.Publisher("/odom_encoder", Odometry, queue_size = 1)
    # Subscriber
    imu_sub     = rospy.Subscriber("/zed/zed2/zed_node/imu/data", Imu, imu_callback)
    encoder_sub = rospy.Subscriber("STM32_twist", Twist, encoder_callback)
    # Encoder 
    path_encoder = Path()
    path_encoder.header.stamp = rospy.Time.now()
    path_encoder.header.frame_id = 'map'
    # Encoder Twist 
    en_odom = Odometry()
    en_odom.header.stamp = rospy.Time.now()
    en_odom.header.frame_id = "map"
    # Imu
    path_imu = Path()
    path_imu.header.stamp = rospy.Time.now()
    path_imu.header.frame_id = 'map'
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        if en_is_need_pub:
            #--- update encoder path ----#
            en_odom.header.stamp = rospy.Time.now()
            en_odom.pose.pose.position.x = x_en
            en_odom.pose.pose.position.y = y_en
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, t_en)
            en_odom.pose.pose.orientation.x = q[0]
            en_odom.pose.pose.orientation.y = q[1]
            en_odom.pose.pose.orientation.z = q[2]
            en_odom.pose.pose.orientation.w = q[3]
            #--- update encoder path 
            #path_encoder.poses.append(p)
            #pub_encoder_path.publish(path_encoder)
            pub_encoder_odom.publish(en_odom)
            #--- update encoder twist ----#
            en_is_need_pub = False

        if imu_is_need_pub:
            #--- update imu path ----# 
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "map"
            p.pose.position.x = x_imu
            p.pose.position.y = y_imu
            # q = tf_conversions.transformations.quaternion_from_euler(0, 0, q_imu)
            p.pose.orientation.x = q_imu[0]
            p.pose.orientation.y = q_imu[1]
            p.pose.orientation.z = q_imu[2]
            p.pose.orientation.w = q_imu[3]
            path_imu.poses.append(p)
            pub_imu_pose.publish(p)
            pub_imu_path.publish(path_imu)
            # 
            imu_is_need_pub = False 
            
        rate.sleep()