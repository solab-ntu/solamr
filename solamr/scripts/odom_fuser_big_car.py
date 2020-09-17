#!/usr/bin/env python
# System import
import rospy
import sys
from math import atan2,acos,sqrt,pi,sin,cos,tan
import time
# ROS import 
import tf2_ros
import tf # conversion euler
import tf_conversions
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
# Custom import
from lucky_utility.ros.rospy_utility import get_tf, send_tf, vec_trans_coordinate, normalize_angle

TWO_CAR_LENGTH = 0.93 # m

class Odom_fuser():
    def __init__(self):
        rospy.init_node('odom_fuser',anonymous=False)
        rospy.Subscriber("/car1/theta", Float64, self.car1_theta_cb)
        rospy.Subscriber("/car2/theta", Float64, self.car2_theta_cb)
        rospy.Subscriber("/car1/initialpose", PoseWithCovarianceStamped, self.init_cb)
        self.theta1 = None #
        self.theta2 = None
        
        # /car1/odom -> /car1/base_link
        self.car1_xyt = None # (x,y,theta)
        self.car1_xyt_last = None # 
        self.car1_map = None
        
        # /car2/odom -> /car2/base_link
        self.car2_xyt = None # (x,y,theta)
        self.car2_xyt_last = None # 
        self.car2_map = None
        
        # Output, # /carB/odom -> /carB/base_link
        self.carB_odom = [0,0,0]
        self.carB_map = [0,0,0]
        
        # For getting Tf
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

    def init_cb(self, data):
        '''
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/PoseWithCovariance pose
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
        float64[36] covariance
        '''
        rospy.loginfo("[odom_fuser_single_AMR] Received initialpose from RVIZ")
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (_,_,yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.update_global_localization((data.pose.pose.position.x, data.pose.pose.position.y, yaw))

    def update_global_localization(self, update_xyt):
        '''
        '''
        if self.carB_odom[0] == None or self.carB_map[0] == None:
            # if map it's not valid yet, ignore
            return 

        odom = vec_trans_coordinate((self.carB_odom[0], self.carB_odom[1]), (0, 0, self.carB_map[2]))
        rho = update_xyt[2] - (self.carB_odom[2] + self.carB_map[2])
        rota_odom_x = cos(rho)*odom[0] - sin(rho)*odom[1]
        rota_odom_y = sin(rho)*odom[0] + cos(rho)*odom[1]
        
        self.carB_map = (update_xyt[0] - rota_odom_x,
                        update_xyt[1] - rota_odom_y,
                        normalize_angle(self.carB_map[2] + rho))

    def car1_theta_cb(self, data):
        self.theta1 = data.data
    
    def car2_theta_cb(self, data):
        self.theta2 = normalize_angle(data.data + pi)

    def run_once(self):
        # Update TF
        car1_xyt = get_tf(self.tfBuffer, "car1/raw/odom", "car1/raw/base_link")
        car2_xyt = get_tf(self.tfBuffer, "car2/raw/odom", "car2/raw/base_link")
        car1_map = get_tf(self.tfBuffer, "car1/raw/map", "car1/raw/odom")
        car2_map = get_tf(self.tfBuffer, "car2/raw/map", "car2/raw/odom")
        # Check TF and thetas are valid
        if  car1_xyt == None or car2_xyt == None or\
            car1_map == None or car2_map == None or\
            self.theta1 == None or self.theta2 == None:
            time.sleep(1)
            return False
        else:
            self.car1_xyt = car1_xyt
            self.car2_xyt = car2_xyt
        
        # Update carB_map , TODO need TEST
        if car1_map != self.car1_map:
            rospy.loginfo("[odom_fuser_big_car] car1 get map->odom tf update")
            self.car1_map = car1_map
            (init_car1_x, init_car1_y) = vec_trans_coordinate(car1_xyt[:2], self.car1_map)
            init_car1_t = self.car1_map[2] + car1_xyt[2]
            rospy.loginfo("[odom_fuser_big_car] init_car1 = " + str((init_car1_x, init_car1_y, init_car1_t)))
            (init_carB_x, init_carB_y) = vec_trans_coordinate( (cos(-self.theta1)*(L/2),
                                                                sin(-self.theta1)*(L/2)),
                                         (init_car1_x, init_car1_y, init_car1_t) )
            init_carB_t = init_car1_t - self.theta1
            rospy.loginfo("[odom_fuser_big_car] init_carB = " + str((init_carB_x, init_carB_y, init_carB_t)))
            update_global_localization((init_carB_x, init_carB_y, init_carB_t))

        #if car2_map != self.car2_map:
        #    self.car2_map = car2_map
        # TODO CAR2 map localization 

        # Check last data
        if self.car1_xyt_last == None or self.car2_xyt_last == None:
            self.car1_xyt_last = self.car1_xyt
            self.car2_xyt_last = self.car2_xyt
            return False
        
        # Start calculate odometry
        (dx1, _) = vec_trans_coordinate((self.car1_xyt[0] - self.car1_xyt_last[0], 
                                         self.car1_xyt[1] - self.car1_xyt_last[1]),
                                         (0,0,-self.car1_xyt_last[2]))
        (dx2, _) = vec_trans_coordinate((self.car2_xyt[0] - self.car2_xyt_last[0], 
                                         self.car2_xyt[1] - self.car2_xyt_last[1]),
                                         (0,0,-self.car2_xyt_last[2]))
        dxB = (dx1*cos(self.theta1) - dx2*cos(self.theta2))/2.0
        dyB = (dx1*sin(self.theta1) - dx2*sin(self.theta2))/2.0
        vec_cars_x = dx2*cos(self.theta2) + dx1*cos(self.theta1) + TWO_CAR_LENGTH
        vec_cars_y = dx2*sin(self.theta2) + dx1*sin(self.theta1)
        dtB = atan2(vec_cars_y, vec_cars_x)

        # Increment add to carB_odom
        (odom_dx, odom_dy) = vec_trans_coordinate((dxB, dyB), (0,0,self.carB_odom[2]))
        self.carB_odom[0] += odom_dx
        self.carB_odom[1] += odom_dy
        self.carB_odom[2] += dtB

        # 
        self.car1_xyt_last = self.car1_xyt
        self.car2_xyt_last = self.car2_xyt
        return True 

    def publish(self):
        send_tf_z_offset(self.carB_map, "carB/map", "carB/odom")
        send_tf(self.carB_odom, "carB/odom", "carB/base_link")


def send_tf_z_offset(xyt, frame_id, child_frame_id):
    '''
    Argument:
        xyt: (x,y,theta)
    '''
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = xyt[0]
    t.transform.translation.y = xyt[1]
    t.transform.translation.z = 0.1
    q = tf_conversions.transformations.quaternion_from_euler(0, 0,xyt[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)


def main(args):
    # Init naive controller
    odom_fuser = Odom_fuser()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        odom_fuser.run_once()
        odom_fuser.publish() # Publish even it can't get tf
        rate.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
