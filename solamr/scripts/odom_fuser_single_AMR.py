#!/usr/bin/env python
# System import
import rospy
import sys
from math import atan2,acos,sqrt,pi,sin,cos,tan
# ROS import 
import tf2_ros
import tf # conversion euler
import tf_conversions
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
# Custom import
from rospy_tool.rospy_tool_lib import get_tf, send_tf, vec_trans_coordinate, normalize_angle, cal_ang_distance
class Odom_Fuser_Single_AMR():
    def __init__(self):
        self.map_xyt = (None, None, None) #(x,y,theta)
        self.map_rtabmap_last = (None, None, None) #(x,y,theta)
        self.odom_xyt = (None, None, None) #(x,y,theta)
        self.marker1_xyt_last = (None, None, None)
        self.tag_list = []
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
        if self.odom_xyt[0] == None or self.map_xyt[0] == None:
            # if map it's not valid yet, ignore
            return 

        odom = vec_trans_coordinate((self.odom_xyt[0], self.odom_xyt[1]), (0, 0, self.map_xyt[2]))
        rho = update_xyt[2] - (self.odom_xyt[2] + self.map_xyt[2])
        rota_odom_x = cos(rho)*odom[0] - sin(rho)*odom[1]
        rota_odom_y = sin(rho)*odom[0] + cos(rho)*odom[1]
        
        self.map_xyt = (update_xyt[0] - rota_odom_x,
                        update_xyt[1] - rota_odom_y,
                        normalize_angle(self.map_xyt[2] + rho))
   
    def run_once(self):
        # Update TF
        map_rtabmap_xyt = get_tf(self.tfBuffer, ROBOT_NAME+"/raw/map", ROBOT_NAME+"/raw/odom", ignore_time = True)
        odom_xyt = get_tf(self.tfBuffer, ROBOT_NAME+"/raw/odom", ROBOT_NAME+"/raw/base_link", ignore_time = True)
        
        if odom_xyt != None:
            self.odom_xyt = odom_xyt
        if map_rtabmap_xyt != None:
            if map_rtabmap_xyt != self.map_rtabmap_last: # Rtabmap map -> odom has update
                self.map_xyt = map_rtabmap_xyt
                # TODO need test, offset rtabmap origin
                # self.map_xyt = (self.map_xyt[0] + 0.2, self.map_xyt[1] + 0.3, self.map_xyt[2])
                self.map_rtabmap_last = map_rtabmap_xyt
        
        if self.map_xyt[0] != None and self.odom_xyt[0] != None:
            return True 
        else:
            return False

    def publish(self):
        send_tf_z_offset(self.map_xyt, ROBOT_NAME + "/map", ROBOT_NAME + "/odom")
        send_tf_z_offset(self.odom_xyt, ROBOT_NAME + "/odom", ROBOT_NAME + "/base_link")

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


if __name__ == '__main__':
    rospy.init_node('odom_fuser_single_AMR',anonymous=False)
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")
    odom_fuser_single_AMR = Odom_Fuser_Single_AMR()
    rospy.Subscriber("/" + ROBOT_NAME + "/initialpose", PoseWithCovarianceStamped, odom_fuser_single_AMR.init_cb)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if odom_fuser_single_AMR.run_once():
            odom_fuser_single_AMR.publish()
        rate.sleep()
