#!/usr/bin/env python  
import rospy
import tf_conversions
import tf2_ros
import tf
import tf.msg
from lucky_utility.ros.rospy_utility import get_tf, send_tf, vec_trans_coordinate, normalize_angle

if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub_tf = rospy.Publisher("/tf_rviz", tf.msg.tfMessage, queue_size = 10)
    
    # Get launch file parameters
    ROBOT_NAME = rospy.get_param(param_name="~robot_name", default="car1")

    rate = rospy.Rate(10.0)

    tf_request_list = ((ROBOT_NAME + "/map", ROBOT_NAME + "/odom"),
                       (ROBOT_NAME + "/odom", ROBOT_NAME + "/base_link"),
                       (ROBOT_NAME + "/base_link", ROBOT_NAME + "/laser_front"),
                       (ROBOT_NAME + "/base_link", ROBOT_NAME + "/laser_rear"),

                       #(ROBOT_NAME + "map", ROBOT_NAME + "s_center_laser"),
                       #(ROBOT_NAME + "map", ROBOT_NAME + "center_peer"),
                       #(ROBOT_NAME + "map", ROBOT_NAME + "center_big_car"),
                       #(ROBOT_NAME + "s_center_laser", ROBOT_NAME + "s_standby_laser"),
                       ("carB/map", "carB/odom"),
                       ("carB/odom", "carB/base_link"),
                       ("carB/base_link", "car1/base_link"),
                       ("carB/base_link", "car2/base_link"),

                       (ROBOT_NAME + "/base_link", ROBOT_NAME + "/s_front"),
                       (ROBOT_NAME + "/s_front", ROBOT_NAME + "/s_standby_camera"),
                       (ROBOT_NAME + "/s_front", ROBOT_NAME + "/s_center_camera"))


    while not rospy.is_shutdown():
        tf_list = []
        for i in tf_request_list:
            try:
                t = tfBuffer.lookup_transform(i[0], i[1], rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            else:
                tf_list.append(t)
        
        pub_tf.publish(tf_list)
        rate.sleep()

