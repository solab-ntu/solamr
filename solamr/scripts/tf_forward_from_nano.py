#!/usr/bin/env python  
import rospy
import tf2_ros
import tf
import tf.msg
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('tf_forwarder')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")
    
    tf_request_list = [#(ROBOT_NAME+"/raw/map", ROBOT_NAME+"/raw/odom"),
                       #(ROBOT_NAME+"/raw/odom", ROBOT_NAME+"/raw/base_link"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/shelf_one"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/shelf_two"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/home"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/A_site"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/B_site")]

    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for i in tf_request_list:
            try:
                trans = tfBuffer.lookup_transform(i[0], i[1], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            else:
                # Reset time stamp
                trans.header.stamp = rospy.Time.now()
                # Get rid of /raw/
                s_list = i[0].split('/')
                trans.header.frame_id = s_list[0] + "/" + s_list[2]
                s_list = i[1].split('/')
                trans.child_frame_id = s_list[0] + "/" + s_list[2]
                # Z = 0.0
                trans.transform.translation.z = 0.05
                br.sendTransform(trans)
        rate.sleep()

