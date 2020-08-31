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
    
    is_parameters_set = False
    while not is_parameters_set:
        try:
            robot_name = rospy.get_param("/unique_parameter/robot_name") # Find paramters in ros server
            is_parameters_set = True
        except:
            rospy.loginfo("robot_name are not found in rosparam server, keep on trying...")
            rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
            continue
    
    
    tf_request_list = [(robot_name+"/raw/map", robot_name+"/raw/odom"),
                       (robot_name+"/raw/odom", robot_name+"/raw/base_link")]
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        tf_list = []
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

