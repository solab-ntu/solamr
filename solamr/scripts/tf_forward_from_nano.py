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
    
    tf_request_list = [
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/shelf_car1"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/shelf_car2"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/home"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/A_site"),
                       (ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/B_site")]
                       #(ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/marker1"),
                       #(ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/marker2"),
                       #(ROBOT_NAME+"/raw/base_link", ROBOT_NAME+"/raw/marker3")]

    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    trans_last_list = [9999.0, 9999.0, 9999.0, 9999.0, 9999.0]
    while not rospy.is_shutdown():
        for i in range(len(tf_request_list)):
            try:
                trans = tfBuffer.lookup_transform(tf_request_list[i][0],
                                                  tf_request_list[i][1],
                                                  rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            else:
                # Check this is really a different tf
                if abs(trans_last_list[i] - trans.transform.translation.x ) < 0.00000001:
                    continue # ignore it 
                else:
                    trans_last_list[i] = trans.transform.translation.x
                    # Reset time stamp
                    trans.header.stamp = rospy.Time.now() # Substiutde nano timestamp
                    # Get rid of /raw/
                    trans.header.frame_id
                    s_list = trans.header.frame_id.split('/')
                    trans.header.frame_id = s_list[0] + "/" + s_list[2]
                    s_list = trans.child_frame_id.split('/')
                    trans.child_frame_id = s_list[0] + "/" + s_list[2]
                    # Z = 0.0
                    trans.transform.translation.z = 0.05
                    br.sendTransform(trans)
        rate.sleep()

