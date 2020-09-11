#!/usr/bin/env python  
import rospy
import tf_conversions
import tf2_ros
import tf
import tf.msg
from lucky_utility.ros.rospy_utility import get_tf, send_tf, vec_trans_coordinate, normalize_angle

if __name__ == '__main__':
    rospy.init_node('rviz_monitor_forwarder')

    # TF listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # Publisher
    pub_tf = rospy.Publisher("/server/tf_rviz", tf.msg.tfMessage, queue_size = 10)
    
    # Get launch file parameters
    TF_RVIZ_LIST = rospy.get_param(param_name="~tf_rviz_list")
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        tf_list = []
        for i in TF_RVIZ_LIST:
            try:
                t = tfBuffer.lookup_transform(i[0], i[1], rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
                # rospy.logdebug("[rviz_monitor_forwarder] Can't get " + str(i[0] + "->" + str(i[1])) + ", " + str(e))
            else:
                tf_list.append(t)
        
        pub_tf.publish(tf_list)
        rate.sleep()
