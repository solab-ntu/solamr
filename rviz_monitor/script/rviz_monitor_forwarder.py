#!/usr/bin/env python  
import rospy
import tf_conversions
import tf2_ros
import tf
import tf.msg
from tf2_msgs.msg import TFMessage

def tf_callback(data):
    '''
    geometry_msgs/TransformStamped[] transforms
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        string child_frame_id
    geometry_msgs/Transform transform
        geometry_msgs/Vector3 translation
            float64 x
            float64 y
            float64 z
        geometry_msgs/Quaternion rotation
            float64 x
            float64 y
            float64 z
            float64 w
    '''
    pub_tf.publish(data)

if __name__ == '__main__':
    rospy.init_node('rviz_monitor_forwarder')

    # TF listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")

    # Publisher tf.msg.tfMessage
    pub_tf = rospy.Publisher("/server/tf_rviz", TFMessage, queue_size = 10)
    rospy.Subscriber("/" + ROBOT_NAME + "/tf_rviz", TFMessage, tf_callback)
    # Get launch file parameters
    TF_RVIZ_LIST = rospy.get_param(param_name="~tf_rviz_list")
    # rospy.loginfo(str(TF_RVIZ_LIST))
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # tf_list = []
        # for i in TF_RVIZ_LIST:
        #     try:
        #         t = tfBuffer.lookup_transform(i[0], i[1], rospy.Time())
        #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #         pass
        #         # rospy.logwarn("[rviz_monitor_forwarder] Can't get " + str(i[0] + "->" + str(i[1])) + ", " + str(e))
        #     else:
        #         tf_list.append(t)
        
        # pub_tf.publish(tf_list)
        rate.sleep()
