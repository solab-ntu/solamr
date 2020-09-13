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

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()
