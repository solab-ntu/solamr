#!/usr/bin/env python  
import rospy
import rospkg
import yaml
import tf_conversions
import tf2_ros
import tf
import tf.msg
from rospy_tool.rospy_tool_lib import get_tf, send_tf, vec_trans_coordinate, normalize_angle
from solamr.srv import StringSrv

def set_param_cb(req):
    '''
    '''
    global TF_RVIZ_LIST
    try:
        path = rospkg.RosPack().get_path('solamr') + "/params/tf_rviz/" + req.data + ".yaml"
        with open(path) as file:
            rospy.loginfo("[tf_rviz_forwarder] Load yaml file from " + path)
            params = yaml.safe_load(file)
            TF_RVIZ_LIST = params['tf_rviz_list']
            return "OK"
    except OSError:
        rospy.logerr("[tf_rviz_forwarder] Yaml file not found at " + req.data)
        return "Yaml file not found"

if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    # Get param from launch file
    ROBOT_NAME = rospy.get_param(param_name="~robot_name")

    # TF listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    # Publisher
    pub_tf = rospy.Publisher("/" + ROBOT_NAME + "/tf_rviz", tf.msg.tfMessage, queue_size = 10)
    
    # Get launch file parameters
    TF_RVIZ_LIST = rospy.get_param(param_name="~tf_rviz_list")
    
    # Service
    rospy.Service(name="~set_param", service_class=StringSrv, handler=set_param_cb)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        tf_list = []
        for i in TF_RVIZ_LIST:
            try:
                t = tfBuffer.lookup_transform(i[0], i[1], rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logdebug("[tf_rviz_forwarder] Can't get " + str(i[0] + "->" + str(i[1])) + ", " + str(e))
            else:
                tf_list.append(t)
        
        pub_tf.publish(tf_list)
        rate.sleep()

