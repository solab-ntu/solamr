#!/usr/bin/env python
import rospy
import sys
# ROS msg and libraries
from geometry_msgs.msg import Twist
from std_msgs.msg import String
# MQTT
from ros_mqtt_bridge import  Ros_mqtt_bridge

def main(args):
    #----- Init node ------# 
    rospy.init_node('ros_mqtt_bridge_nano2rpi', anonymous=False)
    # Parameter
    ROBOT_NAME = rospy.get_param(param_name="~robot_name", default="car1")
    # init ros_mqtt_bridge
    ros_mqtt_bridge = Ros_mqtt_bridge(client_id=ROBOT_NAME+"/nano", broker_ip="192.168.1.1", port=1883,
                                      keepalive=10, clean_session=True)
    r = rospy.Rate(10) #call at 50HZ # Need to be faster than tf hz 
    while (not rospy.is_shutdown()):
        ros_mqtt_bridge.publish_tf(frame_id = ROBOT_NAME+"/map", 
                                   child_id = ROBOT_NAME+"/odom",
                                   mqtt_topic = "mqtt_tf_map2odom")
        ros_mqtt_bridge.publish_tf(frame_id = ROBOT_NAME+"/odom", 
                                   child_id = ROBOT_NAME+"/base_link",
                                   mqtt_topic = "mqtt_tf_odom2base_link")
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
