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
    rospy.init_node('ros_mqtt_bridge_rpi2nano', anonymous=False)
    # Parameter
    ROBOT_NAME = rospy.get_param(param_name="~robot_name", default="car1")
    # init ros_mqtt_bridge
    ros_mqtt_bridge = Ros_mqtt_bridge(client_id = ROBOT_NAME+"/rpi", broker_ip="192.168.1.1", port=1883, 
                                      keepalive=10, clean_session=True)
    ros_mqtt_bridge.init_tf_subscriber(frame_id = ROBOT_NAME+"/raw/map",
                                       child_id = ROBOT_NAME+"/raw/odom",
                                       mqtt_topic="mqtt_tf_map2odom")
    ros_mqtt_bridge.init_tf_subscriber(frame_id = ROBOT_NAME+"/raw/odom",
                                       child_id = ROBOT_NAME+"/raw/base_link",
                                       mqtt_topic="mqtt_tf_odom2base_link")
    r = rospy.Rate(10) #call at 50HZ # Need to be faster than tf hz 
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
