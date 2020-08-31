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
    rospy.init_node('ros_mqtt_bridge', anonymous=False)
    
    # init ros_mqtt_bridge
    ros_mqtt_bridge = Ros_mqtt_bridge(client_id="mqtt_master", broker_ip="broker.hivemq.com", port=1883, keepalive=10, clean_session=True)
    # ros_mqtt_bridge init publisher
    ros_mqtt_bridge.init_publisher(ros_topic = "ros_cmd_master", mqtt_topic = "mqtt_cmd", data_type = "geometry_msgs/Twist")

    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
