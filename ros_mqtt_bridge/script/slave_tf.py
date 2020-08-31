#!/usr/bin/env python
import rospy
import sys
# ROS msg and libraries
from geometry_msgs.msg import Twist# Global path
from std_msgs.msg import String
# MQTT
from ros_mqtt_bridge import  Ros_mqtt_bridge

def main(args):
    #----- Init node ------#
    rospy.init_node('ros_mqtt_bridge', anonymous=False)
    
    # init ros_mqtt_bridge
    ros_mqtt_bridge = Ros_mqtt_bridge(client_id="mqtt_slave", broker_ip="broker.hivemq.com", port=1883, keepalive=10, clean_session=True)
    # ros_mqtt_bridge Subscirber
    # ros_mqtt_bridge.init_subscriber(ros_topic = "ros_cmd_slave", mqtt_topic = "mqtt_cmd", data_type = "geometry_msgs/Twist")
    ros_mqtt_bridge.init_tf_subscriber(frame_id = "map", child_id = "base_link_master",mqtt_topic="mqtt_tf")

    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
