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
    rospy.init_node('ros_mqtt_bridge_leader', anonymous=False)
    # init ros_mqtt_bridge
    ros_mqtt_bridge = Ros_mqtt_bridge(client_id="mqtt_leader", broker_ip="10.0.0.1", port=1883, keepalive=10, clean_session=True)
    ros_mqtt_bridge.init_publisher("/car2/cmd_vel", "p2p_cmd", "geometry_msgs/Twist")
    ros_mqtt_bridge.init_subscriber("/car2/theta", "car2_theta", "std_msgs/Float64")
    ros_mqtt_bridge.init_tf_subscriber(frame_id = "car2/raw/map", child_id = "car2/raw/odom",mqtt_topic="mqtt_tf_1")
    ros_mqtt_bridge.init_tf_subscriber(frame_id = "car2/raw/odom", child_id = "car2/raw/base_link",mqtt_topic="mqtt_tf_3")
    r = rospy.Rate(10) #call at 50HZ # Need to be faster than tf hz 
    while (not rospy.is_shutdown()):
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
