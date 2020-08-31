#!/usr/bin/env python
import rospy
import sys
# MQTT
from ros_mqtt_bridge import  Ros_mqtt_bridge

def main(args):
    #----- Init node ------# 
    rospy.init_node('ros_mqtt_bridge_follower', anonymous=False)
    
    # init ros_mqtt_bridge
    ros_mqtt_bridge = Ros_mqtt_bridge(client_id="mqtt_follower", broker_ip="10.0.0.1", port=1883, keepalive=10, clean_session=True)
    # ros_mqtt_bridge init publisher
    
    ros_mqtt_bridge.init_subscriber("/car2/rap_cmd", "mqtt_topic", "geometry_msgs/Twist")
    ros_mqtt_bridge.init_subscriber("/car2/cmd_vel", "p2p_cmd", "geometry_msgs/Twist")
    # ros_mqtt_bridge.init_publisher("/car2/zed2/zed_node/odom", "mqtt_odom", "nav_msgs/Odometry")
    r = rospy.Rate(10) #call at 10HZ
    while (not rospy.is_shutdown()):
        ros_mqtt_bridge.publish_tf(frame_id = "car2/map", child_id = "car2/odom", mqtt_topic = "mqtt_tf_1")
        ros_mqtt_bridge.publish_tf(frame_id = "car2/odom", child_id = "car2/base_link", mqtt_topic = "mqtt_tf_3")
        ros_mqtt_bridge.publish_tf(frame_id = "car2/map", child_id = "car2/center_big_car", mqtt_topic = "mqtt_tf_2")
        
        r.sleep()

if __name__ == '__main__':
    try:
       main(sys.argv)
    except rospy.ROSInterruptException:
        pass
