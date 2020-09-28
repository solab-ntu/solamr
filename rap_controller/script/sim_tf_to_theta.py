#! /usr/bin/env python
import rospy
import tf2_ros
from std_msgs.msg import Float64

from rospy_tool.rospy_tool_lib import get_tf

if __name__ == '__main__':
    rospy.init_node("tf2theta", anonymous=True)
    PUB_CAR1_THETA = rospy.Publisher("/car1/theta", Float64,queue_size = 1,latch=False)
    PUB_CAR2_THETA = rospy.Publisher("/car2/theta", Float64,queue_size = 1,latch=False)

    # Tf listner
    TFBUFFER = tf2_ros.Buffer()
    tf2_ros.TransformListener(TFBUFFER)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t_car1 = get_tf(TFBUFFER, "base_link", "car1")
        t_car2 = get_tf(TFBUFFER, "base_link", "car2")
        if t_car1 != None:
            PUB_CAR1_THETA.publish(t_car1[2])
        if t_car2 != None:
            PUB_CAR2_THETA.publish(t_car2[2])
        rate.sleep()
