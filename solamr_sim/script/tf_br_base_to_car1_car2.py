#! /usr/bin/env python

"""
tf broadcaster for base_link (double shelf) to car1 and car2
"""

import math
import threading

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import tf
import tf2_ros


class TfListener:
    """
    Use threading to get listener tf odom -> base_link, and save them to global vars.
    Attributes:
        _tf_buffer (tf2_ros.Buffer):
    """
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buffer)  # for buffer

    def _job(self):
        '''
        Get tf odom to base_link
        '''
        rate = rospy.Rate(hz=10.0)
        while not rospy.is_shutdown():
            try:
                t = self._tf_buffer.lookup_transform(
                    target_frame="odom",
                    source_frame=FRAME_BASE_LINK,
                    time=rospy.Time())
            except Exception as err:
                rospy.loginfo(err)
            else:
                global BIG_CAR_P, BIG_CAR_Q
                BIG_CAR_P[0] = t.transform.translation.x
                BIG_CAR_P[1] = t.transform.translation.y
                BIG_CAR_Q[0] = t.transform.rotation.x
                BIG_CAR_Q[1] = t.transform.rotation.y
                BIG_CAR_Q[2] = t.transform.rotation.z
                BIG_CAR_Q[3] = t.transform.rotation.w
            rate.sleep()

    def start_thread(self):
        '''
        Start threading
        '''
        thread = threading.Thread(target=self._job, name='job')
        thread.start()


def _cb_car1_imu(msg):
    """
    Callback for topic car1 imu
    """
    global CAR1_Q
    CAR1_Q[0] = msg.orientation.x
    CAR1_Q[1] = msg.orientation.y
    CAR1_Q[2] = msg.orientation.z
    CAR1_Q[3] = msg.orientation.w

def _cb_car2_imu(msg):
    """
    Callback for topic car2 imu
    """
    global CAR2_Q
    CAR2_Q[0] = msg.orientation.x
    CAR2_Q[1] = msg.orientation.y
    CAR2_Q[2] = msg.orientation.z
    CAR2_Q[3] = msg.orientation.w

def get_car1_tf():
    """
    Get tf base_link -> car1
    Which is combined by base_link and imu information
    """
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = FRAME_BASE_LINK
    t.child_frame_id = FRAME_CAR1
    t.transform.translation.x = 0.465
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    _q = tf.transformations.quaternion_multiply(CAR1_Q, tf.transformations.quaternion_inverse(BIG_CAR_Q))
    t.transform.rotation.x = _q[0]
    t.transform.rotation.y = _q[1]
    t.transform.rotation.z = _q[2]
    t.transform.rotation.w = _q[3]
    return t

def get_car2_tf():
    """
    Get tf base_link -> car2
    Which is combined by base_link and imu information
    """
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = FRAME_BASE_LINK
    t.child_frame_id =FRAME_CAR2
    t.transform.translation.x = -0.465
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    _q = tf.transformations.quaternion_multiply(CAR2_Q, tf.transformations.quaternion_inverse(BIG_CAR_Q))
    t.transform.rotation.x = _q[0]
    t.transform.rotation.y = _q[1]
    t.transform.rotation.z = _q[2]
    t.transform.rotation.w = _q[3]
    return t

if __name__ == '__main__':

    # -- global vars
    BIG_CAR_P = [0.0, 0.0]  # base_link pose (tf odom to base_link)
    BIG_CAR_Q = [0.0, 0.0, 0.0, 1.0]  # base_link orientation (tf odom to base_link)
    CAR1_Q = [0.0, 0.0, 0.0, 1.0]  # car1 imu orientation (initial zero)
    CAR2_Q = [0.0, 0.0, 0.0, 1.0]  # car2 imu orientation (initial zero)

    # -- ros node function
    ## -- parameters
    rospy.init_node('tf_br_base_to_car1_car2')

    FRAME_BASE_LINK = rospy.get_param(param_name="~frame_base_link")
    FRAME_CAR1 = rospy.get_param(param_name="~frame_car1")
    FRAME_CAR2 = rospy.get_param(param_name="~frame_car2")
    top_car1_imu = rospy.get_param(param_name="~car1_imu")
    top_car2_imu = rospy.get_param(param_name="~car2_imu")
    pub_rate = rospy.get_param(param_name="~publish_rate")

    ## -- subscriber
    rospy.Subscriber(name=top_car1_imu, data_class=Imu, callback=_cb_car1_imu)
    rospy.Subscriber(name=top_car2_imu, data_class=Imu, callback=_cb_car2_imu)

    # -- tf listenser from big_car
    tf_listener = TfListener()
    tf_listener.start_thread()

    # -- tf broadcaster for car1, car2
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(hz=int(pub_rate))
    try:
        while not rospy.is_shutdown():
            car1_tf = get_car1_tf()
            car2_tf = get_car2_tf()
            br.sendTransform(transform=car1_tf)
            br.sendTransform(transform=car2_tf)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
