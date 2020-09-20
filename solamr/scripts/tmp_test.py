#!/usr/bin/env python  
import rospy
import rospkg
import yaml
import tf_conversions
import tf2_ros
import tf
import tf.msg
from lucky_utility.ros.rospy_utility import get_tf, send_tf, vec_trans_coordinate, normalize_angle
from geometry_msgs.msg import Twist, Point, PoseStamped, Polygon, Point32, PoseWithCovarianceStamped
from solamr.srv import StringSrv
import threading
import time 


def test_thread():
    while IS_STOPPING_ROBOT:
        print ("inside test thread")
        pub_cmd_vel.publish(Twist())
        time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('tmp_test')
    
    # Publisher
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    IS_STOPPING_ROBOT = True
    # Start checking IS_RUN
    t_check_running = threading.Thread(target=test_thread)
    t_check_running.start()
    
    rate = rospy.Rate(10.0)
    time_start = time.time()
    while not rospy.is_shutdown():
        if time.time() - time_start > 5: # sec
            IS_STOPPING_ROBOT = False
            t_check_running.join()
            print("join thread")
        rate.sleep()

