#!/usr/bin/env python
# Python 
import time
from math import radians
# ROS
import rospy
import roslaunch
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
# SMACH
import smach


def init_roslaunch(file_path):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    return roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))


class Single_AMR(smach.State):

    def __init__(self, file_path):
        super(Single_AMR, self).__init__(outcomes=('Single_Assembled', 'done'), output_keys=["target"])
        self.file_path = file_path

    def execute(self, userdata):
        rospy.loginfo('fsm/Single_AMR')
        
        roslaunch = init_roslaunch(self.file_path)
        roslaunch.start()
        time.sleep(5)
        roslaunch.shutdown()
        return 'Single_Assembled'


class Single_Assembled(smach.State):

    def __init__(self, file_path):
        super(Single_Assembled, self).__init__(outcomes=['Single_AMR', 'done'], input_keys=["target"], output_keys=["behavior"])
        # self.action_move_base = action_move_base
        # self.status = None

    def execute(self, userdata):
        rospy.loginfo('fsm/Single_Assembled')
        # userdata.target = None
        time.sleep(2)
        return 'Single_AMR'
        '''
        x_m, y_m, rz_deg, behavior = userdata.target
        userdata.behavior = behavior

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_m
        goal.target_pose.pose.position.y = y_m
        q = quaternion_from_euler(0.0, 0.0, radians(rz_deg))
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo('fsm/Single_Assembled : {} {} {}'.format(x_m, y_m, rz_deg))
        self.status = None
        self.action_move_base.send_goal(goal=goal, feedback_cb=None, done_cb=self.done_cb)
        self.action_move_base.wait_for_result()

        if self.status == 3:
            return 'done'
        else:
            return 'failed'
        '''

    def done_cb(self, status, result):
        # -- http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        self.status = status


# class BehaviorState(smach.State):

#     def __init__(self, pub_cmd_vel):
#         super(BehaviorState, self).__init__(outcomes=['done'], input_keys=["behavior"])
#         self.pub_cmd_vel = pub_cmd_vel

#     def execute(self, userdata):
#         if userdata.behavior == 1:
#             rospy.loginfo("fsm/BehaviorState : rotating")
#             self.rotate(wz=1.0, sec=2.0)
#             self.rotate(wz=-1.0, sec=4.0)
#             self.rotate(wz=1.0, sec=2.0)
#         else:
#             rospy.loginfo("fsm/BehaviorState : just a waypoint")
#             time.sleep(1)
#         return 'done'

#     def rotate(self, wz, sec):
#         rate = rospy.Rate(hz=5)
#         t_start = rospy.get_rostime().to_sec()
#         while True:
#             twist = Twist()
#             t_now = rospy.get_rostime().to_sec()
#             if t_now - t_start < sec:
#                 twist.angular.z = wz
#                 self.pub_cmd_vel.publish(twist)
#             else:
#                 self.pub_cmd_vel.publish(twist)  # zero cmd
#                 break
#             rate.sleep()


def parse_file(path_file):

    with open(name=path_file, mode="r") as fileIO:
        _lines = fileIO.readlines()

    waypoints = list()
    for line in _lines:
        if line and not line.isspace():
            target = tuple(float(i) for i in line.split(","))  # (x_m, y_m, rz_deg, behavior)
            waypoints.append(target)

    return waypoints

if __name__ == "__main__":

    path_file = "../file/waypoints.csv"
    waypoints = parse_file(path_file=path_file)

    for p in waypoints:
        print(p)
