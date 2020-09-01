#!/usr/bin/env python
# Python 
import time
import os 
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
        super(Single_AMR, self).__init__(outcomes=('Find_Shelf', 'done'), output_keys=["target"])
        self.launch_file = file_path

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Single_AMR')
        # TODO Dark Magic, Change footprint back to single AMR
        os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.22,-0.22,0.0], [-0.22,0.22,0.0], [0.22,0.22,0.0], [0.22,-0.22,0.0]]'")
        
        roslaunch = init_roslaunch(self.launch_file)
        roslaunch.start()
        time.sleep(5)
        roslaunch.shutdown()
        return 'Find_Shelf'


class Find_Shelf(smach.State):

    def __init__(self):
        super(Find_Shelf, self).__init__(outcomes=('abort', 'done'), input_keys=["tagID", "proximity_goal"])

    def execute(self, userdata):
        '''
        userdata.tagID -> find shelf
        userdata.proximity_goal -> A fuszy goal that shelf prababaly nearby
        '''
        # TODO Listen to service abort
        rospy.loginfo('[fsm] Execute Find_Shelf')
        # TODO send_goal (userdata.proximity_goal)
        # Keep checking tagid, and make sure standby tf is published
        time.sleep(1)
        return 'done'

class Go_Dock_Standby(smach.State):

    def __init__(self):
        super(Go_Dock_Standby, self).__init__(outcomes=('abort','done'), output_keys=[])

    def execute(self, userdata):
        '''
        '''
        rospy.loginfo('[fsm] Execute Go_Dock_Standby')
        time.sleep(1)
        # TODO Go to standby point , if navi goal is 
        # Send_goal()
        return 'done'

class Dock_In(smach.State):

    def __init__(self):
        super(Dock_In, self).__init__(outcomes=('Single_Assembled', 'Double_Assembled', 'abort'), output_keys=["target"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Dock_In')
        # TODO # Switch launch file, You have to know you're car1/car2 at this point
        # TODO How to decide goto Single_Assembled or Double_Assembled
        # If IR said it's done or navigation goal is reached
        time.sleep(1)
        return 'Single_Assembled'

class Dock_Out(smach.State):

    def __init__(self):
        super(Dock_Out, self).__init__(outcomes=('abort', 'done'), output_keys=["target"])

    def execute(self, userdata):
        '''
        '''
        rospy.loginfo('[fsm] Execute Dock_Out')
        
        # Switch launch file 
        # TODO How to decide goto Single_Assembled or Double_Assembled
        # TODO reached dock_out navi goal
        time.sleep(1)
        return 'done'

class Single_Assembled(smach.State):

    def __init__(self):
        super(Single_Assembled, self).__init__(outcomes=['Dock_Out'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Single_Assembled')
        
        # TODO Dark Magic, Change footprint to single Assembled
        os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.37,-0.37,0.0], [-0.37,0.37,0.0], [0.37,0.37,0.0], [0.37,-0.37,0.0]]'")
        time.sleep(5)
        return 'Dock_Out'
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


class Double_Assembled(smach.State):

    def __init__(self, file_path):
        super(Double_Assembled, self).__init__(outcomes=['Dock_Out'], output_keys=["target"])
        self.launch_file = file_path

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Double_Assembled')

        roslaunch = init_roslaunch(self.launch_file)
        roslaunch.start()
        time.sleep(5)
        # roslaunch.shutdown()
        return 'Dock_Out'


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
