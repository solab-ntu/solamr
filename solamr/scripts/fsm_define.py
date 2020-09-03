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

shelf_id: 87
tag_location: [1,2,3]
wait_location: [4,5,6]
goal_location: [7,8,9]
home_location: [9,9,9]
class Task(object):
    def __init__(self, mode, shelf_id, tag_location,
                 wait_location = None, goal_location = None, home_location = None):
        self.mode = mode
        self.shelf_id = shelf_id
        self.tag_location = tag_location
        self.wait_location = wait_location
        self.goal_location = goal_location
        self.home_location = home_location

#######################
### Global Function ###
#######################

def init_roslaunch(file_path):
    '''
    Let it able to switch roslaunch 
    Argument:
        file_path: string - the roslaunch file path you want to launch
    Return:
        parent of roslaunch process
    '''
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    return roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((file_path,)))

class Single_AMR(smach.State):

    def __init__(self, file_path):
        super(Single_AMR, self).__init__(outcomes=('Find_Shelf', 'done'), output_keys=["task"])
        self.launch_file = file_path

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Single_AMR')
        # TODO Dark Magic, Change footprint back to single AMR
        os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.22,-0.22,0.0], [-0.22,0.22,0.0], [0.22,0.22,0.0], [0.22,-0.22,0.0]]'")
        
        roslaunch = init_roslaunch(self.launch_file)
        roslaunch.start()
        userdata.task = TASK
        time.sleep(5)
        roslaunch.shutdown()
        return 'Find_Shelf'

class Find_Shelf(smach.State):

    def __init__(self):
        super(Find_Shelf, self).__init__(outcomes=('abort', 'done'), input_keys=["task"])

    def execute(self, userdata):
        '''
        userdata.tagID -> find shelf
        userdata.proximity_goal -> A fuszy goal that shelf prababaly nearby
        '''
        # TODO Listen to service abort
        rospy.loginfo('[fsm] Execute Find_Shelf')
        rospy.loginfo(str(userdata.task.mode))
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
        super(Single_Assembled, self).__init__(outcomes=['Dock_Out', 'Go_Way_Point'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Single_Assembled')
        
        # TODO Dark Magic, Change footprint to single Assembled
        os.system("rostopic pub --once /move_base/local_costmap/footprint geometry_msgs/Polygon -- '[[-0.37,-0.37,0.0], [-0.37,0.37,0.0], [0.37,0.37,0.0], [0.37,-0.37,0.0]]'")
        time.sleep(5)
        return 'Go_Way_Point'
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

class Go_Way_Point(smach.State):
    def __init__(self):
        super(Go_Way_Point, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Go_Way_Point')
        time.sleep(1)
        return 'done'

class Go_Goal(smach.State):
    def __init__(self):
        super(Go_Goal, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Go_Goal')
        time.sleep(1)
        return 'done'

class Go_Home(smach.State):
    def __init__(self):
        super(Go_Home, self).__init__(outcomes=['done', 'abort'], input_keys=["target"], output_keys=["behavior"])

    def execute(self, userdata):
        rospy.loginfo('[fsm] Execute Go_Home')
        time.sleep(1)
        return 'done'

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
