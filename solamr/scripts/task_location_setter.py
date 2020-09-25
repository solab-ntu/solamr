#!/usr/bin/env python  
import rospkg
import yaml
import rospy

# ROS msg and libraries
from tf import transformations
import tf
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing 
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

###########################################
###  Interactive Marker Modification    ###
###########################################
def make6DofMarker( fixed, interaction_mode, pose, show_6dof = False, name="marker"):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose = pose
    int_marker.scale = 0.3

    int_marker.name = ""
    int_marker.description = ""

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof: 
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    int_marker.name = name 
    int_marker.description = name 
    
    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    server.insert(int_marker, marker_feedback_cb)
    # menu_handler.apply( server, int_marker.name )

def makeYaxisMarker(position, name ):
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.pose.position = position
    int_marker.scale = 1

    q = tf.transformations.quaternion_from_euler(0, 0, pi/2)
    int_marker.pose.orientation.x = q[0]
    int_marker.pose.orientation.y = q[1]
    int_marker.pose.orientation.z = q[2]
    int_marker.pose.orientation.w = q[3]

    int_marker.pose.orientation.x
    int_marker.header.frame_id = "map"
    int_marker.name = name
    int_marker.description = name

    # create a grey box marker
    box_marker = makeBox(int_marker)

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_y"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    # add the control to the interactive marker
    int_marker.controls.append(rotate_control);

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, marker_feedback_cb)

def makeBox( msg ):
    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def marker_feedback_cb(data):
    '''
    '''
    print (data)

if __name__ == '__main__':
    rospy.init_node('task_location_setter')
    try:
        path = rospkg.RosPack().get_path('solamr') + "/params/task/task_location.yaml"
        with open(path) as file:
            rospy.loginfo("[tf_rviz_forwarder] Load yaml file from " + path)
            LOCATIONS = yaml.safe_load(file)
    except OSError:
        rospy.logerr("[tf_rviz_forwarder] Yaml file not found at " + path)
    print (LOCATIONS)

    #------- Interactive Markers ---------# 
    server = InteractiveMarkerServer("basic_controls")


    #------- Markers: Two cars -----------# 
    # print ("car_1 theta : " + str(car_1.theta))
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose = Pose(Point(0,0,0), Quaternion(q[0],q[1],q[2],q[3])) 
    make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True , "test_1")
    # q = tf.transformations.quaternion_from_euler(0, 0, car_2.theta)
    # pose = Pose(Point(car_2.x,car_2.y,0), Quaternion(q[0],q[1],q[2],q[3]))# 45 degree
    # make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, pose, True , "car_2")



    #---- Init Markers ----# 
    init = InteractiveMarkerFeedback()
    init.marker_name = ""
    init.header.frame_id = "map"
    marker_feedback_cb(init)
    server.applyChanges()
    r = rospy.Rate(30) #call at 30HZ
    while (not rospy.is_shutdown()):
        r.sleep()