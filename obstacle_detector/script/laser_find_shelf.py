#!/usr/bin/env python
import rospy
# ROS msg and libraries
from nav_msgs.msg import OccupancyGrid, Path # Global map 
from geometry_msgs.msg import TransformStamped, Point, PoseArray, PoseStamped, Pose2D, Pose,PoseWithCovarianceStamped, Quaternion# Global path
from visualization_msgs.msg import Marker, MarkerArray # Debug drawing
from std_msgs.msg import String, Float64
import tf
import tf2_ros
from obstacle_detector.msg import Obstacles
from math import atan2,acos,sqrt,pi,sin,cos
# Lucky utility
from lucky_utility.ros.rospy_utility import Marker_Manager, get_tf, send_tf,\
                                            normalize_angle,cal_ang_distance,\
                                            cal_avg_angle, vec_trans_coordinate

class Corner():
    def __init__(self, corner, neighbor1, neighbor2):
        self.corner = corner#  (x,y)
        self.neighbor1 = neighbor1 # (x,y)
        self.neighbor2 = neighbor2 # (x,y)
        self.center = self.cal_center(corner, neighbor1, neighbor2)

    def cal_center(self,c1,c2,c3):
        '''
        Calculate center of shelf by right triangle (c1,c2,c3)
        Input: 
            c1 : (x,y) - this is a right angle coner
            c2 : 
            c3 : - Two vertice adjcency to coner
        Output:
            center: (x,y)
        '''
        a = (c2[0] - c1[0] , c2[1] - c1[1])
        b = (c3[0] - c1[0] , c3[1] - c1[1])
        return ( ( c1[0] + a[0]/2.0 + b[0]/2.0 , c1[1] + a[1]/2.0 + b[1]/2.0 ) )
    

class Shelf_finder():
    def __init__(self,
                 sheft_length_tolerance,
                 angle_tolerance,
                 max_circle_radius,
                 search_radius,
                 name):
        self.scan = None
        self.center = (None, None, None) # (x,y,theta) - centers of shelf
        self.search_center = None# (x,y)
        self.corner_dict = {}
        #------ Parameters --------#
        self.sheft_length_tolerance = sheft_length_tolerance # Meter, Tolerance of detecing shelf's length
        self.angle_tolerance = angle_tolerance # Radian, Tolerance of detecing right angle
        self.max_circle_radius = max_circle_radius # Meter, Ignore clusters's radius bigger than MAX_CIRCLE_RADIUS
        self.search_radius = search_radius
        self.name = name # "base" or "peer"
        #----- Init node ------#
        # Name of this node, anonymous means name will be auto generated.
        rospy.init_node('laser_find_shelf', anonymous=False)
        #----- Subscriber -------# 
        if ROLE == "leader":
            self.viz_marker = Marker_Manager("obstacle_detector/markers/" + name)
            self.viz_marker.register_marker("corners_"+name, 7, "car1/base_link", (0,0,255) , 0.1)
            self.viz_marker.register_marker("edges_"+name, 5  , "car1/base_link", (255,255,0), 0.02)
        else:
            self.viz_marker = Marker_Manager("obstacle_detector/markers/" + name)
            self.viz_marker.register_marker("corners_"+name, 7, ROBOT_NAME+"/map", (0,0,255) , 0.1)
            self.viz_marker.register_marker("edges_"+name, 5  , ROBOT_NAME+"/map", (255,255,0), 0.02)

    def set_mode(self,sheft_length_tolerance, angle_tolerance, max_circle_radius, search_radius):
        '''
        '''
        self.sheft_length_tolerance = sheft_length_tolerance 
        self.angle_tolerance = angle_tolerance 
        self.max_circle_radius = max_circle_radius 
        self.search_radius = search_radius

    def cal_distance(self,c1,c2):
        '''
        Calculate Eucleides distance between c1 and c2 
        Input:
            c1: (x1,y1)
            c2: (x2,y2)
        Return: 
            float - Eucleides distance
        '''
        dx = c1[0] - c2[0]
        dy = c1[1] - c2[1]
        return sqrt(dx**2 + dy**2)

    def cal_angle(self,c1,c2,c3):
        '''
        Calculate angle between c1c2 and c1c3
        Input:
            c1: (x1,y1)
            c2: (x2,y2)
            c3: (x3,y3)
        Output: 
            angle - Radian
        '''
        a = (c2[0] - c1[0] , c2[1] - c1[1])
        b = (c3[0] - c1[0] , c3[1] - c1[1])
        return acos( round( (a[0]*b[0] + a[1]*b[1]) / (self.cal_distance(c1,c2) * self.cal_distance(c1,c3) ) ,5) )

    def check_search_region(self, search_center, search_radius, test_point):
        '''
        Check test point is in search region or not
        Arguments:
            search_center - (x,y), center of search region
            search_radius - float, radius of search region 
            test_point - (x,y), point that need to be checked.
        Return: 
            True - Inside search region
            False - Outside search region 
        Note that if search_center or search_radiusa are None, then return Ture without checking(disable checking).
        '''
        if search_center == None or search_radius == None: # skip checking 
            return True 
        else: # need to check
            if self.cal_distance(test_point, search_center) > search_radius: #Outside search region
                return False
            else:
                return True

    def cal_corner(self, raw_data, search_center):
        '''
        Arguement:
            raw_data - obstacle_detector/Obstacles
            search_center - (x,y)
            ref_heading - heading calculate last time
        Return:
            corner_dict: {(x1,y1): Corner(), (x2,y2): Corner()}
        '''
        # Ignore too large circle, and those outside the search region
        c_list = []
        for c in raw_data.circles:
            c_point = (c.center.x, c.center.y)
            if c.true_radius <= self.max_circle_radius and\
               self.check_search_region(search_center, self.search_radius, c_point):
                c_list.append(c_point)

        # Get adj_dict, indicate adjecency relationship between pionts.
        adj_dict = {} # {c1:[ (L1,c2) , (L2,c3) ], c2: .....}
        for c1 in c_list:
            for c2 in c_list:
                l = self.cal_distance(c1,c2)
                if  abs(l - SHELF_LEN)          < self.sheft_length_tolerance or\
                    abs(l - SHELF_LEN_DIAGONAL) < self.sheft_length_tolerance :
                    if c1 not in adj_dict: # New circle
                        adj_dict[c1] = []
                    adj_dict[c1].append( (l , c2) )
        
        # Get conner in adj_list
        corner_dict = {}
        for c in adj_dict:
            if len(adj_dict[c]) <= 1: # Too few edge, definitely not a corner
                continue 
            # Find two edge that has 90 degree and both with lenth L 
            for e1 in adj_dict[c]:
                for e2 in adj_dict[c]:
                    if  abs(e1[0] - SHELF_LEN) < self.sheft_length_tolerance and\
                        abs(e2[0] - SHELF_LEN) < self.sheft_length_tolerance: # Both edge has length L
                        angle = self.cal_angle(c , e1[1] ,e2[1] )
                        if abs(angle - pi/2) < self.angle_tolerance and c not in corner_dict:# Angle is 90 degree
                            # Found coner!!!
                            corner_dict[c] = Corner(c, e1[1] ,e2[1])
        return corner_dict

    def cal_heading(self,c1,c2,c3,ref):
        '''
        Calculate heading of shelft by a right triangle(c1,c2,c3)
        Because shelf is a sqare, there're four possible directions (NSWE)
        we choose one direction that is cloest to ref angle.
        Arguments:
            c1 : (x,y) - this is a coner
            c2 : (x,y) - Two vertice adjcency to coner
            c3 : (x,y)
            ref : referance angle, output angle should be as near as possible to ref
        Return:
            float - heading
        Note that if ref is None, it will pick a direction arbitrarily
        '''
        a = (c2[0] - c1[0] , c2[1] - c1[1])
        b = (c3[0] - c1[0] , c3[1] - c1[1])
        if ref != None: # two sides of the right triangle, give us two values of heading
            ang1 = self.find_nearest_angle_to_ref ( atan2(a[1], a[0]) , ref)
            ang2 = self.find_nearest_angle_to_ref ( atan2(b[1], b[0]) , ang1)
        else:
            ang1 = atan2(a[1], a[0])
            ang2 = self.find_nearest_angle_to_ref ( atan2(b[1], b[0]) , ang1)
        return cal_avg_angle((ang1, ang2))
    
    def find_nearest_angle_to_ref(self, angle ,ref):
        '''
        Find nearest angle to ref
        All four possible angle : 
        [angle , angle+pi/2 + angle +pi + angle + 3*pi/2]
        '''
        possible_angles = [angle, angle+(pi/2) , angle+pi, angle+(3*pi/2) ]
        
        ans = None
        min_dtheta = float('inf')
        for i in possible_angles:
            ang_dis = cal_ang_distance(i, ref)
            if ang_dis < min_dtheta:
                min_dtheta = ang_dis
                ans = i
            if 2*pi - ang_dis < min_dtheta:
                min_dtheta = 2*pi - ang_dis
                ans = i
        return ans

    def cal_avg_center(self, ref_ang):
        '''
        Calculate avgerage center from self.corner_dict
        Return :
            (x,y,theta) 
        '''
        avg_center = [0,0,0] # (x,y,yaw)
        
        # Get x,y
        for c in self.corner_dict:
            avg_center[0] += self.corner_dict[c].center[0]
            avg_center[1] += self.corner_dict[c].center[1]
        avg_center[0] /= len(self.corner_dict)
        avg_center[1] /= len(self.corner_dict)
        
        # Get theta
        heading_list = []
        for c in self.corner_dict:
            heading_list.append(self.cal_heading(c, self.corner_dict[c].neighbor1,
                                self.corner_dict[c].neighbor2, ref_ang))
        avg_center[2] = cal_avg_angle(heading_list)
        return avg_center

    def publish_viz(self):
        self.viz_marker.publish()
    
    def run_once(self, ref_ang):
        '''
        return True: Allow publish
        '''
        self.corner_dict = self.cal_corner(self.scan, self.search_center)
        if self.corner_dict == {}:
            return False # Can't find any corner

        # Get average center(x,y)
        self.center = self.cal_avg_center(ref_ang)

        # Update debug markers
        self.viz_marker.update_marker("corners_"+self.name, tuple(self.corner_dict.keys()))
        line_list = []
        for corner in self.corner_dict:
            line_list.extend([corner, self.corner_dict[corner].neighbor1, 
                              corner, self.corner_dict[corner].neighbor2])
        self.viz_marker.update_marker("edges_"+self.name, line_list)

        return True
    
class Two_shelf_finder():
    def __init__(self):
        self.big_car_xyt = [None, None, None]
        # 
        self.scan = None
        # Init laser_finder
        self.shelf_finder_base = Shelf_finder(sheft_length_tolerance = SHEFT_LENGTH_TOLERANCE,
                                                angle_tolerance = ANGLE_TOLERANCE,
                                                max_circle_radius = MAX_CIRCLE_RADIUS,
                                                search_radius = SEARCH_RADIUS,
                                                name="base")
        self.shelf_finder_peer = Shelf_finder(sheft_length_tolerance = SHEFT_LENGTH_TOLERANCE,
                                                angle_tolerance = ANGLE_TOLERANCE,
                                                max_circle_radius = MAX_CIRCLE_RADIUS,
                                                search_radius = SEARCH_RADIUS,
                                                name="peer") # After combine # TODO Use topic or service
        
        self.direction_list = [(TOW_CAR_LENGTH, 0),
                               (-TOW_CAR_LENGTH, 0),
                               (0, TOW_CAR_LENGTH),
                               (0, -TOW_CAR_LENGTH)]
        #----- TF -----------#
        # For getting Tf map -> base_link
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.base_link_xy = None
        # Subscrieer 
        rospy.Subscriber("raw_obstacles", Obstacles, self.obstacle_cb)
        # Publisher
        self.pub_theta = rospy.Publisher("/"+ ROBOT_NAME +"/theta", Float64, queue_size = 1,latch=False)
        self.theta = 0.0
    
    def obstacle_cb(self,data):
        '''
        This callback will be called when /raw_obstacle is published
        Get self.center - center of shelft neer base_link
        and self.center_peer - center of shelft neer base_link_peer
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        obstacle_detector/SegmentObstacle[] segments
            geometry_msgs/Point first_point
                    float64 x
                    float64 y
                    float64 z
            geometry_msgs/Point last_point
                    float64 x
                    float64 y
                    float64 z
        obstacle_detector/CircleObstacle[] circles
            geometry_msgs/Point center
                float64 x
                float64 y
                float64 z
            geometry_msgs/Vector3 velocity
                float64 x
                float64 y
                float64 z
            float64 radius
            float64 true_radius
        '''
        self.scan = data
    
    def run_once(self):
        # Update scan
        if self.scan == None: # No scan data
            return False
        else:
            self.shelf_finder_base.scan = self.scan
            self.shelf_finder_peer.scan = self.scan
        
        # Calculate base center
        self.shelf_finder_base.search_center = (0,0)
        if self.shelf_finder_base.run_once(self.big_car_xyt[2]):
            # Publish base shelf
            self.shelf_finder_base.publish_viz()

            # Calculate peer center
            marker_tmp_list = []
            for p_tem in self.direction_list:
                theta = self.shelf_finder_base.center[2]
                (x, y) = vec_trans_coordinate(p_tem, (0, 0, theta))
                self.shelf_finder_peer.search_center = (x,y)
                marker_tmp_list.append((x,y))
                
                if self.shelf_finder_peer.run_once(self.big_car_xyt[2]):# Found the center of peer
                    # Cache the direction
                    self.direction_list.remove(p_tem)
                    self.direction_list.insert(0,p_tem)
                    break

            #Publish peer shelf
            if self.shelf_finder_peer.corner_dict == {}:
                rospy.logdebug("[laser_finder] Can't find peer shelft center.")
            else:
                # Publish peer
                self.shelf_finder_peer.publish_viz()
                # Calculate big car
                vec_big_car = (self.shelf_finder_base.center[0] - self.shelf_finder_peer.center[0], 
                               self.shelf_finder_base.center[1] - self.shelf_finder_peer.center[1])
                self.big_car_xyt = ((self.shelf_finder_base.center[0] + self.shelf_finder_peer.center[0])/2.0,
                                    (self.shelf_finder_base.center[1] + self.shelf_finder_peer.center[1])/2.0,
                                    atan2(vec_big_car[1], vec_big_car[0]))
            return True #We need base.center only, big_car_xyt is for ref_ang
        return False
    
    def publish_theta(self):
        # Get theta1 or theta2
        if ROLE == "leader": # Theta1
            self.theta = normalize_angle(-self.shelf_finder_base.center[2]) 
        elif ROLE == "follower": # Theta2
            self.theta = normalize_angle(-self.shelf_finder_base.center[2] + pi)
        self.pub_theta.publish(self.theta)

        # Send tf
        if ROLE == "leader":
           send_tf((TOW_CAR_LENGTH/2.0, 0, self.theta), "carB/base_link", "car1/base_link")

if __name__ == '__main__':
    rospy.init_node('laser_find_shelf',anonymous=False)
    # Get launch file parameters
    # System
    ROBOT_NAME    = rospy.get_param(param_name="~robot_name", default="car1")
    ROLE          = rospy.get_param(param_name="~role", default="leader")
    FREQUENCY     = rospy.get_param(param_name="~frequency", default="10")
    # Kinematics
    TOW_CAR_LENGTH =rospy.get_param(param_name="~two_car_length", default="0.93")
    SHELF_LEN      =rospy.get_param(param_name="~shelf_length", default="0.73")
    MAX_CIRCLE_RADIUS = rospy.get_param(param_name="~max_circle_radius", default="0.20")
    SEARCH_RADIUS = rospy.get_param(param_name="~search_radius", default="0.55")
    # Tolerance
    SHEFT_LENGTH_TOLERANCE = rospy.get_param(param_name="~sheft_length_tolerance", default="0.1")
    ANGLE_TOLERANCE =  rospy.get_param(param_name="~angle_tolerance", default="5")*pi/180 # Degree

    SHELF_LEN_DIAGONAL = SHELF_LEN * sqrt(2)
    TWO_SHELF_FINDER = Two_shelf_finder()
    INIT = False
    rate = rospy.Rate(FREQUENCY)
    while not rospy.is_shutdown():
        if TWO_SHELF_FINDER.run_once():
            TWO_SHELF_FINDER.publish_theta()
            INIT = True
        elif ROLE == "leader":
            if INIT == False : # TODO test dont' init
                send_tf((TOW_CAR_LENGTH/2.0, 0, 0), "carB/base_link", "car1/base_link")
                rospy.logerr("[laser_finder] NOt init yet")
            else:
                send_tf((TOW_CAR_LENGTH/2.0, 0, TWO_SHELF_FINDER.theta), "carB/base_link", "car1/base_link")
        rate.sleep()
