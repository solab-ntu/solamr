import rospy
import roslaunch
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String

def init_roslaunch():
    global ROSLAUNCH_PARENT
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    ROSLAUNCH_PARENT = roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((PATH_FILE,)))

def pub_zero():
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    if PUB_CMD_VEL:
        PUB_CMD_VEL.publish(twist)

def cb_launch(request):
    global IS_START, IS_RUNNING

    response = SetBoolResponse()
    response.success = True

    if request.data:
        if IS_RUNNING:
            response.success = False
            response.message = "is running"
        else:
            IS_START = True
            IS_RUNNING = True
            response.message = "start"
    else:
        IS_START = False
        IS_RUNNING = False
        ROSLAUNCH_PARENT.shutdown()
        rospy.loginfo("switch stop : {}".format(PATH_FILE))
        pub_zero()
        init_roslaunch()
        response.message = "shutdown"
        if PUB_STATUS:
            PUB_STATUS.publish("")

    return response

if __name__ == "__main__":

    rospy.init_node(name='switch', anonymous=False)

    # -- Get parameters
    PATH_FILE = rospy.get_param(param_name="~path_file")
    is_shutdown_zero_vel = rospy.get_param(param_name="~is_shutdown_zero_vel")
    is_pub_status = rospy.get_param(param_name="~is_pub_status")

    # -- Node function
    rospy.Service(name="~turn_on", service_class=SetBool, handler=cb_launch)
    PUB_CMD_VEL = None
    PUB_STATUS = None

    if is_shutdown_zero_vel:
        PUB_CMD_VEL = rospy.Publisher(name="cmd_vel", data_class=Twist, queue_size=1)

    if is_pub_status:
        PUB_STATUS = rospy.Publisher(name="/status_for_web", data_class=String, queue_size=1)

    # -- Initialize ROSLAUNCH_PARENT
    ROSLAUNCH_PARENT = None
    init_roslaunch()

    # -- Flag for running ROSLaunchParent in main process
    IS_START = False
    IS_RUNNING = False

    # -- Loop
    rate = rospy.Rate(1.0)  # 1 Hz
    while not rospy.is_shutdown():
        if IS_START:
            rospy.loginfo("switch start : {}".format(PATH_FILE))
            ROSLAUNCH_PARENT.start()
            IS_START = False
            if PUB_STATUS:
                PUB_STATUS.publish("Executing")
        rate.sleep()