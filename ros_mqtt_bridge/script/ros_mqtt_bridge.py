# MQTT
from mqtt.lib_mqtt import MQTT_OBJ
from rospy_message_converter import json_message_converter
import roslib.message
import time
import rospy
import json
import tf2_ros

class Ros_mqtt_bridge():
    def __init__(self, client_id , broker_ip , port , keepalive, clean_session):
        self.mqtt_obj = MQTT_OBJ(client_id, broker_ip, port, keepalive, clean_session)
        # container
        self.publisher = {} # {ros_topic : (mqtt_topic,data_type)}
        self.subscriber= {} # {mqtt_topic: (ros_topic, data_type, ros_pub_handle)}
        self.tf_subscriber = {} # {mqtt_topic: (frame_id, child_id)}
        # Tf
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcast = tf2_ros.TransformBroadcaster()
        # 
        self.last_timestamp = None    
    ##################
    ###  Publisher ###
    ##################
    def init_publisher(self, ros_topic, mqtt_topic, data_type):
        '''
        subscribe content in ros_topic , and relay it to MQTT topic and publish
        '''
        if ros_topic in self.publisher: 
            rospy.logwarn("[Ros_mqtt_bridge] Publisher ROS Topic name: " + ros_topic + " already exist. Reject init.")
        else: 
            self.publisher[ros_topic] = (mqtt_topic,data_type)
            rospy.Subscriber(ros_topic , roslib.message.get_message_class(data_type) , callback = self._ros_cb, callback_args = ros_topic)
            rospy.loginfo("[Ros_mqtt_bridge] Forward topic: " + ros_topic + "(ROS) --> "+ mqtt_topic + "(MQTT)")
    
    def publish_tf(self, frame_id, child_id, mqtt_topic):
        '''
        publish ros tf to mqtt topic
        '''
        try:
            #t = self.tfBuffer.lookup_transform( frame_id, child_id, rospy.Time() )
            t = self.tfBuffer.lookup_transform( frame_id, child_id, rospy.Time(0) )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e :
            pass
        else:
            if self.last_timestamp != t.header.stamp:
                self.last_timestamp = t.header.stamp
                self.mqtt_obj.publish(mqtt_topic, json_message_converter.convert_ros_message_to_json(t) , qos = 0, retain = False) # non-blocking msg

    def _ros_cb(self, data, topic_name):# This is Ros topic call back
        try:
            (mqtt_topic,data_type) = self.publisher[topic_name]
        except KeyError:
            rospy.logwarn("[Ros_mqtt_bridge] Publisher Topic name: " + topic_name + " doesn't exist. Need to init first. Ignore message.")
            return 
        # Mqtt publish 
        self.mqtt_obj.publish(mqtt_topic, json_message_converter.convert_ros_message_to_json(data) , qos = 0, retain = False) # non-blocking msg
    
    ###################
    ###  Subscriber ###
    ###################
    def init_subscriber(self,ros_topic, mqtt_topic, data_type):
        '''
        subscirbe content int mqtt_topic, and relay it to ROS topic and publish 
        '''
        if mqtt_topic in self.subscriber:
            rospy.logwarn("[Ros_mqtt_bridge] Subscribe Topic name: " + mqtt_topic + " already exist. Reject init.")
        else:
            pub = rospy.Publisher(ros_topic  , roslib.message.get_message_class(data_type)  ,queue_size = 1,  latch=False)
            self.subscriber[mqtt_topic] = (ros_topic, data_type, pub)
            self.mqtt_obj.add_subscriber([(mqtt_topic, 0, self._mqtt_cb)])
            rospy.loginfo("[Ros_mqtt_bridge] Forward topic: " + mqtt_topic + "(MQTT) --> "+ ros_topic + "(ROS)")
    
    def init_tf_subscriber(self, frame_id, child_id, mqtt_topic):
        self.mqtt_obj.add_subscriber([(mqtt_topic, 0, self._mqtt_tf_cb)])
        self.tf_subscriber[mqtt_topic] = (frame_id , child_id)
        rospy.loginfo("[Ros_mqtt_bridge] Forward topic: " + mqtt_topic + "(MQTT) --> "+ "/tf" + "(ROS)")

    def _mqtt_tf_cb(self, client, userdata, message):
        t = json_message_converter.convert_json_to_ros_message("geometry_msgs/TransformStamped", message.payload)
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.tf_subscriber[message.topic][0]
        t.child_frame_id  = self.tf_subscriber[message.topic][1]
        self.tfBroadcast.sendTransform(t)

    def _mqtt_cb(self, client, userdata, message):
        try:
            (ros_topic,data_type,pub) = self.subscriber[message.topic]
        except KeyError:
            rospy.logwarn("[Ros_mqtt_bridge] Publisher Topic name: " + message.topic + " doesn't exist. Need to init first. Ignore message.")
            return
        # Ros Publish
        pub.publish( json_message_converter.convert_json_to_ros_message(data_type, message.payload) )
        # print("[mqtt_example] topic_CB :  " + str(message.payload) + "(Q" + str(message.qos) + ", R" + str(message.retain) + ")")



