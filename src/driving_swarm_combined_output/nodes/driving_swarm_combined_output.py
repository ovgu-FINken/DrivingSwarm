#!/usr/bin/env python
import rospy
import tf_conversions
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Int8, Int32, Float64
import numpy as np
import sys
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from driving_swarm_msgs.msg import localisation_meta

def update_tf(tf_buffer, tf_broadcaster, bot_count):
        for id in range(bot_count): #0,1,2,..N-1
            try: #get tf from loc_system_locSytemName -> World -> .. -> tb3_id/base_footprint
                #print("Lookup: " +  "loc_system_" + locSystemName + " --> " + 'tb3_' + str(id) + '/base_footprint')
                #TODO: choose locSystem
                transform_msg = tf_buffer.lookup_transform("world", 'loc_system_fake_camera/target' + str(id+1), rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue #if not possible try next

            #TF: world -> target
            broadcast_tf(tf_broadcaster, "world", "tb_" + str(id+1), transform_msg)
        

def publish_metadata(has_orientation, correct_mapping, accuracy):
    localisation_meta_msg = localisation_meta()
    localisation_meta_msg.has_orientation = has_orientation
    localisation_meta_msg.correct_mapping = correct_mapping
    localisation_meta_msg.accuracy = accuracy
    topic_metadata.publish(localisation_meta_msg);

if __name__ == '__main__':

    
    loc_system_name = "combined_output"

    meta_has_orientation = True
    meta_correct_mapping = True
    meta_accuracy = 1

    rospy.init_node(loc_system_name)
    
    #get params
    bot_count = rospy.get_param('~bot_count')

    #create topic publisher
    topic_metadata = rospy.Publisher('loc_system_meta_' + loc_system_name, localisation_meta,  queue_size=1)

    #create tf buffer
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    #create tf broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10) #Hz

    #main loop:
    while not rospy.is_shutdown():
        #update tf
        update_tf(tf_buffer, tf_broadcaster, bot_count)
        #update meta topic
        publish_metadata(meta_has_orientation, meta_correct_mapping, meta_accuracy)
        rate.sleep()