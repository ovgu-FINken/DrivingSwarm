#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Int8
import numpy as np

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

locSystemName = "fakelocalisation"

def create_static_tf(parent, child, transform): #((x,y,z), (q1,q2,q3,q4))):
   # print("Parent: " + parent + " Child: " + child)
    if(type(transform) is not tuple): 
        br = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform = transform
        br.sendTransform(t)

def create_transform_msg((x,y,z),(qx,qy,qz,qw)):
    t = Transform(Vector3(x,y,z), Quaternion(qx,qy,qz,qw))
    return t

def sample_normal_distribution(mu, sigma):
    s = float(np.random.normal(mu, sigma, 1))
    return s

def add_error_to_transformation(transformation):
    transformation.translation.x *=  sample_normal_distribution(1, target_random_multiplicativ_error_sigma)
    transformation.translation.x += sample_normal_distribution(0, target_random_additiv_error_sigma)
    transformation.translation.y *= sample_normal_distribution(1, target_random_multiplicativ_error_sigma)
    transformation.translation.y += sample_normal_distribution(0, target_random_additiv_error_sigma)
    transformation.translation.z *= sample_normal_distribution(1, target_random_multiplicativ_error_sigma)
    transformation.translation.z += sample_normal_distribution(0, target_random_additiv_error_sigma)

    transformation.rotation.x *= sample_normal_distribution(1, target_random_multiplicativ_error_sigma)
    transformation.rotation.x += sample_normal_distribution(0, target_random_additiv_error_sigma)
    transformation.rotation.y *= sample_normal_distribution(1, target_random_multiplicativ_error_sigma)
    transformation.rotation.y += sample_normal_distribution(0, target_random_additiv_error_sigma)
    transformation.rotation.z *= sample_normal_distribution(1, target_random_multiplicativ_error_sigma)
    transformation.rotation.z += sample_normal_distribution(0, target_random_additiv_error_sigma)
    transformation.rotation.w *= sample_normal_distribution(1, target_random_multiplicativ_error_sigma)
    transformation.rotation.w += sample_normal_distribution(0, target_random_additiv_error_sigma)

    return transformation

def read_tf(count):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        for id in range(0, count):
            try:
                transform_msg = tfBuffer.lookup_transform("loc_system_" + locSystemName, 'tb3_' + str(id) + '/base_footprint', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            #add errors here: additiv, multiplicativ
            transformation_with_error = add_error_to_transformation(transform_msg.transform)
            create_static_tf("loc_system_" + locSystemName,
                             "loc_system_" + locSystemName +"/target" + str(id),
                             transformation_with_error)
        topic_has_orientation.publish(1) # Orientation is correct
        topic_correct_mapping.publish(1)  #target1 = robot1 etc.
        create_static_tf('world', "loc_system_" + locSystemName, create_transform_msg((loc_system_offset_x, loc_system_offset_y, 0), (0, 0, 0, 1)))
        rate.sleep()

if __name__ == '__main__':
    topic_has_orientation = rospy.Publisher('loc_system_meta_'+locSystemName+'/has_orientation', Int8, queue_size=1)
    topic_correct_mapping = rospy.Publisher('loc_system_meta_'+locSystemName+'/correct_mapping', Int8, queue_size=1)
    rospy.init_node('fakeLocalisation')
    botCount = rospy.get_param('~bot_count')
    loc_system_offset_x = rospy.get_param('~loc_system_offset_x')
    loc_system_offset_y = rospy.get_param('~loc_system_offset_y')
    loc_system_scale_x = rospy.get_param('~loc_system_scale_x')
    loc_system_scale_y = rospy.get_param('~loc_system_scale_y')
    target_random_multiplicativ_error_sigma = rospy.get_param('~target_multiplicativ_sigma')
    target_random_additiv_error_sigma = rospy.get_param('~target_additiv_sigma')
    print("Bot Count: " + str(botCount))

    read_tf(botCount)

