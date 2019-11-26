#!/usr/bin/env python
import rospy
import tf_conversions
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Int8
import numpy as np

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

locSystemName = "fakelocalisation"

def create_tf(tf_broadcaster, parent, child, transform): #((x,y,z), (q1,q2,q3,q4))):
   # print("Parent: " + parent + " Child: " + child)
    if(type(transform) is not tuple):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform = transform
        tf_broadcaster.sendTransform(t)

def create_transform_msg((x,y,z),(qx,qy,qz,qw)):
    t = Transform(Vector3(x,y,z), Quaternion(qx,qy,qz,qw))
    return t

def sample_normal_distribution(mu, sigma):
    s = float(np.random.normal(mu, sigma, 1))
    return s

def add_noise_to_transformation(transformation):
    #add noise on translation
    transformation.translation.x *=  sample_normal_distribution(1, target_random_multiplicative_noise_translation_sigma)
    transformation.translation.x += sample_normal_distribution(0, target_random_additive_noise_translation_sigma)
    transformation.translation.y *= sample_normal_distribution(1, target_random_multiplicative_noise_translation_sigma)
    transformation.translation.y += sample_normal_distribution(0, target_random_additive_noise_translation_sigma)
    transformation.translation.z *= sample_normal_distribution(1, target_random_multiplicative_noise_translation_sigma)
    transformation.translation.z += sample_normal_distribution(0, target_random_additive_noise_translation_sigma)

    #calc euler from quaternion
    quaternion = (
        transformation.rotation.x,
        transformation.rotation.y,
        transformation.rotation.z,
        transformation.rotation.w)
    euler = list(euler_from_quaternion(quaternion)) #tuples are inmutable -> cast to list
    #add noise
    for i in range(2):
        euler[i] *= sample_normal_distribution(1,target_random_multiplicative_noise_rotation_sigma)
        euler[i] += sample_normal_distribution(0,target_random_additive_noise_rotation_sigma)
        #no z noise

    #calc quaternion from euler
    q = quaternion_from_euler(euler[0],euler[1],euler[2])

    #change format
    transformation.rotation.x = q[0]
    transformation.rotation.y = q[1]
    transformation.rotation.z = q[2]
    transformation.rotation.w = q[3]
    print(transformation)
    return transformation

def update_tf(tf_buffer, tf_broadcaster, bot_count):
        for id in range(0, bot_count):
            try: #get tf from loc_system_locSytemName -> World -> .. -> tb3_id/base_footprint
                transform_msg = tf_buffer.lookup_transform("loc_system_" + locSystemName, 'tb3_' + str(id) + '/base_footprint', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue #if not possible try next

            transformation_with_noise = add_noise_to_transformation(transform_msg.transform)

            #loc_system -> target
            create_tf(tf_broadcaster, "loc_system_" + locSystemName,
                             "loc_system_" + locSystemName +"/target" + str(id),
                             transformation_with_noise)

        #world -> loc_system
        create_tf(tf_broadcaster, 'world', "loc_system_" + locSystemName,
                         (create_transform_msg((loc_system_offset_x, loc_system_offset_y, 0), (0, 0, 0, 1))))

def publish_metadata(has_orientation, correct_mapping):
    topic_has_orientation.publish(has_orientation)
    topic_correct_mapping.publish(correct_mapping)

if __name__ == '__main__':
    rospy.init_node('fakeLocalisation')
    #create topic publisher
    topic_has_orientation = rospy.Publisher('loc_system_meta_' + locSystemName + '/has_orientation', Int8, queue_size=1)
    topic_correct_mapping = rospy.Publisher('loc_system_meta_' + locSystemName + '/correct_mapping', Int8, queue_size=1)

    #load all parameters:
    bot_count = rospy.get_param('~bot_count')
    loc_system_offset_x = rospy.get_param('~loc_system_offset_x')
    loc_system_offset_y = rospy.get_param('~loc_system_offset_y')
    loc_system_scale_x = rospy.get_param('~loc_system_scale_x')
    loc_system_scale_y = rospy.get_param('~loc_system_scale_y')
    target_random_multiplicative_noise_translation_sigma = rospy.get_param('~target_multiplicative_noise_translation_sigma')
    target_random_additive_noise_translation_sigma = rospy.get_param('~target_additive_noise_translation_sigma')
    target_random_multiplicative_noise_rotation_sigma = rospy.get_param('~target_multiplicative_noise_rotation_sigma')
    target_random_additive_noise_rotation_sigma = rospy.get_param('~target_additive_noise_rotation_sigma')

    #create tf buffer
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    #create tf broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    has_orientation = True
    correct_mapping = True

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        update_tf(tf_buffer, tf_broadcaster, bot_count)
        publish_metadata(has_orientation, correct_mapping)
        rate.sleep()
