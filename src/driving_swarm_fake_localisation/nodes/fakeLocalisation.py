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
from gazebo_msgs.srv import GetModelState

mapping = []
# broadcast a transformation
def broadcast_tf(tf_broadcaster, parent, child, transform):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform = transform
        tf_broadcaster.sendTransform(t)

def create_transform_msg((x,y,z),(qx,qy,qz,qw)):
    t = Transform(Vector3(x,y,z), Quaternion(qx,qy,qz,qw))
    return t

#returns a sample from normal distribution
def sample_normal_distribution(mu, sigma):
    s = float(np.random.normal(mu, sigma, 1))
    return s

# returns a scaled transformation
def scale_transformation(transformation):
    transformation.translation.x *= target_scale_x;
    transformation.translation.y *= target_scale_y;
    return transformation

# returns a transformation with added noise.
def add_noise_to_transformation(transformation):
    #add noise on translation
    transformation.translation.x *= sample_normal_distribution(1, target_random_multiplicative_noise_translation_sigma)
    transformation.translation.x += sample_normal_distribution(0, target_random_additive_noise_translation_sigma)
    transformation.translation.y *= sample_normal_distribution(1, target_random_multiplicative_noise_translation_sigma)
    transformation.translation.y += sample_normal_distribution(0, target_random_additive_noise_translation_sigma)

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
    #print(transformation)
    return transformation

# reads TF from simulation and updates TF
def update_tf(tf_buffer, tf_broadcaster, bot_count, locSystemName, loc_system_rotation_angle, model_state_proxy):
        for id in range(bot_count): #0,1,2,..N-1
            model_state = model_state_proxy('tb3_' + str(id+1), '')

            if not model_state.success:
                continue

            transform = Transform()
            transform.translation = model_state.pose.position
            transform.rotation = model_state.pose.orientation

            #scale and add nose to the data
            transformation_with_noise = add_noise_to_transformation(scale_transformation(transform))

            #TF: loc_system -> target
            broadcast_tf(tf_broadcaster, "loc_system_" + locSystemName,
                             "loc_system_" + locSystemName +"/target" + mapping[id],
                             transformation_with_noise)

        #TF: world -> loc_system
        q = quaternion_from_euler(0, 0, loc_system_rotation_angle)
        broadcast_tf(tf_broadcaster, 'world', "loc_system_" + locSystemName,
                         (create_transform_msg((loc_system_offset_x, loc_system_offset_y, 0), (q[0], q[1], q[2], q[3]))))

#publishes data to the metadata topic
def publish_metadata(has_orientation, correct_mapping, accuracy):
    localisation_meta_msg = localisation_meta()
    localisation_meta_msg.has_orientation = has_orientation
    localisation_meta_msg.correct_mapping = correct_mapping
    localisation_meta_msg.accuracy = accuracy
    topic_metadata.publish(localisation_meta_msg);

#generates an array from 0 to botcount with rising(correct = true) or random (correct = false) order
def genMapping(bot_count, correct_mapping = False):
    mapping = np.arange(1, bot_count+1)
    if correct_mapping:
       return np.array(mapping).astype('str')
    else:
       return np.random.permutation(np.array(mapping)).astype('str')

if __name__ == '__main__':

    #read the locsystemname as arg
    if len(sys.argv) < 2:
        print("please pass name as arg, using default: fakelocalisation")
        loc_system_name = "fakelocalisation"
    else:
        loc_system_name = sys.argv[1]

    rospy.init_node(loc_system_name)

    #read all local parameters:
    bot_count = rospy.get_param('~bot_count')
    loc_system_offset_x = rospy.get_param('~loc_system_offset_x')
    loc_system_offset_y = rospy.get_param('~loc_system_offset_y')
    loc_system_scale_x = rospy.get_param('~loc_system_scale_x')
    loc_system_scale_y = rospy.get_param('~loc_system_scale_y')
    loc_system_rotation_angle = rospy.get_param('~loc_system_rotation_angle')

    target_random_multiplicative_noise_translation_sigma = rospy.get_param('~target_multiplicative_noise_translation_sigma')
    target_random_additive_noise_translation_sigma = rospy.get_param('~target_additive_noise_translation_sigma')
    target_random_multiplicative_noise_rotation_sigma = rospy.get_param('~target_multiplicative_noise_rotation_sigma')
    target_random_additive_noise_rotation_sigma = rospy.get_param('~target_additive_noise_rotation_sigma')
    target_scale_x = rospy.get_param('~target_scale_x')
    target_scale_y = rospy.get_param('~target_scale_y')

    update_rate = rospy.get_param('~update_rate')

    meta_has_orientation = rospy.get_param('~meta_has_orientation')
    meta_correct_mapping = rospy.get_param('~meta_correct_mapping')
    meta_accuracy = rospy.get_param('~meta_accuracy')

    #create topic publisher
    topic_metadata = rospy.Publisher('loc_system_meta_' + loc_system_name, localisation_meta,  queue_size=1)

    #create tf buffer
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    #create tf broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    #generate target mapping (random/correct)
    mapping = genMapping(bot_count, meta_correct_mapping)

    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                         GetModelState)

    rate = rospy.Rate(update_rate) #Hz

    #main loop:
    while not rospy.is_shutdown():
        #update tf
        update_tf(tf_buffer, tf_broadcaster, bot_count, loc_system_name, loc_system_rotation_angle, get_model_state)
        #update meta topic
        publish_metadata(meta_has_orientation, meta_correct_mapping, meta_accuracy)
        rate.sleep()
