#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import yaml
from driving_swarm_msgs.msg import localisation_meta


def broadcast_tf(tf_broadcaster, parent, child, x, y):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform = geometry_msgs.msg.Transform()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.w = 1
        tf_broadcaster.sendTransform(t)


# Finds tf frames with target_string in the name
# where the frame and the parent frame also have
# 'prefix' as prefix
# e.g.  loc_system_a/target1 -> loc_system_a/tb3_1
# Will be found with prefix = 'loc_system_a' and
# target_string = 'tb3_'
def find_frames(frames, prefix, target_string):
    target_frames = []

    for k, v in frames.items():
        if prefix in k and target_string in k and prefix in v["parent"]:
            target_frames.append(k)

    return target_frames


def update_tf(tf_buffer, tf_broadcaster, bot_count):

        # Save all transforms with weight so they can be combined later
        transforms = {}
        for loc_system, meta in loc_systems.items():
            for frame in meta['frames']:
                try:  # lookup from world to loc_system_XX/targetN
                    tf_from = "world"
                    tf_to = frame
                    transform_msg = tf_buffer.lookup_transform(tf_from,
                                                               tf_to,
                                                               rospy.Time(0))

                    # Extract targte name
                    target_name = ""
                    try:
                        target_name = frame.split('/')[1]
                    except IndexError:
                        # Wrong frame
                        continue

                    if target_name not in transforms:
                        transforms[target_name] = \
                                  [(transform_msg, meta['weight'])]
                    else:
                        transforms[target_name].append(
                                   (transform_msg, meta['weight']))

                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    continue  # if not possible try next

            # Combine transforms
            for target, bot_transforms in transforms.items():
                pos = [0, 0]
                weight_sum = 0
                for transform, weight in bot_transforms:
                    pos[0] = pos[0] + \
                        transform.transform.translation.x * weight
                    pos[1] = pos[1] + \
                        transform.transform.translation.y * weight
                    weight_sum = weight_sum + weight
                if weight_sum != 0:
                    pos[0] = pos[0] / weight_sum
                    pos[1] = pos[1] / weight_sum

                broadcast_tf(tf_broadcaster, 'world', target, pos[0], pos[1])


def publish_metadata(has_orientation, correct_mapping, accuracy):
    localisation_meta_msg = localisation_meta()
    localisation_meta_msg.has_orientation = has_orientation
    localisation_meta_msg.correct_mapping = correct_mapping
    localisation_meta_msg.accuracy = accuracy
    topic_metadata.publish(localisation_meta_msg)


if __name__ == '__main__':

    loc_system_name = "combined_output"

    meta_has_orientation = True
    meta_correct_mapping = True
    meta_accuracy = 1

    rospy.init_node(loc_system_name)

    # get params
    # Expected format:
    # { 'loc_system_a': {'weight': 0.7}, 'loc_system...}
    loc_systems = rospy.get_param('~loc_systems')

    # create topic publisher
    topic_metadata = rospy.Publisher('loc_system_meta_' + loc_system_name,
                                     localisation_meta,
                                     queue_size=1)

    # create tf buffer
    tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)

    # create tf broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10)  # Hz

    # main loop:
    while not rospy.is_shutdown():
        frames = tf_buffer.all_frames_as_yaml()
        frames = yaml.load(frames)

        # Find all turtlebot frames localized by each system
        for loc_system, meta in loc_systems.items():
            loc_frames = find_frames(frames, loc_system, 'tb3_')
            meta['frames'] = loc_frames

        # update tf
        update_tf(tf_buffer, tf_broadcaster, loc_systems)
        # update meta topic
        publish_metadata(meta_has_orientation,
                         meta_correct_mapping,
                         meta_accuracy)
        rate.sleep()
