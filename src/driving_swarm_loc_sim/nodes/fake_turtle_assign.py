#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

# e.g. : loc_system_fake_UWB/target1 --> loc_system_fake_UWB/tb3_1

def static_transform(_from, _to):
    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = _from
    t.child_frame_id = _to
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)
    return


if __name__ == '__main__':
    rospy.init_node("fakeUwbTurtlebotAssing")

    uwb_tf_prefix = rospy.get_param('~uwb_tf_prefix') #loc_system_fake_UWB/
    bot_count = rospy.get_param('~bot_count')
    tb3_module_id_dict = rospy.get_param('~tb3_module_id_dict')
    update_rate = rospy.get_param('~update_rate')
    rate = rospy.Rate(update_rate) #Hz

    while not rospy.is_shutdown():
        try:
            for i in range(1, bot_count+1):
                moduleID = tb3_module_id_dict["tb3_" + str(i)]
                static_transform(uwb_tf_prefix + "target" + str(moduleID), uwb_tf_prefix + "tb3_" + str(i))
        except:
            rospy.logerr("no matching id entry found ... abort")

        rate.sleep()
