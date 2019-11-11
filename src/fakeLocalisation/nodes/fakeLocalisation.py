#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg

locSystemName = "fakelocalisation"

def create_static_tf(parent, child, ((x,y,z), (q1,q2,q3,q4))):

    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()

    t.header.frame_id = parent
    t.child_frame_id = child

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    t.transform.rotation.x = q1
    t.transform.rotation.y = q2
    t.transform.rotation.z = q3
    t.transform.rotation.w = q4

    br.sendTransform(t)

def read_tf(count):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for id in range(0, count):
            try:
                pose = tfBuffer.lookup_transform('loc_system_meta_" + locSystemName', 'tb3_' + str(id) + '/odom', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            create_static_tf("loc_system_meta_" + locSystemName,
                             "loc_system_meta_" + locSystemName +"/target" + id,
                             pose)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fakeLocalisation')
    botCount = rospy.get_param('~bot_count')
    print("Bot Count: " + str(botCount))
    create_static_tf("world", "loc_system_meta_" + locSystemName, ((0, 0, 0), (0, 0, 0, 1)))
    read_tf(botCount)

    rospy.spin()