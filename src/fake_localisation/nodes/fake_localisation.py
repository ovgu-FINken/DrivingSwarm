#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Int8


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


def read_tf(count):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        for id in range(0, count):
            try:
                transform_msg = tfBuffer.lookup_transform('world', 'tb3_' + str(id) + '/odom', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            create_static_tf("loc_system_" + locSystemName,
                             "loc_system_" + locSystemName +"/target" + str(id),
                             transform_msg.transform)
	topic_has_orientation.publish(1) # Orientation is correct
	topic_correct_mapping.publish(1)  #target1 = robot1 etc.    
	rate.sleep()

if __name__ == '__main__':
    topic_has_orientation = rospy.Publisher('loc_system_meta_'+locSystemName+'/has_orientation', Int8, queue_size=1)
    topic_correct_mapping = rospy.Publisher('loc_system_meta_'+locSystemName+'/correct_mapping', Int8, queue_size=1)
    rospy.init_node('fakeLocalisation')
    botCount = rospy.get_param('~bot_count')
    print("Bot Count: " + str(botCount))
    create_static_tf("world", "loc_system_" + locSystemName, create_transform_msg((0, 0, 0), (0, 0, 0, 1)))

    read_tf(botCount)

    rospy.spin()
