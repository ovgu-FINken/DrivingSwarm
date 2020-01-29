#!/usr/bin/env python

# FOR TESTING PUBLISHING / MULTIMASTER

import rospy

from std_msgs.msg import String

rospy.init_node('test_pub')

pub = rospy.Publisher('test_pub', String, queue_size=10)
max_calls = rospy.get_param('~max_calls', 60)
current_calls = 0
rate = rospy.Rate(1)
hello_str = 'hello world ' + str(rospy.get_time())

rospy.loginfo('started')
rospy.loginfo(hello_str)
pub.publish('started')
pub.publish(hello_str)

while current_calls < max_calls:
    current_calls += 1
    hello_str = "hello world " + str(rospy.get_time())
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()

rospy.loginfo('done')
