#!/usr/bin/env python

# FOR TESTING aserver/-client

import os

import rospy

from std_msgs.msg import String

class TestAClient:

    def __init__(self):
        pub = rospy.Publisher('test_aclient', String, queue_size=10)
        rate = rospy.Rate(10)
        pub.publish("started")
        self.service = rospy.Service('atest', core.srv.atest, test_cb)
        
        while not rospy.is_shutdown():
            hello_str = "hello world " + str(rospy.get_time())
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('atest')
    test_aclient = TestAClient()
    rospy.spin()

