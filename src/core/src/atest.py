#!/usr/bin/env python

# FOR TESTING aserver/-client

import os

import rospy

from std_msgs.msg import String
from core.srv import BehaviourStatus

class TestAClient:

    def __init__(self):
        self.pub = rospy.Publisher('test_aclient', String, queue_size=10)
        self.pub.publish("started")
        self.service = rospy.Service('atest', BehaviourStatus, self.test_cb)

        self.hello_str = "hello world " + str(rospy.get_time())
        rospy.loginfo(self.hello_str)
        self.pub.publish(self.hello_str)

    def test_cb(self, msg):
        self.hello_str = "hello world " + str(rospy.get_time())
        self.pub.publish(self.hello_str)
        return [1, self.hello_str]

if __name__ == '__main__':
    rospy.init_node('atest')
    atest = TestAClient()
    rospy.spin()

