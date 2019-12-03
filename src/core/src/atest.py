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
        self.service = rospy.Service('atest', BehaviourStatus, self.get_status)
        self.service = rospy.Service('test_call', BehaviourStatus, self.test_cb)

        self.hello_str = "hello world " + str(rospy.get_time())
        self.prog = 0
        self.status = "PRO such start"
        rospy.loginfo(self.hello_str)
        self.pub.publish(self.hello_str)

    def test_cb(self, msg):
        self.status = msg.status
        self.hello_str = "hello world " + str(rospy.get_time())
        self.pub.publish(self.hello_str)
        return [self.prog, self.hello_str]

    def get_status(self, msg):
        print("status requested")
        return [self.prog, self.status]

if __name__ == '__main__':
    rospy.init_node('atest')
    atest = TestAClient()
    rospy.spin()

