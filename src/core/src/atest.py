#!/usr/bin/env python

# FOR TESTING aserver/-client

import os

import rospy

from std_msgs.msg import String
from core.srv import BehaviourStatus

class TestAClient:

    def __init__(self):
        self.pub = rospy.Publisher('test_aclient', String, queue_size=10)
        self.pub.publish('started')

        self.service = rospy.Service('atest', BehaviourStatus, self.get_status)
        self.service = rospy.Service('test_call', BehaviourStatus, self.test_cb)

        self.max_calls = rospy.get_param('~max_calls', 11)
        self.current_calls = 0

        self.hello_str = 'hello world ' + str(rospy.get_time())
        self.prog = 0
        self.status = "PRO start"
        #rospy.logwarn(self.hello_str)
        #self.pub.publish(self.hello_str)

    def test_cb(self, msg):
        #self.status = msg.status
        self.current_calls += 1
        self.hello_str = "hello world " + str(rospy.get_time())
        self.pub.publish(self.hello_str)
        return [self.prog, self.hello_str]

    def get_status(self, msg):
        if self.current_calls > self.max_calls:
            self.prog = 100
            self.status = "SUC done"
        self.prog=self.current_calls
        rospy.loginfo("status requested")
        return [self.prog, self.status]

if __name__ == '__main__':
    rospy.init_node('atest')
    atest = TestAClient()
    rospy.spin()

