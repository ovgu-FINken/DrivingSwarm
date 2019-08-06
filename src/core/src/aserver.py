#!/usr/bin/env python

import rospy
import actionlib

from core.msg import 

class Sender:
    def __init__(self):
        self.pub = rospy.Publisher("commands", String, queue_size=1)
        
    def readDefaultFile(self):
if __name__ == '__main__':
		rospy.init_node('sender')
		sender = Sender()
		rospy.spin()