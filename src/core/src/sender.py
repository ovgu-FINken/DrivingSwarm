#!/usr/bin/env python
# TODO: send commands to topic
# Create python script for cli to communicate with this script

# USE ACTIONLIBS i guess
#

# Communicate via subprocesses
# Choose mode (File vs CLI vs Hybrid)
import rospy

class Sender:
    def __init__(self):
        self.pub = rospy.Publisher("commands", String, queue_size=1)
        
    def readDefaultFile(self):
if __name__ == '__main__':
		rospy.init_node('sender')
		sender = Sender()
		rospy.spin()