#!/usr/bin/env python 
import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.pose)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tb3_0/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
