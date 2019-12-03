#!/usr/bin/env python

import rospy
from core.srv import BehaviourStatus
from core.msg import BehaviourGoal, BehaviourActionGoal

print("started: "+rospy.get_namespace()+"/test.py")

rospy.wait_for_service('/turtlebot2/test_call')

test_call = rospy.ServiceProxy('/turtlebot2/test_call', BehaviourStatus)

for i in range(10):
    print( test_call("PRO wow"))
    rospy.sleep(5)

print( test_call("SUC i did it"))
print("ended")
