#!/usr/bin/env python

import rospy
from core.srv import BehaviourStatus
from core.msg import BehaviourGoal, BehaviourActionGoal

print("started: "+rospy.get_namespace()+"/test.py")

rospy.wait_for_service('/turtlebook1/atest')

atest = rospy.ServiceProxy('/turtlebook1/atest', BehaviourStatus)

for i in range(10):
    print( atest())

print("ended")
