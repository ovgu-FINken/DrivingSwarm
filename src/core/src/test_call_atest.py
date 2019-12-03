#!/usr/bin/env python

import rospy
from core.srv import BehaviourStatus
from core.msg import BehaviourGoal, BehaviourActionGoal

print("started: "rospy.get_namespace()+"/test.py")

rospy.wait_for_service('/turtlebook1/atest')

atest = rospy.ServiceProxy('/turtlebook1/atest', behav)

for i in enumerate(10):
    print( atest(i))

print("ended")
