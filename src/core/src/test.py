#!/usr/bin/env python

import rospy
from core.srv import BehaviourAPI
from core.msg import BehaviourGoal, BehaviourActionGoal

test = BehaviourAPI()

print(dir(test))

print(rospy.get_namespace())
