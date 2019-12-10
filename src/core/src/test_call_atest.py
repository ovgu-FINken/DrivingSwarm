#!/usr/bin/env python

import rospy
from core.srv import BehaviourStatus
from core.msg import BehaviourGoal, BehaviourActionGoal

print('started: '+rospy.get_namespace()+'/test.py')

atest_ns = "turtlebot2"

service_string = '/'+atest_ns+'/test_call'

rospy.wait_for_service(service_string, timeout=30)

test_call = rospy.ServiceProxy(service_string, BehaviourStatus)

for i in range(10):
    print( test_call())#)i, 'PRO wow[' + str(i) + ']'))
    rospy.sleep(5)

print( test_call())#100, 'SUC i did it'))
print('ended')
