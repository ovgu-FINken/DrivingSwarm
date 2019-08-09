#!/usr/bin/env python

import rospy
import actionlib
import yaml

from core.srv import BehaviourAClientAPI

class BehaviourAClient:
    def __init__(self):
        # open behaviour_list which contains all goals and respective calls
        actionfile = open('../cfg/behaviour_list.yaml', 'r')
        self.actions = yaml.load(actionfile)
        actionfile.close()

        # mode swiches between interactive mode (tui) and non (.yaml file)
        self.mode = rospy.get_param('~inter_mode', False)

        # load behaviour_flow for sequencing of behaviours
        if self.mode:
            self.inter_srv = rospy.Service('behaviour_aclient_api', core.srv.BehaviourAClientAPI, api_handler )
        else:
            flowfile = open('../cfg/behaviour_flow.yaml', 'r')
            self.flow = yaml.load(flowfile)
            flowfile.close()

        self.a_client = SimpleActionClient()
        
        rospy.loginfo('[behaviour_aclient]: starting with interactive_mode ' + str(mode))


    def 
    

    def api_handler(req):
    # 0 = set a new goal (preempt old one)
    # 1 = get feedback on goal
    # 2 = cancel active goal:
        answer = BehaviourAClientAPI()

        
        return answer
    
if __name__ == '__main__':
    rospy.init_node('behaviour_aclient')
    behaviour_aclient = BehaviourAClient()
    rospy.spin()