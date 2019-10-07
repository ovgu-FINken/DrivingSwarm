#!/usr/bin/env python
import os
from datetime import datetime

import rospy
import actionlib
import yaml

from core.msg import BehaviourAction, BehaviourGoal

class BehaviourAClient:
    def __init__(self):
        # open behaviour_list which contains all goals and the respective service calls
        # behaviour_name: service_call
        goalfile = open('../cfg/behaviour_list.yaml', 'r')
        self.goals = yaml.load(goalfile)
        goalfile.close()

        # mode swiches between interactive mode (tui) and non (.yaml file)
        self.mode = rospy.get_param('~inter_mode', False)

        # start service / load behaviour_flow for sequencing of behaviours
        if self.mode:
            self.inter_srv = rospy.Service(rospy.get_namespace()+'behaviour_aclient_api', core.srv.BehaviourAPI, self.api_handler )
        else:
            # flowfile must be of type (DOUBLE QUOTES ARE IMPORTANT HERE)
            # - ['behaviour_name', "['pkg','launch.file','arg1:=val','arg2:=val', ...]"]
            # - ['behaviour2_name', ...]
            filedir = os.path.dirname(__file__)
            flowfile = open(os.path.join(filedir,'../cfg/behaviour_flow.yaml'), 'r')
            self.flow = yaml.load(flowfile)
            flowfile.close()

            try:
                os.remove(os.path.join(filedir, '../cfg/flow.log'))
            except:
                rospy.loginfo('[behaviour_aclient]: no old flow.log found')

        # initialise on all servers (turtlebots)
        # TODO
            
            for i, namespace in self.names
                self.a_clients[i] = actionlib.SimpleActionClient(namespace+'behaviour', BehaviourAction)


        rospy.loginfo('[behaviour_aclient]: starting with interactive_mode ' + str(mode))
        client.wait_for_server()

        if not self.mode:
            self.flow_handler()


# handles behaviour_flow-mode
    def flow_handler(self):
        filedir = os.path.dirname(__file__)
        self.log = open(os.path.join(filedir,'../cfg/flow.log'),'a')
        
        for behav in self.flow:
            self.flow_step = False
            goal = BehaviourGoal()
            goal.goal_name = behav[0]
            goal.goal_call = behav[1]
            
            for client in self.a_clients:
                client.send_goal(goal,  feedback_cb=self.flow_feedback, done_cb=self.flow_done)

            self.log.write('[' + datetime.now().time() + ']: ' + behav[0] + ' with call: ' + behav[1])

            rate = rospy.Rate(2)
            
            while not self.flow_step >= self.no_bots:
                rospy.sleep(rate)
                
            rospy.loginfo('[behaviour_aclient]: moving on to next behaviour in flow')

        rospy.loginfo('[behaviour_aclient]: flow done')
        log.close()
        
            
    def flow_feedback(self, feedback):
        self.log.write('['+datetime.now().time()+']: ' + ' PROG[' + feedback.prog_perc + '%] - ' + feedback.prog_status)
    
    def flow_done(self, term_state, result):
        succ = 'SUCCESS - ' if result.res_success else 'FAILURE - '
        self.log.write('['+datetime.now().time()+']: ' + ' DONE: ' + succ + result.res_msg)
        self.flow_step += 1

# TODO
# handler for api-requests
    def api_handler(self, req):
    # 0 = set a new goal (preempt/replace old one)
    # 1 = get feedback on goal
    # 2 = cancel active goal
    # 3 = try to reconnect bots
    # 3-4 = pause / unpause (in future)
        answer = {'answ_type':0, 'answ_ns':[],'answ_msg':[]}
        # TODO
        if req.req_type == 0:
            # correct yaml is sent by tui (cf flowfile)
            behav = yaml.load(req.req_msg)
                    
            goal = BehaviourGoal()
            goal.goal_name = behav[0]
            goal.goal_call = behav[1]
            send_goal()
        
        return answer
#TODO
    def ap_feedback(self, feedback):
#TODO
    def api_done(self, result):



if __name__ == '__main__':
    rospy.init_node('behaviour_aclient')
    behaviour_aclient = BehaviourAClient()
    rospy.spin()
