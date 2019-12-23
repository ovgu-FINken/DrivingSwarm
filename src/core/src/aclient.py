s#!/usr/bin/env python

# RUNS ON THE TURTLEBOOK

import os
from datetime import datetime

import rospy
import actionlib
import yaml

from core.msg import BehaviourAction, BehaviourGoal
from core.srv import BehaviourAPI

class BehaviourAClient:
    def __init__(self):
        filedir = os.path.dirname(__file__)
        # open behaviour_list which contains all behavs in respective packages
        #pkg_name:
        #  -service_call

        behav_file = open(os.path.join(filedir,'../cfg/behaviour_list.yaml'), 'r')
        self.behav_list = yaml.safe_load(behav_file)
        behav_file.close()

        # mode swiches between interactive mode (tui) and non (.yaml file)
        self.mode = rospy.get_param('~inter_mode', False)

        # timeout for waiting for action clients
        self.action_timeout = rospy.Duration(rospy.get_param('~action_timeout', 60))

        # TODO timeout for TUI?

        # start service / load behaviour_flow for sequencing of behaviours
        if self.mode:
            self.inter_srv = rospy.Service('behaviour_aclient_api', BehaviourAPI, self.api_handler )
            rospy.logwarn('NOT IMPLEMENTED YET')
        else:
            # flow_file must be of type
            # - ['behaviour_name', 'pkg', "['file.launch','arg1:=val','arg2:=val', ...]"]
            # - ['behaviour2_name', ...]
            # (DOUBLE QUOTES (") ARE IMPORTANT HERE)

            flow_file = open(os.path.join(filedir,'../cfg/behaviour_flow.yaml'), 'r')
            self.flow = yaml.safe_load(flow_file)
            flow_file.close()

            #try:
            #    os.remove(os.path.join(filedir, '../cfg/flow.log'))
            #except:
            #    rospy.loginfo('[INIT] no old flow.log found')

        # initialise on all servers (turtlebots)
            bot_file = open(os.path.join(filedir,'../cfg/bot_list.yaml'), 'r')
            self.names = yaml.safe_load(bot_file)
            bot_file.close()

            #all action clients
            self.a_clients = []
            #number of bots
            self.no_bots = 0
            for namespace in self.names:
               self.no_bots += 1
               self.a_clients.append(actionlib.SimpleActionClient( '/' + namespace +'/behaviour_aserver', BehaviourAction))

        rospy.logwarn('[INIT] Starting with interactive_mode ' + str(self.mode))

        for a_client in self.a_clients:
            rospy.logwarn('[INIT] Waiting for ' + a_client.action_client.ns)
            if not a_client.wait_for_server(timeout=self.action_timeout):
                self.a_clients.remove(a_client)
                self.no_bots -= 1

        rospy.logwarn(self.a_clients)

        if not self.no_bots > 0:
            rospy.logwarn('[INIT] a_clients is empty (all unreachable)')
            return 1

        if not self.mode:
            rospy.logwarn('[INIT] Entering [FLOW] mode')
            self.flow_handler()

# handles behaviour_flow-mode
    def flow_handler(self):
        #filedir = os.path.dirname(__file__)
        #self.log = open(os.path.join(filedir,'../cfg/flow.log'),'a')

        for flow_item in self.flow:

            try:
                if flow_item[0] not in self.behav_list[flow_item[1]]:
                    rospy.signal_shutdown('[ERROR] ' +flow_item[0]+ ' not in behaviour_list (' +flow_item[1]+ ')')
                    exit()
            except Exception as e:
                rospy.signal_shutdow('[ERROR] (' + str(type(e))+str(e) + ') while looking up ' + flow_item[0] + ' in pkg ' + flow_item[1])
                exit()

            self.flow_step = 0
            behav_goal = BehaviourGoal()
            behav_goal.behav_name = flow_item[0]
            behav_goal.behav_pkg = flow_item[1]
            behav_goal.behav_call = flow_item[2]

            rospy.logwarn("[FLOW] " + flow_item[0])

            for a_client in self.a_clients:
                rospy.logwarn('[FLOW] Sending to ' + a_client.action_client.ns)
                a_client.send_goal(behav_goal, feedback_cb=self.flow_feedback, done_cb=self.flow_done)

            #self.log.write('[' + str(datetime.now().time()) + ']: ' + str(flow_item[0]) + ' with call: ' + str(flow_item[1]))

            rate = rospy.Rate(2)
            rospy.logwarn('[FLOW] Sleep until flow is done')
            while not self.flow_step >= self.no_bots:
                rate.sleep()

            rospy.logwarn('[FLOW] moving on to next behaviour in flow')

        rospy.logwarn('[FLOW] flow done')
        #self.log.close()
        rospy.signal_shutdown('[FLOW] done') 

    def flow_feedback(self, feedback):
        rospy.logwarn('[FEEDBACK] [' +str(feedback.ns)+ '] ' + str(feedback.prog_status) +' '+ str(feedback.prog_perc))
        #self.log.write('['+str(datetime.now().time())+'][' + str(feedback.ns) + ']: ' + ' PROG[' + str(feedback.prog_perc) + '%] - ' + str(feedback.prog_status))

    def flow_done(self, term_state, result):
        rospy.logwarn('[FLOWDONE] ' + str(result.res_msg))
        #succ = 'SUCCESS - ' if result.res_success else 'FAILURE - '
        #self.log.write('['+str(datetime.now().time())+'][' + str(result.ns) + ']: ' + ' DONE ' + succ + result.res_msg)
        self.flow_step += 1

# TODO TODO
# handler for api-requests
    def api_handler(self, req):
    # 0 = set a new behav (preempt/replace old one)
    # 1 = get feedback on behav
    # 2 = cancel active behav
    # 3 = try to reconnect bots
    # 3-4 = pause / unpause (in future)
        answer = {'answ_type':0, 'answ_ns':[],'answ_msg':[]}
        # TODO
        if req.req_type == 0:
            # correct yaml is sent by tui (cf flow_file)
            behav = yaml.safe_load(req.req_msg)

            behav = BehaviourGoal()
            behav.behav_name = behav[0]
            behav.behav_pkg = behav[1]
            behav.behav_call = behav[2]
            send_behav()

        return answer
#TODO
    def api_feedback(self, feedback):
        rospy.logwarn("WIP")
#TODO
    def api_done(self, result):
        rospy.logwarn("WIP")


if __name__ == '__main__':
    rospy.init_node('behaviour_aclient')
    behaviour_aclient = BehaviourAClient()
    rospy.spin()
