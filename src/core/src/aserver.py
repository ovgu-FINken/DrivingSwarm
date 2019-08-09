#!/usr/bin/env python

import os

import rospy
import roslaunch
import actionlib
import yaml

from core.msg import BehaviourAction, BehaviourActionFeedback, BehaviourActionResult

# default action-values, need non-default values

SUCCEEDED = BehaviourActionResult()
SUCCEEDED.result.res_success = True
SUCCEEDED.result.res_msg = 'no problems here'

FAILURE = BehaviourActionResult()
FAILURE.result.res_success = False
FAILURE.result.res_msg = 'insert your error here'

PREEMPT = BehaviourActionResult()
PREEMPT.result.res_success = False
PREEMPT.result.res_msg = 'goal has been preempted'

PROGRESS = BehaviourActionFeedback()
PROGRESS.feedback.prog_perc = 0
PROGRESS.feedback.prog_status = 'default progress'

#TODO: add msg.ns for turtlebot_namespace

class BehaviourAServer:
    def __init__(self):
        # open behaviour_list which contains all goals and the respective service_calls:
        # behaviour_name: service_call
        filedir = os.path.dirname(__file__)
        goalfile = open(os.path.join(filedir, '../cfg/behaviour_list.yaml'), 'r')
        self.goals = yaml.load(goalfile)
        goalfile.close()

        # timeout for looking for service of launched node
        self.srv_timeout = rospy.get_param('~srv_timeout',60)

        self.name = rospy.get_namespace()
        
        # new simpleactionserver
        self.a_server=actionlib.SimpleActionServer(self.name+'behaviour', BehaviourAction, self.execute, False)
        rospy.loginfo('[behaviour_aserver]: starting with timeout '+str(self.srv_timeout))
        self.a_server.start()
        
    def execute(self, goal):
        # if sent goal_name is not a valid/known action don't do anything
        if goal.goal_name not in self.goals:
            FAILURE.result.res_msg = 'invalid action'
            self.a_server.set_succeeded(FAILURE)
            return

        # prepare roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # goal_call has to be in the style of ['pkg','launch.file','arg1:=val','arg2:=val', ...]
        roslaunch_call = yaml.load(goal.goal_call)
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
        roslaunch_args = cli_args[2:]

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, roslaunch_args=[roslaunch_args])
        parent.start()
        
        try:
            rospy.wait_for_service(self.goals[goal.goal_name], timeout=self.srv_timeout)
        except rospy.ROSException:
            FAILURE.result.res_msg = 'failed to start behaviour_service in 60s'
            self.a_server.set_aborted(FAILURE)
            return
            # service class = pkg.srv.ServiceName
            # using eval!

        behav_service = rospy.ServiceProxy(self.goals[goal.goal_name]), eval(goal.goal_name).srv.eval(self.goals[goal.goal_name])
    
        # TODO: implement start, stop and pause request (future)

        rate = rospy.Rate(2)    
            
        while True:
            # preemption request by a_client
            self.preempted = self.a_server.is_preempt_requested()
            if self.preempted:
                self.a_server.set_preempted(PREEMPT)
                parent.stop()
                return

            # service file must be of type
            # ---
            # uint8 perc
            # string msg
            status_answer = behav_service.get_status()
            status_perc = status_answer.perc
            status_msg = status_answer.msg
            
            # service node has encountered an error
            if status_msg[:5] == 'ERROR':
                FAILURE.result.res_msg = status_msg
                self.a_server.set_aborted(FAILURE)
                parent.stop()
                return         
            # service node succeeded
            elif status_msg[:7] == 'SUCCESS':
                SUCCESS.result.res_msg = status_msg
                self.a_server.set_succeeded(SUCCESS)
                parent.stop()
                return
            # service node progress
            else:
                PROGRESS.feedback.prog_perc = status_perc
                PROGRESS.feedback.prog_status = status_msg
                self.a_server.publish_feedback(PROGRESS)
            
            rate.sleep()

        FAILURE.result.res_msg= 'escaped while true'
        self.a_server.set_aborted(FAILURE)
        parent.stop()

if __name__ == '__main__':
		rospy.init_node('behaviour_aserver')
		behaviour_aserver = BehaviourAServer()
		rospy.spin()