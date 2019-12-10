#!/usr/bin/env python

# RUNS ON THE TURTLEBOT

import os

import rospy
import roslaunch
import actionlib
import yaml

from core.msg import BehaviourAction, BehaviourActionFeedback, BehaviourActionResult
from core.srv import BehaviourStatus

# default action-values, need non-default values

SUCCEEDED = BehaviourActionResult()
SUCCEEDED.result.res_success = True
SUCCEEDED.result.res_msg = 'no problems here'
SUCCEEDED.result.ns = rospy.get_namespace()

FAILURE = BehaviourActionResult()
FAILURE.result.res_success = False
FAILURE.result.res_msg = 'insert your error here'
FAILURE.result.ns = rospy.get_namespace()

PREEMPT = BehaviourActionResult()
PREEMPT.result.res_success = False
PREEMPT.result.res_msg = 'goal has been preempted'
PREEMPT.result.ns = rospy.get_namespace()

PROGRESS = BehaviourActionFeedback()
PROGRESS.feedback.prog_perc = 0
PROGRESS.feedback.prog_status = 'default progress'
PROGRESS.feedback.ns = rospy.get_namespace()

class BehaviourAServer:
    def __init__(self):
        # open behaviour_list which contains all goals and the respective service_calls:
        # behaviour_name: service_call
        filedir = os.path.dirname(__file__)
        behav_file = open(os.path.join(filedir, '../cfg/behaviour_list.yaml'), 'r')
        self.behav_list = yaml.safe_load(behav_file)
        behav_file.close()


        # timeout for looking for service of launched node
        self.srv_timeout = rospy.get_param('~srv_timeout',60)
        self.name = rospy.get_namespace()

        # new simpleactionserver
        self.a_server=actionlib.SimpleActionServer('behaviour_aserver', BehaviourAction, self.execute, False)
        rospy.logwarn('[behaviour_aserver]: starting with timeout '+str(self.srv_timeout))
        self.a_server.start()

    def execute(self, behav):
        # if sent behav_name is not a valid/known action don't do anything
        if behav.behav_name not in self.behav_list[behav.behav_pkg]:
            FAILURE.result.res_msg = 'invalid action'
            rospy.logwarn('Sent behav_name is not a known behaviour (in behav_list)')
            self.a_server.set_succeeded(FAILURE)
            return
        rospy.logwarn('Preparing roslaunch for [' + behav.behav_name + ']')
        # prepare roslaunch
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # behav_call has to be in the style of ['launch.file','arg1:=val','arg2:=val', ...]
        roslaunch_call = yaml.safe_load(behav.behav_call)
        roslaunch_args = ''

        if len(roslaunch_call) > 1:
            for arg in roslaunch_call[2:]:
                roslaunch_args += arg + ' '

        roslaunch_command = 'roslaunch ' + behav.behav_pkg + ' ' + roslaunch_call[0] + ' ' + roslaunch_args

        roslaunch_p = subprocess.Popen(roslaunch_command, shell=True)

        # OLD
        #roslaunch_call_file = roslaunch.rlutil.resolve_launch_arguments(roslaunch_call)[0]
        #if len(roslaunch_call) > 1:
        #    roslaunch_call_args = roslaunch_call[1:]
        #else:
        #    roslaunch_call_args = []

        #parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_call_file, roslaunch_args=[roslaunch_call_args])
        #parent.start()
        
        rospy.logwarn('Checking ' + roslaunch_command)

        state = roslaunch_p.poll()
        if state is None:
            rospy.logwarn('Running fine')
        elif state < 0:
            rospy.loginfo('Terminated with error')
            FAILURE.result.res_msg = 'failed to start with roslaunch'
            self.a_server.set_aborted(FAILURE)
        elif state > 0:
            rospy.loginfo('Terminated')
            FAILURE.result.res_msg = 'started but terminated with roslaunch'
            self.a_server.set_aborted(FAILURE)

        service_name = "/" + rospy.get_namespace() + "/" + behav.behav_name

        # launched file needs to start a service within 60 seconds
        try:
            rospy.wait_for_service(service_name, timeout=self.srv_timeout)
        except rospy.ROSException:
            FAILURE.result.res_msg = 'failed to start behaviour_service in 60s'
            self.a_server.set_aborted(FAILURE)
            return
            # CONVENTION: use predefined service class

        behav_service = rospy.ServiceProxy(service_name, BehaviourStatus)

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

            # TODO: document service messages
            # message: err <errmessage>
            # service node has encountered an error
            if status_msg[:3] == 'ERR':
                FAILURE.result.res_msg = status_msg[3:]
                self.a_server.set_aborted(FAILURE)
                parent.stop()
                return
            # service node succeeded
            elif status_msg[:3] == 'SUC':
                SUCCESS.result.res_msg = status_msg[4:]
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
