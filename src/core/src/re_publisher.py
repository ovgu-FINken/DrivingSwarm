#!/usr/bin/env python
# Node for re-publishing topics (with current ROS-time, disregarding previous time-stamps)

import rospy
from rospy import ROSException

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class RePublisher:
    def __init__(self):
        self.ns = rospy.get_namespace()
        self.params = {}
        self.params['list_sub_topics'] = rospy.get_param('~list_sub_topics','/bookturtle1/scan_gazebo').split(',')
        self.params['list_pub_topics'] = rospy.get_param('~list_pub_topics', '/bookturtle1/scan').split(',')
        self.params['list_message_types'] = rospy.get_param('~list_message_types', 'LaserScan').split(',')

        log_string = ''
        for name, value in self.params.iteritems():
            log_string += '- ' + name + ': ' + str(value) + '\n'
        rospy.loginfo('\n[dist_ea] init with:\n' + log_string[:-1])

        if not self.params['list_sub_topics'] or len(self.params['list_sub_topics']) != len(self.params['list_pub_topics']) or len(self.params['list_pub_topics']) != len(self.params['list_message_types']):
            raise ROSException('Invalid sub/pub/type-lists:\nSubs: ' + str(self.params['list_sub_topics']) + '\nPubs: ' + self.params['list_pub_topics'] + '\nTypes: ' + str(self.params['list_message_types']))

        self.subs = []
        self.pubs = {}

        for sub_topic, pub_topic, topic_type in zip(self.params['list_sub_topics'], self.params['list_pub_topics'], self.params['list_message_types']):
            #TODO USING EVAL
            exec('self.pubs[sub_topic] = rospy.Publisher(pub_topic, '+ topic_type +', queue_size=10)')
            eval('self.subs.append(rospy.Subscriber(sub_topic,'+ topic_type +', self.cb_scan))')

        rospy.loginfo('[re_publisher] Init done')

    def cb_scan(self, msg):
        msg.header.stamp = rospy.Time.now()
        self.pubs[msg._connection_header['topic']].publish(msg)

if __name__== '__main__':
    rospy.init_node('re_publisher')
    re_publisher = RePublisher()
    rospy.spin()
