#!/usr/bin/env python 
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import actionlib 

current_goal = None
pub = None

def callback(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.pose)
    global current_goal
    current_goal = generate_follower_goal(msg.pose)
    

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/tb3_0/odom", Odometry, callback)

def talker():
    global pub
    pub = rospy.Publisher("/tb3_2/cmd_vel", Twist, queue_size=10)
    

def generate_follower_goal(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "world"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose.pose
    return goal


def generate_rando_goal():
    goal = Twist()
    goal.angular.z = random.uniform(-0.1,0.1)
    goal.linear.z = random.uniform(-1,1)
    return goal

if __name__ == '__main__':
    listener()
    talker()
    follower_client = actionlib.SimpleActionClient("/tb3_1/move_base", MoveBaseAction)
    follower_client.wait_for_server()
    rate = rospy.Rate(5)
    counter = 10
    while not rospy.is_shutdown():
        follower_client.send_goal(current_goal)
        rospy.loginfo(rospy.get_caller_id() + "Bot 1 got new goal %s", current_goal)
        counter -= 1
        if counter == 0:
            pub.publish(generate_rando_goal())
            counter = 10
        rate.sleep()
