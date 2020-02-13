#!/bin/sh

if [ $# -ne 5 ]
  then
	echo "Wrong number of args"
	echo "args are $@"
	exit
fi

echo "Call with:"
echo "ros_port:=$1"
echo "gazebo_port:=$2"
echo "multi_robot_name:=$3"
echo "mcast_group:=$4"
echo "mcast_port:=$5"

export ROS_MASTER_URI="http://localhost:$1"
export GAZEBO_MASTER_URI="http://localhost:$2"

#export ROS_HOSTNAME="$3"

roslaunch core additional_master_gazebo.launch ros_port:=$1 gazebo_port:=$2 multi_robot_name:=$3 mcast_group:=$4 mcast_port:=$5

echo "ALL DONE"
