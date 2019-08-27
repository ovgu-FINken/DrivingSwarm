#!/bin/bash
source ~/.bashrc
NUM=0
tmux new-session -s turtlebots -d
tmux rename-window -t turtlebots "robot"
tmux send-keys -t turtlebots:$NUM "roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=$ROS_HOSTNAME set_lidar_frame_id:=$ROS_HOSTNAME/base_scan" C-m
NUM=$((++NUM))
read -t 10
tmux new-window -t turtlebots -n "remote"
tmux send-keys -t turtlebots:$NUM "roslaunch turtlebot3_bringup turtlebot3_remote.launch multi_robot_name:=$ROS_HOSTNAME" C-m
NUM=$((++NUM))
tmux new-window -t turtlebots -n "teleop"
tmux send-keys -t turtlebots:$NUM "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch multi_robot_name:=$ROS_HOSTNAME" C-m
NUM=$((++NUM))

tmux attach
