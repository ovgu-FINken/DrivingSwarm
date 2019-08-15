#!/bin/bash
tmux new-session -s turtlebots -d
tmux rename-window -t turtlebots "gazebo"
tmux send-keys -t turtlebots:0 "roslaunch tracking_certainty_grid simulation.launch" C-m
sleep 5
tmux new-window -t turtlebots -n "slam 0"
tmux send-keys -t turtlebots:1 "ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_0/base_footprint set_odom_frame:=tb3_0/odom set_map_frame:=tb3_0/map" C-m
tmux new-window -t turtlebots -n "slam 1"
tmux send-keys -t turtlebots:2 "ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_1/base_footprint set_odom_frame:=tb3_1/odom set_map_frame:=tb3_1/map" C-m
tmux new-window -t turtlebots -n "slam 2"
tmux send-keys -t turtlebots:3 "ROS_NAMESPACE=tb3_2 roslaunch turtlebot3_slam turtlebot3_gmapping.launch set_base_frame:=tb3_2/base_footprint set_odom_frame:=tb3_2/odom set_map_frame:=tb3_2/map" C-m
tmux new-window -t turtlebots -n "certainty grid"
tmux send-keys -t turtlebots:4 "roslaunch tracking_certainty_grid tracking_simulation.launch" C-m
tmux new-window -t turtlebots -n "rqt"
tmux send-keys -t turtlebots:5 "rqt" C-m
tmux new-window -t turtlebots -n "rviz"
tmux send-keys -t turtlebots:6 "ROS_NAMESPACE=tb3_0 rviz" C-m
tmux attach
