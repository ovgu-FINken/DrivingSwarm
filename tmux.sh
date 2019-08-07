#!/bin/bash
tmux new-session -s turtlebots -d
tmux rename-window -t turtlebots "gazebo"
tmux send-keys -t turtlebots:0 "roslaunch turtlebot3_gazebo turtlebot3_world.launch" C-m
sleep 5
tmux new-window -t turtlebots -n "slam"
tmux send-keys -t turtlebots:1 "roslaunch turtlebot3_slam turtlebot3_gmapping.launch" C-m
tmux new-window -t turtlebots -n "navigation"
tmux send-keys -t turtlebots:2 "roslaunch turtlebot3_navigation turtlebot3_navigation_no_map.launch" C-m
tmux new-window -t turtlebots -n "rqt"
tmux send-keys -t turtlebots:3 "rqt" C-m
tmux attach
