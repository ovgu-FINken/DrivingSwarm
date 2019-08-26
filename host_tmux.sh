#!/bin/bash
MAPPING=${MAPPING:=gmapping}
NUM=0
tmux new-session -s turtlebots -d
tmux rename-window -t turtlebots "host"
tmux send-keys -t turtlebots:$NUM "htop" C-m
NUM=$((++NUM))
read -t 3
tmux new-window -t turtlebots -n "mapping 0"
tmux send-keys -t turtlebots:$NUM "roslaunch tracking_certainty_grid $MAPPING.launch robot_name:=turtlebot1" C-m
NUM=$((++NUM))
tmux new-window -t turtlebots -n "rqt"
tmux send-keys -t turtlebots:$NUM "rqt" C-m
NUM=$((++NUM))

tmux attach
