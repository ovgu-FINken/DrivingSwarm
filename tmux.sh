#!/bin/bash
MAPPING=${MAPPING:=amcl}
NUM=0
tmux new-session -s turtlebots -d
tmux rename-window -t turtlebots "gazebo"
tmux send-keys -t turtlebots:$NUM "roslaunch tracking_certainty_grid simulation.launch world:=turtlebot3_world.world" C-m
NUM=$((++NUM))
read -t 3
tmux new-window -t turtlebots -n "mapping 0"
tmux send-keys -t turtlebots:$NUM "roslaunch tracking_certainty_grid $MAPPING.launch robot_name:=tb3_0" C-m
NUM=$((++NUM))
tmux new-window -t turtlebots -n "mapping 1"
tmux send-keys -t turtlebots:$NUM "roslaunch tracking_certainty_grid $MAPPING.launch robot_name:=tb3_1" C-m
NUM=$((++NUM))
tmux new-window -t turtlebots -n "mapping 2"
tmux send-keys -t turtlebots:$NUM "roslaunch tracking_certainty_grid $MAPPING.launch robot_name:=tb3_2" C-m
NUM=$((++NUM))
read -t 3
tmux new-window -t turtlebots -n "navigation"
tmux send-keys -t turtlebots:$NUM "roslaunch tracking_certainty_grid tracking_simulation.launch" C-m
NUM=$((++NUM))
tmux new-window -t turtlebots -n "rqt"
tmux send-keys -t turtlebots:$NUM "rqt" C-m
NUM=$((++NUM))
tmux new-window -t turtlebots -n "rviz"
tmux send-keys -t turtlebots:$NUM "ROS_NAMESPACE=tb3_0 rviz" C-m
NUM=$((++NUM))
tmux new-window -t turtlebots -n "certainty_grid"
tmux send-keys -t turtlebots:$NUM "ROS_NAMESPACE=tb3_0 roslaunch tracking_certainty_grid certainty_grid.launch" C-m
tmux attach
