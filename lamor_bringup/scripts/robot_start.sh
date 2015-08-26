#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'cameras'
tmux new-window -t $SESSION:4 -n 'ui'
tmux new-window -t $SESSION:5 -n 'navigation'
tmux new-window -t $SESSION:6 -n 'ppl_perception'
tmux new-window -t $SESSION:7 -n 'scheduler'
tmux new-window -t $SESSION:8 -n 'control'


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch db_path:=$HOME/mongodb_lamor"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_robot.launch with_mux:=false"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_cameras.launch head_camera:=true head_ip:=$HEAD_PC head_user:=lamor chest_camera:=true chest_ip:=$CHEST_PC chest_user:=lamor"

tmux select-window -t $SESSION:4
tmux send-keys "HOST_IP=192.168.0.100 DISPLAY=:0 roslaunch aaf_bringup aaf_ui.launch mary_machine:=$HEAD_PC mary_machine_user:=lamor"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_navigation.launch map:=/opt/strands/map/aaf_winter.yaml topological_map:=aaf_deployment"

tmux select-window -t $SESSION:6
tmux send-keys "DISPLAY=:0 roslaunch perception_people_launch people_tracker_robot.launch machine:=$HEAD_PC user:=lamor"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off