#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n '2d_nav'
tmux new-window -t $SESSION:4 -n 'topo_nav'
tmux new-window -t $SESSION:5 -n 'ppl_emulator'
tmux new-window -t $SESSION:6 -n 'qsr_lib'


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
tmux send-keys "DISPLAY=:0 roslaunch strands_morse uol_bl_morse.launch env:=uol_bl_human_fast"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch lamor_bringup uol_bl_nav2d.launch map:=\$(rospack find strands_morse)/uol/maps/uol_bl.yaml"

tmux select-window -t $SESSION:4
tmux send-keys "DISPLAY=:0 roslaunch lamor_bringup lamor_sim_navigation.launch dataset:=bl_sim"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:0 rosrun people_tracker_emulator posestamped_to_ppl_tracker_msgs.py"

tmux select-window -t $SESSION:6
tmux send-keys "DISPLAY=:0 rosrun qsr_lib qsrlib_ros_server.py"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
