#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n '2d_nav'
tmux new-window -t $SESSION:4 -n 'navigation'


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
tmux send-keys "DISPLAY=:0 roslaunch strands_morse uol_bl_morse.launch"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch strands_morse uol_bl_nav2d.launch"

tmux select-window -t $SESSION:4
tmux send-keys "DISPLAY=:0 roslaunch strands_morse uol_bl_nav2d.launch"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off