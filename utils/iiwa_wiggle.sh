#!/bin/bash

byobu new-session -d -s wiggle
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0
byobu split-window -h
byobu select-pane -t 2
byobu split-window -h
byobu select-pane -t 3

byobu send-keys -t 0 'xhost + && docker exec -it iiwa_container bash -it -c "roscore"' 'C-m'
sleep 5.
byobu send-keys -t 1 'xhost + && docker exec -it iiwa_container bash -it -c "rosrun iiwa_tutorials wiggle"' 'C-m'

byobu attach -t wiggle