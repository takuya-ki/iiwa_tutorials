#!/bin/bash

byobu new-session -d -s moveit
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 1

byobu send-keys -t 0 'xhost + && docker exec -it iiwa_container bash -it -c "roslaunch iiwa_tool_moveit moveit_planning_execution.launch sim:=false"' 'C-m'

byobu attach -t moveit