#!/bin/bash

byobu new-session -d -s pp
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0
byobu split-window -h
byobu select-pane -t 2
byobu split-window -h
byobu select-pane -t 3

byobu send-keys -t 0 'xhost + && docker exec -it iiwa_container bash -it -c "roslaunch iiwa_controller iiwa_commander.launch sim:=true"' 'C-m'
sleep 5.
byobu send-keys -t 1 'xhost + && docker exec -it iiwa_container bash -it -c "rosrun iiwa_controller iiwa_moveit_commander.py"' 'C-m'
sleep 5.
byobu send-keys -t 2 'xhost + && docker exec -it iiwa_container bash -it -c "python3 /catkin_ws/src/iiwa_tutorials/scripts/dummy_socket_client.py"' 'C-m'

byobu attach -t pp