#!/bin/bash




#closing controllers and bringup
/usr/bin/tmux send-keys -t bringup.0 "" C-c
/usr/bin/tmux send-keys -t camera.0 "" C-c
/usr/bin/tmux send-keys -t modbus.0 "" C-c
/usr/bin/tmux send-keys -t rfid.0 "" C-c
/usr/bin/tmux send-keys -t follower.0 "" C-c
/usr/bin/tmux send-keys -t statecontrol.0 "" C-c
/usr/bin/tmux send-keys -t joystick.0 "" C-c
#/usr/bin/tmux send-keys -t rosserial.0 "" C-c
sleep 7

#killing all tmux servers
/usr/bin/tmux kill-session -t bringup

/usr/bin/tmux kill-session -t camera

/usr/bin/tmux kill-session -t modbus

/usr/bin/tmux kill-session -t rfid

/usr/bin/tmux kill-session -t follower

/usr/bin/tmux kill-session -t statecontrol

/usr/bin/tmux kill-session -t joystick