#!/bin/bash
#starting ros1 and ros2 bridge

#sudo ifconfig eno2 192.168.1.125
# sleep 1
# usr/bin/tmux -2 new-session -d -s bridge
# usr/bin/tmux send-keys -t bridge.0 "source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge dynamic_bridge" ENTER
# sleep 5


#starting controllers and bringup
sleep 1
/usr/bin/tmux -2 new-session -d -s bringup
#usr/bin/tmux send-keys -t bringup.0 "source /opt/ros/foxy/setup.bash && source /home/amr1000/Music/Onix_ros2/install/setup.bash && ros2 launch onix_base base_launch.py" ENTER
/usr/bin/tmux send-keys -t bringup.0 "source /opt/ros/foxy/setup.bash && source /home/xcy/sona_ws/install/setup.bash && ros2 launch onix_robot base_launch.py" ENTER
sleep 5


/usr/bin/tmux -2 new-session -d -s camera
/usr/bin/tmux send-keys -t camera.0 "source /opt/ros/foxy/setup.bash && source /home/xcy/sona_ws/install/setup.bash && ros2 launch usb_camera_driver usb_camera_node.launch.py" ENTER
sleep 5

#usr/bin/tmux -2 new-session -d -s rosserial
#usr/bin/tmux send-keys -t rosserial.0 "source /opt/ros/foxy/setup.bash && source /home/amr1000/Music/Onix_ros2/install/setup.bash && rosrun rosserial_python serial_node.py _port:=/dev/#usb_arduino_mega" 
#ENTER

/usr/bin/tmux -2 new-session -d -s modbus
/usr/bin/tmux send-keys -t modbus.0 "source /opt/ros/foxy/setup.bash && source /home/xcy/sona_ws/install/setup.bash && ros2 run modbus_driver modbus_driver" ENTER
sleep 5

/usr/bin/tmux -2 new-session -d -s rfid
/usr/bin/tmux send-keys -t rfid.0 "source /opt/ros/foxy/setup.bash && source /home/xcy/sona_ws/install/setup.bash && ros2 run modbus_driver rfid_modbus_driver" ENTER
sleep 5

/usr/bin/tmux -2 new-session -d -s follower
/usr/bin/tmux send-keys -t follower.0 "source /opt/ros/foxy/setup.bash && source /home/xcy/sona_ws/install/setup.bash && ros2 run follower follower_node" ENTER
sleep 5

/usr/bin/tmux -2 new-session -d -s statecontrol
/usr/bin/tmux send-keys -t statecontrol.0 "source /opt/ros/foxy/setup.bash && source /home/xcy/sona_ws/install/setup.bash && ros2 run state_control state_node" ENTER
sleep 5

/usr/bin/tmux -2 new-session -d -s joystick
/usr/bin/tmux send-keys -t joystick.0 "source /opt/ros/foxy/setup.bash && source /home/xcy/sona_ws/install/setup.bash && ros2 launch agv_joy_control teleop_joy.launch.py " ENTER
sleep 5

