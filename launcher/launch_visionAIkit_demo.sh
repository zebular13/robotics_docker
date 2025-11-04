#!/bin/bash

# Launch script using expect
BOARD_IP="192.168.1.2"
PASSWORD="oelinux123"

/usr/bin/expect << EOF
spawn ssh -o StrictHostKeyChecking=no root@$BOARD_IP
expect "password:"
send "$PASSWORD\r"
expect "#"
send "export HOME=/home\r"
expect "#"
send "export ROS_DOMAIN_ID=0\r"
expect "#"
send "source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh\r"
expect "#"
send "ros2 pkg executables | grep hand_controller\r"
expect "#"
send "ros2 launch hand_controller demo11_mogiros_car_part1_ei1dials.launch.py verbose:=False model:=/root/hand_controller/hands-v2-yolov5-linux-aarch64-qnn-v36.eim use_flask:=True\r"
expect "#"
interact
EOF