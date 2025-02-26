#!/bin/bash
# 更新 ROS 环境

# LOG_FILE=/home/hzx/motor/log/can_start_$(date +"%Y%m%d_%H%M%S").log
# mkdir -p $(dirname "$LOG_FILE")

# source /opt/ros/melodic/setup.bash >> $LOG_FILE 2>&1
# source /home/hzx/motor/devel/setup.bash >> $LOG_FILE 2>&1
# sleep 1
# # 设置并启动 CAN 接口
# sudo ip link set can0 up type can bitrate 1000000 >> $LOG_FILE 2>&1
# sudo ifconfig can0 up >> $LOG_FILE 2>&1
# sleep 1
# # 启动 CAN 节点
# echo "Starting CAN nodes..." >> $LOG_FILE
# roslaunch pubmotor can_motor.launch >> $LOG_FILE 2>&1 &
# CAN_LAUNCH_PID=$!
# echo $CAN_LAUNCH_PID >> /home/hzx/motor/ros_pids.txt
# sleep 1

source /opt/ros/melodic/setup.bash
source /home/hzx/motor/devel/setup.bash

sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up
sleep 1
roslaunch pubmotor can_motor.launch