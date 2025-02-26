#!/bin/bash

# 等待系统启动稳定
sleep 5

# 创建日志文件
LOG_FILE=/home/hzx/motor/log/start_ros_$(date +"%Y%m%d_%H%M%S").log

mkdir -p $(dirname "$LOG_FILE")

# 清理旧的 ROS 日志（可选）
# rosclean purge -y >> $LOG_FILE 2>&1

# 更新 ROS 环境
source /opt/ros/melodic/setup.bash >> $LOG_FILE 2>&1
source /home/hzx/motor/devel/setup.bash >> $LOG_FILE 2>&1
sleep 1
# 在启动 roscore 之前终止已有的 roscore 实例
# if pgrep -f roscore > /dev/null; then
#     echo "Another roscore is running, killing it..." >> $LOG_FILE
#     killall roscore
#     sleep 1
# fi

# 启动 roscore
echo "Starting roscore..." >> $LOG_FILE
roscore >> $LOG_FILE 2>&1 &
ROSCORE_PID=$!
echo $ROSCORE_PID >> /home/hzx/motor/ros_pids.txt
sleep 1

# 等待 roscore 完全启动
echo "Waiting for roscore to start..." >> $LOG_FILE
until rostopic list > /dev/null 2>&1; do
    sleep 1
done

# 确保 roscore 仍然在运行
# if ! ps -p $ROSCORE_PID > /dev/null; then
#     echo "roscore failed after starting, exiting..." >> $LOG_FILE
#     exit 1
# fi

echo "roscore is up and running." >> $LOG_FILE


# 设置并启动 CAN 接口
# sudo ip link set can0 up type can bitrate 1000000 >> $LOG_FILE 2>&1
# sudo ifconfig can0 up >> $LOG_FILE 2>&1
# sleep 1
# # 启动 CAN 节点
# echo "Starting CAN nodes..." >> $LOG_FILE
# roslaunch pubmotor can_motor.launch >> $LOG_FILE 2>&1 &
# CAN_LAUNCH_PID=$!
# echo $CAN_LAUNCH_PID >> /home/hzx/motor/ros_pids.txt
# sleep 1

# 启动 Joy 控制节点
echo "Starting JOY nodes..." >> $LOG_FILE
roslaunch wheeltec_joy WirelessJoy_crtl_agv.launch >> $LOG_FILE 2>&1 &
JOY_LAUNCH_PID=$!
echo $JOY_LAUNCH_PID >> /home/hzx/motor/ros_pids.txt

sleep 1

# 启动主程序节点
echo "Starting main node..." >> $LOG_FILE
rosrun pubmotor submuti >> $LOG_FILE 2>&1 &
MAIN_NODE_PID=$!
echo $MAIN_NODE_PID >> /home/hzx/motor/ros_pids.txt

echo "All nodes started at $(date)" >> $LOG_FILE

# 等待所有后台进程结束
wait $ROSCORE_PID $CAN_LAUNCH_PID $JOY_LAUNCH_PID $MAIN_NODE_PID
