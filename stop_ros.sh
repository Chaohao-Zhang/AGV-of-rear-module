#!/bin/bash

# 设置日志文件
LOG_FILE=/home/hzx/motor/log/stop_ros_$(date +"%Y%m%d_%H%M%S").log
echo "Stopping ROS nodes at $(date)" >> $LOG_FILE

# 读取并终止所有进程
if [ -f /home/hzx/motor/ros_pids.txt ]; then
    while IFS= read -r pid
    do
        # 检查进程是否存在
        if ps -p $pid > /dev/null 2>&1; then
            # 获取进程所有者
            pid_user=$(ps -o user= -p $pid)
            # 获取当前用户
            current_user=$(whoami)

            # 如果进程所有者是当前用户，直接终止
            if [ "$pid_user" == "$current_user" ]; then
                echo "Killing process $pid owned by $current_user" | tee -a $LOG_FILE
                kill $pid
            else
                # 需要使用 sudo 终止其他用户的进程
                echo "Killing process $pid owned by $pid_user using sudo" | tee -a $LOG_FILE
                sudo kill $pid
            fi

            # 检查进程是否被成功终止，如果没有则强制终止
            sleep 1
            if ps -p $pid > /dev/null 2>&1; then
                echo "Process $pid did not terminate, forcing kill..." | tee -a $LOG_FILE
                if [ "$pid_user" == "$current_user" ]; then
                    kill -9 $pid
                else
                    sudo kill -9 $pid
                fi
            else
                echo "Process $pid terminated successfully." | tee -a $LOG_FILE
            fi
        else
            echo "Process $pid is not running or already terminated." | tee -a $LOG_FILE
        fi
    done < /home/hzx/motor/ros_pids.txt

    # 删除PID文件
    rm -f /home/hzx/motor/ros_pids.txt
    echo "PID file removed." >> $LOG_FILE
else
    echo "PID file not found" | tee -a $LOG_FILE
fi

echo "All processes stopped at $(date)" >> $LOG_FILE