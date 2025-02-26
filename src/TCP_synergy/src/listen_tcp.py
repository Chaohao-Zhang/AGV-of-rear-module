#!/usr/bin/env python3
#coding=utf-8
import rospy
import socket
import struct
from geometry_msgs.msg import Twist

def tcp_receiver():
    # 初始化 ROS 节点
    rospy.init_node('tcp_twist_receiver', anonymous=True)

    # 创建 TCP 服务器，监听外部设备的连接
    host = '0.0.0.0'  # 监听所有网络接口
    port = 5000        # 假设外部设备发送消息到端口 5000
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    
    rospy.loginfo(f"Waiting for connection on {host}:{port}...")
    
    # 等待外部设备连接
    client_socket, client_address = server_socket.accept()
    rospy.loginfo(f"Connection established with {client_address}")
    
    # 创建 Twist 消息的发布者
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        try:
            # 接收数据（这里假设每次接收到的数据长度为 24 字节：包含线速度和角速度）
            data = client_socket.recv(12)
            # print("Received data:", data)

            # 如果收到的数据长度不为 24 字节，跳过当前循环
            if len(data) != 12:
                continue

            # 解析数据：数据结构假设是 3 个浮动值，分别是 linear.x, linear.y, angular.z
            linear_x, angular_z,angular_R = struct.unpack('fff', data)
            
            # 创建并填充 geometry_msgs/Twist 消息
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            twist_msg.angular.x = angular_R
            rospy.loginfo(f"Received data - linear.x: {twist_msg.linear.x}, angular.z: {twist_msg.angular.z}, turning radius:{twist_msg.angular.x}")
            
            
            # 发布消息到 ROS
            twist_pub.publish(twist_msg)

        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            break
        except Exception as e:
            rospy.logerr(f"Exception occurred: {e}")
            break
    
    # 关闭连接
    client_socket.close()
    server_socket.close()

if __name__ == '__main__':
    try:
        tcp_receiver()
    except rospy.ROSInterruptException:
        pass
