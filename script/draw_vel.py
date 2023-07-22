#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

# 初始化 ROS 节点
rospy.init_node('cmd_vel_plotter')

# 初始化线速度和角速度列表
linear_velocity_list = []
angular_velocity_list = []

# 回调函数，处理接收到的 /cmd_vel 消息
def cmd_vel_callback(msg):
    linear_velocity_list.append(msg.linear.x)
    angular_velocity_list.append(msg.angular.z)

    # 绘制折线图
    plt.subplot(2, 1, 1)
    plt.cla()
    plt.plot(linear_velocity_list, 'b', label='Linear Velocity')
    plt.ylabel('Linear Velocity (m/s)')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.cla()
    plt.plot(angular_velocity_list, 'r', label='Angular Velocity')
    plt.xlabel('Time (t)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend()

    plt.pause(0.01)

# 订阅 /cmd_vel 消息
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

# 显示图形界面
plt.figure()
plt.ion()
plt.show()

# 循环等待退出
rospy.spin()

