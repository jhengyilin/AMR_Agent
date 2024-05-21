#need to rewrite to fit ROS commnad (rclpy, PoseStamped, etc.)


#!/usr/bin/env python
# coding=utf-8

# Import necessary modules for ROS 2 communication
import os  # Operating system module
import select  # I/O multiplexing module
import sys  # System-specific parameters and functions module
# import rclpy  # ROS 2 client library

# Import message types and Quality of Service (QoS) profile for ROS 2
# from geometry_msgs.msg import PoseStamped
# from rclpy.qos import QoSProfile  # Quality of Service profile for ROS 2

# Import modules for controlling terminal I/O settings
import tty  # Terminal control module

from amr_def import *

class MessageHandler:

    def __init__(self, amr_control: AMRControl):
        self.__amr_control = amr_control
        # rclpy.init()
        # qos = QoSProfile(depth=10)
        # self.node = rclpy.create_node('AMR_path')
        # self.pub = self.node.create_publisher(PoseStamped, '/goal_pose', qos)
        # try:
        #     goal_msg=PoseStamped()
        #     goal_msg.header.frame_id='map'
        #     goal_msg.pose.position.x=0.0
        #     goal_msg.pose.position.y=0.0
        #     goal_msg.pose.position.z=0.0
        #     goal_msg.pose.orientation.x=0.0
        #     goal_msg.pose.orientation.y=0.0
        #     goal_msg.pose.orientation.z=0.0
        #     goal_msg.pose.orientation.w=1.0
        #     self.pub.publish(goal_msg)

        # except Exception as e:
        #     print('error!!')
    
    def run(self):
        # rclpy.spin(self.node)
        print("message handler is running")
        print("location name setting:")
        print(self.__amr_control.smart_assist_location_name_setting.get())
        print("pose setting:")
        print(self.__amr_control.smart_assist_pose_setting.get())

    __amr_control = None