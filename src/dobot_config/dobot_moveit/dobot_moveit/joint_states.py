#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
@author FTX
@date 2025 / 03 / 03
'''
from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# import numpy as np

class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)
        self.sub_arm = self.create_subscription( JointState, "/joint_states_robot", self.listener_callback_arm, 10)
        
        self.sub_gripper = self.create_subscription(Float64, "/gripper/status", self.listener_callback_gripper, 10)
        self.gripper_status = 0.0

        self.pub2 = self.create_publisher(JointState, "joint_states", 10)

    def listener_callback_gripper(self, msg):
        self.gripper_status = msg.data
    def listener_callback_arm(self, msg):
        joint = [msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],msg.position[5],0.026 - self.gripper_status]
        print(joint)
        msg2 = JointState()
        msg2.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6","gripper_joint_1"]
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.header.frame_id = 'joint_states'
        msg2.position = joint
        self.pub2.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode("dobot_joint_states")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
