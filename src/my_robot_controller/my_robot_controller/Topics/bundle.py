#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class BundleNode(Node):
    def __init__(self):
        super().__init__('bundle')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Bundle node started")

    def pose_callback(self, msg:Pose):
        self.get_logger().info(str(msg))

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BundleNode()
    rclpy.spin(node)
    rclpy.shutdown()