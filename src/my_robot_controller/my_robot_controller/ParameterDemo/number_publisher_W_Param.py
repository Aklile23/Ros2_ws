#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")

        self.declare_parameter("number", 2.0)
        self.declare_parameter("timer_period", 0.5)

        self.number_ = self.get_parameter("number").value
        self.timer_period = self.get_parameter("timer_period").value

        self.number_publisher = self.create_publisher(Float64, "number", 10)
        self.timer_ = self.create_timer(self.timer_period, self.publish_number)
        self.get_logger().info("Number publisher has been started")

    def publish_number(self):
        msg = Float64()
        msg.data = self.get_parameter("number").value
        self.number_publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()