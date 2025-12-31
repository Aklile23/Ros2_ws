#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client = self.create_client(AddTwoInts, "add_two_ints")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the server to be available...")
        self.get_logger().info("Server is available")
        self.req = AddTwoInts.Request() 
        self.req.a = 3
        self.req.b = 8
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(str(self.future.result().sum))

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.shutdown()    

if __name__ == "__main__":
    main()