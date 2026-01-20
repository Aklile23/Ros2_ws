#! /usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from my_robot_interfaces.action import CountUntil
from rclpy.action import ActionClient

class CountUntilClient(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")

        self.goal_handle_ = None
        
    def send_goal(self, target_number, period):
        # wait for the server 
        self.count_until_client_.wait_for_server()

        # Create a goal 
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal
        self.get_logger().info("Sending goal")
        self.count_until_client_.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        ).add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            "Feedback: current_number = " + str(feedback.current_number)
    )

    def goal_response_cb(self, future):
        self.goal_handle_ = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.get_result_cb)

    def get_result_cb(self, future):
        result = future.result().result
        self.get_logger().info("Result: "  + str(result.reached_number))

    def cancel_goal(self):
        if self.goal_handle_:
            self.get_logger().info("Requesting goal cancellation")
            self.goal_handle_.cancel_goal_async()

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient()
    node.send_goal(10, 1.0)

    time.sleep(3.0)
    node.cancel_goal()

    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
        