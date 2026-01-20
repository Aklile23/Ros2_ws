#! /usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from my_robot_interfaces.action import CountUntil
from rclpy.action import ActionServer

class CountUntilServer(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.action_server = ActionServer(self, CountUntil, "count_until", self.execute_callback)
        self.get_logger().info("Action server started")

    def execute_callback(self, goal_handle):
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action
        self.get_logger().info("Executing goal")
        counter = 0
        for i in range(target_number):

            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                goal_handle.canceled()

                # and send the result
                result = CountUntil.Result()
                result.reached_number = counter
                return result

            counter +=1
            self.get_logger().info(str(counter))
            
            # Publish feedback
            feedback = CountUntil.Feedback()
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)

            time.sleep(period)

        # Once done, set goal final state
        goal_handle.succeed()

        # and send the result
        result = CountUntil.Result()
        result.reached_number = counter
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()