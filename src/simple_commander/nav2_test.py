#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def main():
    rclpy.init()
    nav = BasicNavigator()

    # -- Set initial Pose --#
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = qx
    initial_pose.pose.orientation.y = qy
    initial_pose.pose.orientation.z = qz
    initial_pose.pose.orientation.w = qw

    nav.setInitialPose(initial_pose)

    # -- Wait for Navigation to Finish --#
    nav.waitUntilNav2Active()

    # -- Set Goal Pose --#
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 3.5
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z = qz
    goal_pose.pose.orientation.w = qw

    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    rclpy.shutdown()

if __name__ == '__main__':
    main()