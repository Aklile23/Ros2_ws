from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    param_config = os.path.join(get_package_share_directory("my_robot_bringup"), "config", "number_app.yaml")

    my_first_node = Node(
        package="my_robot_controller",
        executable="test_node",
    )
    number_publisher_W_Param = Node(
        package="my_robot_controller",
        executable="number_publisher_W_Param",
        parameters=[param_config]
    )

    ld.add_action(my_first_node)
    ld.add_action(number_publisher_W_Param)

    return ld