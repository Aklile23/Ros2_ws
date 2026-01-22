from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    managed_node_name = "number_publisher_lifecycle"
    
    my_number_publisher = Node(
        package="lifecycle_py",
        executable="number_publisher_lifecycle",
        name=managed_node_name
    )

    lifecycle_node_manager = Node(
        package="lifecycle_py",
        executable="lifecycle_node_manager",
        parameters=[{"managed_node_name": managed_node_name}]
    )

    ld.add_action(my_number_publisher)
    ld.add_action(lifecycle_node_manager)

    return ld