from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    controller_input_node = Node(
        package="spot_py",
        executable="controller_input",
        name="controller_input"
    )
    spot_controller_node = Node(
        package="spot_py",
        executable="spot_controller_node",
        name="spot_controller_node"
    )
    
    ld.add_action(controller_input_node)
    ld.add_action(spot_controller_node)
    return ld
