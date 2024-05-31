from launch import LaunchDescription, actions
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    controller_input_node = Node(
        package="motion_control",
        executable="controller_input",
        name="controller_input"
    )
    spot_controller_node = Node(
        package="motion_control",
        executable="spot_controller",
        name="spot_controller",
        arguments=["--number_of_cycles", "1"],
        on_exit= actions.Shutdown()
    )
    gyroscope_node = Node(
        package="gyroscope",
        executable="gyroscope",
        name="gyroscope",
        arguments=["--number_of_cycles", "1"],
        on_exit= actions.Shutdown()
    )
    ld.add_action(spot_controller_node)
    ld.add_action(gyroscope_node)
    
    # This node needs to be launched separately, as it needs to 
    #ld.add_action(controller_input_node)
    return ld
