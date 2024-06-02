from launch import LaunchDescription, actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('mpu6050driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'mpu6050.yaml'),
                                           description='Path to the ROS2 parameters file to use.')

    mpu6050driver_node = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )
    servo_server_node = Node(
        package="servo_driver",
        executable="pca9685_server",
        name="pca9685_server"
    )
    spot_controller_node = Node(
        package="motion_control",
        executable="spot_controller",
        name="spot_controller",
        arguments=["--number_of_cycles", "1"],
        on_exit= actions.Shutdown()
    )

    ld.add_action(params_declare)
    ld.add_action(mpu6050driver_node)
    #ld.add_action(servo_server_node)
    ld.add_action(spot_controller_node)

    
    # This node needs to be launched separately, as it needs to 
    #ld.add_action(controller_input_node)
    return ld
