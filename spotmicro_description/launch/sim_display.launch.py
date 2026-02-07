import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('spotmicro_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'spotmicro_sim.rviz')

    robot_description = Command(['xacro ', urdf_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    spot_controller = Node(
        package='motion_control',
        executable='spot_controller',
        parameters=[{'simulation_mode': True}],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        spot_controller,
        rviz2,
    ])
