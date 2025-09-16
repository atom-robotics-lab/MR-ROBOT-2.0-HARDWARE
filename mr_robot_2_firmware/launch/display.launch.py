import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mr_robot_2_firmware')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',  
            description='Flag to enable use_sim_time'),
        robot_localization_node,
    ])
