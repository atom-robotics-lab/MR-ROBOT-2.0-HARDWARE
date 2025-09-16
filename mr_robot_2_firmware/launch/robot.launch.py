from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('mr_robot_2_firmware')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mr_robot_2_firmware',   # your package name
            executable='twist_to_pwm.py',           # script filename
            name='twist_to_pwm',                    # ROS 2 node name
            output='screen'                  # log to console
        ),
        Node(
            package='mr_robot_2_firmware',
            executable='diff_tf.py',
            name='diff_tf',
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
    )
    ])
    
