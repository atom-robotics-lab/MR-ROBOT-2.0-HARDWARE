from launch import LaunchDescription
from launch_ros.actions import Node

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
    ])
