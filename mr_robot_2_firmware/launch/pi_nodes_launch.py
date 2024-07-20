import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, TimerAction

PI_USER = 'atom'
PI_HOST = '192.168.2.102'
PI_PASSWORD = '2811'
PI_WS_DIR = '~/ros2_ws'
LAUNCH_FILE = 'ydlidar_launch.py'

def generate_launch_description():
    def run_on_pi(command):
        # Construct & execute the SSH command using sshpass for password authentication
        ssh_command = f"sshpass -p {PI_PASSWORD} ssh {PI_USER}@{PI_HOST} 'bash -c \"{command}\"'"
        return ExecuteProcess(cmd=['bash', '-c', ssh_command], output='screen')

    return LaunchDescription([
        # Log message to check SSH connection & verify SSH connectivity
        LogInfo(msg="Checking SSH connection to {}...".format(PI_HOST)),
        run_on_pi("echo SSH connection to {} successful.".format(PI_HOST)),
        
        # Change to the ROS 2 workspace directory on the Raspberry Pi
        LogInfo(msg="Changing directory to {} on {}...".format(PI_WS_DIR, PI_HOST)),
        run_on_pi(f"cd {PI_WS_DIR} && pwd"),
        
        # Source the ROS 2 setup script on the Raspberry Pi and execute the launch file
        LogInfo(msg="Sourcing ROS 2 setup script and launching {} on {}...".format(LAUNCH_FILE, PI_HOST)),
        run_on_pi(f"cd {PI_WS_DIR} && source install/setup.bash && ros2 launch ydlidar_ros2_driver {LAUNCH_FILE}"),
        
        # Launching RViz on the host machine with a timer
        LogInfo(msg="Launching RViz on the host machine..."),
        TimerAction(period=6.0, actions=[
            ExecuteProcess(cmd=['rviz2'], output='screen')
        ]),
        
        LogInfo(msg="Launched {} on {} and RViz on the host machine.".format(LAUNCH_FILE, PI_HOST))
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    launch_service = launch.LaunchService()
    launch_service.include_launch_description(ld)
    launch_service.run()
