from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= '',
                package= 'DRobot_package',
                executable= 'drobot_module',
                output= 'screen'
            ),
            Node(
                namespace= '',
                package= 'DRobot_package',
                executable= 'drobot_motor',
                output= 'screen'
            )
        ]
    )