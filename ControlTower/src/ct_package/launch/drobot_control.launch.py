from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= '',
                package= 'ct_package',
                executable= 'drobot_control',
                output= 'screen'
            )
        ]
    )