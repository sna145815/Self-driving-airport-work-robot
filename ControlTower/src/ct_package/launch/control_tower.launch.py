from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace= '',
                package= 'ct_package',
                executable= 'taskManager',
                output= 'screen'
            ),
                        Node(
                namespace= '',
                package= 'ct_package',
                executable= 'robot_manager',
                output= 'screen'
            ),            Node(
                namespace= '',
                package= 'ct_package',
                executable= 'store_manager',
                output= 'screen'
            ),            Node(
                namespace= '',
                package= 'ct_package',
                executable= 'kiosk_manager',
                output= 'screen'
            )
        ]
    )