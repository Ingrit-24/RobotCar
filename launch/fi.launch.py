from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rc_controll',
            executable='Kalman_Node',
            name='KAL',
            output='screen'
        ),
        
        Node(
            package='rc_controll',
            executable='Ecef2Enu_Node',
            name='ECEF2ENU',
            output='screen'
        )
    ])