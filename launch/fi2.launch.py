from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rc_controll',
            executable='Kalman2_Node',
            name='KAL2',
            output='screen'
        ),
        
        Node(
            package='rc_controll',
            executable='Ecef2Enu_Node',
            name='ECEF2ENU',
            output='screen'
        )
    ])