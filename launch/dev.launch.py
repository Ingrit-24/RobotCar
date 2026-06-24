import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    external_pkg_share = get_package_share_directory('gnss_ros_standardization')
    yaml_file_path = os.path.join(external_pkg_share, 'config', 'single_point_positioning.yaml')

    user_name_arg = DeclareLaunchArgument(
        'user_name',
        description='user'
    )
    password_arg = DeclareLaunchArgument(
        'password',
        description='Password for the streaming service'
    )
    
    ntrip = PythonExpression([
        "'ntrip://' + '", LaunchConfiguration('user_name'), "' + ':' + '", LaunchConfiguration('password'), "' + '@citgnss.taroz.net:2101/RTCM3'"
    ])
    return LaunchDescription([
        user_name_arg,
        password_arg,
        LogInfo(msg=['Generated NTRIP URL: ', ntrip]),
        Node(
            package='gnss_ros_standardization',
            executable='single_point_positioning',
            parameters=[yaml_file_path]
        ),
        Node(
            package='gnss_ros_standardization',    
            executable='rtcm_decoder_node', 
            parameters=[{
                'stream_path': ntrip
            }]
        ),
    ])