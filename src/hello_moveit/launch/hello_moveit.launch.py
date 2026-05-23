from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    arg1 = DeclareLaunchArgument('use_sim_time', default_value='true')
    return LaunchDescription([
        arg1,
        Node(
            package='hello_moveit',
            executable='hello_moveit',
            parameters=[{'use_sim_time': True}]
        )
    ])