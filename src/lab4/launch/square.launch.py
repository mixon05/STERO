from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab4',  # Zmień na nazwę swojego pakietu
            executable='square',  # Nazwa wykonywalnego pliku
            name='square_motion_node',
            output='screen',
            parameters=[
                {'side_length': 6.0},  # Długość boku w metrach
                {'repetitions': 2},   # Liczba okrążeń
                {'clockwise': True}   # Ruch zgodny z ruchem wskazówek zegara
            ]
        )
    ])
