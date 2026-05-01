from launch import LaunchDescription
from launch_ros.actions import Node

# For the right_index_multiplier use:
#  - 0.75 for simulation
#  - 0.25 for real robot

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service',
            name='direction_service',
            output='screen',                                       
        ),
    ])
