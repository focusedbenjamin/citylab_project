from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_file = os.path.join(get_package_share_directory('robot_patrol'), 'rviz', 'tb3_default.rviz')

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='go_to_pose_action',
            output='screen',
            emulate_tty=True),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        ),
    ])
