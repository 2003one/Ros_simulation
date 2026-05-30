import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    pkg       = get_package_share_directory('spider_bot_description')
    urdf_path = os.path.join(pkg, 'urdf', 'spider_bot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Publishes TF for every link from joint states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Walking gait node — publishes 12 joint angles at 50 Hz
        Node(
            package='spider_bot_description',
            executable='walk_forward',
            name='walk_forward',
            output='screen',
        ),

        # RViz to watch the motion
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
