import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg              = get_package_share_directory('spider_bot_description')
    urdf_path        = os.path.join(pkg, 'urdf', 'spider_bot.urdf')
    controllers_path = os.path.join(pkg, 'config', 'controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read().replace(
            '$(find spider_bot_description)/config/controllers.yaml',
            controllers_path
        )

    # Inject plugin path so Gazebo finds gz_ros2_control
    gz_env = os.environ.copy()
    ros_lib = '/opt/ros/jazzy/lib'
    existing = gz_env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gz_env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = \
        ros_lib + (':' + existing if existing else '')

    return LaunchDescription([

        # 1. Gazebo with corrected env
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf', '-v', '4'],  # -v 4 = verbose
            output='screen',
            additional_env=gz_env,
        ),

        # 2. Robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 3. Spawn robot
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name',  'spider_bot',
                    '-topic', 'robot_description',
                    '-z',     '0.25',
                ],
                output='screen',
            ),
        ]),

        # 4. Load controllers
        TimerAction(period=7.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen',
            ),
        ]),

        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active', 'joint_group_position_controller'],
                output='screen',
            ),
        ]),

        # 5. Walk
        TimerAction(period=9.0, actions=[
            Node(
                package='spider_bot_description',
                executable='walk_forward',
                name='walk_forward',
                output='screen',
            ),
        ]),
    ])
