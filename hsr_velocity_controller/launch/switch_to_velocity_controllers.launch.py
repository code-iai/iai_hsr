import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution, FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    controller_yaml_file = os.path.join(
    get_package_share_directory('hsr_velocity_controller'),
    'config', 'my_controller_realtime_test.yaml'
)
    # Unspawn existing trajectory controllers
    unspawn_controllers = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            'arm_trajectory_controller',
            'head_trajectory_controller',
            '-c', '/controller_manager'
        ],
        output='screen',
    )

    # Spawner for velocity controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
        'realtime_body_controller_real',
        '--controller-manager', '/controller_manager',
        '--ros-args', '--params-file', controller_yaml_file]
        output='screen'
    )

    return LaunchDescription([
        unspawn_controllers,
        velocity_controller_spawner
    ])
