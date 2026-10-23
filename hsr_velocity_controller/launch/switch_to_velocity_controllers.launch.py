import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Use controller-specific parameter file
    controller_params = os.path.join(
        get_package_share_directory('hsr_velocity_controller'),
        'config', 'realtime_body_controller.yaml'
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

    # Spawner for velocity controller with parameter file
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'realtime_body_controller_real',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_params,
            '--controller-type', 'hsr_velocity_controller_ns/HsrVelocityController',
        ],
        output='screen'
    )

    return LaunchDescription([
        unspawn_controllers,
        velocity_controller_spawner
    ])
