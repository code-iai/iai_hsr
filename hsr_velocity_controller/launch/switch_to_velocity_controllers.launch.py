import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution, FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    description_file = LaunchConfiguration('description_file', default='hsrb4s.urdf.xacro')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([get_package_share_directory('hsr_description'), 'robots', description_file])
    ])
    robot_description = {'robot_description': robot_description_content}

    controller_yaml_file = os.path.join(
    get_package_share_directory('hsr_velocity_controller'),
    'config', 'my_controller_realtime_test.yaml'
)

    # ros2_control Node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[
            robot_description,
            controller_yaml_file,
            {'use_sim_time': False}
        ],
        output='screen'
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
        arguments=['realtime_body_controller_real', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        #ros2_control_node,
        unspawn_controllers,
        velocity_controller_spawner
    ])
