from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # === Launch Arguments ===
    velocity_controller_arg = DeclareLaunchArgument(
        'velocity_controller',
        default_value='False',
        description='Use ros2_control for velocity-based control (instead of GUI)'
    )

    apartment_map_arg = DeclareLaunchArgument(
        'apartment_map',
        default_value='False',
        description='Load apartment navigation stack'
    )

    description_file = LaunchConfiguration('description_file', default='hsrb4s.urdf.xacro')

    # === Robot Description ===
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([get_package_share_directory('hsr_description'), 'robots', description_file])
    ])
    robot_description = {'robot_description': robot_description_content}

    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory('hsr_description'), 'launch', 'display.rviz'
    ])

    controller_yaml_file = os.path.join(
        get_package_share_directory('hsr_velocity_controller'),
        'config', 'my_controller_realtime_test.yaml'
    )

    # === Nodes ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    )

    joint_state_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=UnlessCondition(LaunchConfiguration('velocity_controller')),
    )

    # ros2_control Node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',  # important for spawner
        parameters=[
            robot_description,
            controller_yaml_file,
            {'use_sim_time': False}
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('velocity_controller')),
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('velocity_controller'))
    )

    # Spawner for velocity controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['realtime_body_controller_real', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Optional apartment map
    apartment_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('hsr_navigation'), 'launch', 'hsr_amcl_map.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('apartment_map'))
    )

    return LaunchDescription([
        velocity_controller_arg,
        apartment_map_arg,
        robot_state_publisher_node,
        joint_state_gui_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner,
        rviz_node,
        apartment_map_launch
    ])
