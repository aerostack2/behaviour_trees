from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', description="Drone namespace", 
                              default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('tree', description="Path to XML Behaviour Tree"),
        DeclareLaunchArgument('groot_logger', description="Want to use groot logger?",
                              choices={"true", "false"}, default_value='false'),
        DeclareLaunchArgument('groot_client_port', description="Groot publisher port", default_value='1666'),
        DeclareLaunchArgument('groot_server_port', description="Groot server port", default_value='1667'),
        
        Node(
            package="behaviour_trees",
            executable="behaviour_trees_main",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'tree': LaunchConfiguration('tree'),
                         'use_groot': LaunchConfiguration('groot_logger'),
                         'groot_client_port': LaunchConfiguration('groot_client_port'),
                         'groot_server_port': LaunchConfiguration('groot_server_port')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
