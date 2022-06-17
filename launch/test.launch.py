from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tree = PathJoinSubstitution([
        FindPackageShare('behaviour_trees'),
        'resource', 'beacon.xml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('tree', default_value=tree),
        Node(
            package="behaviour_trees",
            executable="behaviour_trees_main",
            namespace=LaunchConfiguration('drone_id'),
            parameters=[{'tree': LaunchConfiguration('tree')}],
            output="screen",
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
    ])
