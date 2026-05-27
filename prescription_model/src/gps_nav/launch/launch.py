from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare('gps_nav'), 'urdf', 'base.urdf.xacro'
    ])

    # Toggle the mesh with mesh_enabled:=true/false
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' mesh_enabled:=true']),
        value_type=str
    )
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(package='gps_nav', executable='line_nav', name='line_nav'),
    ])
