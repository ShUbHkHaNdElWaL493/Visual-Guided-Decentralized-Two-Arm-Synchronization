from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vgdtas_description'),
            'launch',
            'spawn.launch.py'
        ]))
    )

    moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vgdtas_moveit_config'),
            'launch',
            'moveit.launch.py'
        ]))
    )

    return LaunchDescription([
        spawn_node,
        moveit_node
    ])
