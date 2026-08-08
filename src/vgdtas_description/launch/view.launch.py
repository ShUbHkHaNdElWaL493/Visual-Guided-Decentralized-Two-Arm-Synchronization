from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):

    rviz_config = LaunchConfiguration('rviz_config')

    with open(PathJoinSubstitution([
        FindPackageShare('vgdtas_description'),
        'models',
        'robot.urdf'
    ]).perform(context), 'r') as infp:
        robot_description = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(robot_description, value_type=str)
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return [
        rsp_node,
        rviz_node,
        joint_state_publisher_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('vgdtas_description'),
                'rviz',
                'view.rviz'
            ]),
            description='Rviz config file (absolute path) to use when launching rviz.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
