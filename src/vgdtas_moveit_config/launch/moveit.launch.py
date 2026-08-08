from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    rviz_config = LaunchConfiguration('rviz_config')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='robot', package_name='vgdtas_moveit_config')
        .robot_description_semantic(Path('config') / 'vgdtas_system.srdf')
        .trajectory_execution(Path('config') / 'moveit_controllers.yaml')
        .joint_limits(Path('config') / 'joint_limits.yaml')
        .robot_description_kinematics(Path('config') / 'kinematics.yaml')
        .pilz_cartesian_limits(Path('config') / 'pilz_cartesian_limits.yaml')
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner', 'stomp', 'chomp'],
            default_planning_pipeline='ompl'
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'capabilities': 'move_group/ExecuteTaskSolutionCapability',
                'use_sim_time': True
            }
        ]
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        parameters=[
            moveit_config.to_dict(),
            {
                'moveit_servo': ParameterBuilder('vgdtas_moveit_config')
                .yaml('config/fer_servo.yaml')
                .to_dict(),
                'update_period': 0.01,
                'planning_group_name': 'fer_arm',
                'use_sim_time': True
            },
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('vgdtas_moveit_config'),
                'rviz',
                'moveit.rviz'
            ]),
            description='Rviz config file (absolute path) to use when launching rviz.'
        ),
        move_group_node,
        servo_node,
        rviz_node
    ])
