from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

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

    switch_servo_command_type_node = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/servo_node/switch_command_type',
            'moveit_msgs/srv/ServoCommandType',
            '{command_type: 2}'
        ],
        output='screen'
    )

    ur_pickup_node = Node(
        package='vgdtas_tasks',
        executable='ur_pickup_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ]
    )

    fer_perception_node = Node(
        package='vgdtas_tasks',
        executable='fer_perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ur_loop_node = Node(
        package='vgdtas_tasks',
        executable='ur_loop_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        RegisterEventHandler(
            OnProcessExit(
                target_action=switch_servo_command_type_node,
                on_exit=[ur_pickup_node]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=ur_pickup_node,
                on_exit=[
                    fer_perception_node,
                    TimerAction(
                        period=1.0,
                        actions=[ur_loop_node]
                    )
                ]
            )
        ),
        switch_servo_command_type_node
    ])
