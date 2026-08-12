from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
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

    fer_perception_node = Node(
        package='vgdtas_tasks',
        executable='fer_perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
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

    calibrate_offset_node = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/fer_calibrate_offset', 'std_srvs/srv/Trigger'
        ],
        output='screen'
    )

    deactivate_fer_jtc_node = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'fer_joint_trajectory_controller'
        ],
        output='screen'
    )

    activate_fer_jtc_node = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--activate', 'fer_joint_trajectory_controller'
        ],
        output='screen'
    )

    fer_commander_node = Node(
        package='vgdtas_tasks',
        executable='fer_commander_node',
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
                target_action=ur_pickup_node,
                on_exit=[calibrate_offset_node]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=calibrate_offset_node,
                on_exit=[deactivate_fer_jtc_node]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=deactivate_fer_jtc_node,
                on_exit=[activate_fer_jtc_node]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_fer_jtc_node,
                on_exit=[
                    fer_commander_node,
                    ur_loop_node
                ]
            )
        ),
        fer_perception_node,
        ur_pickup_node
    ])
