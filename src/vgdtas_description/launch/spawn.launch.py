from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):

    world = LaunchConfiguration('world')

    gz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ])),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world]
        }.items(),
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/fer_camera_link/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/fer_camera_link/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen',
    )

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

    entity_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',
            'vgdtas',
            '-topic',
            'robot_description',
            '-allow_renaming',
            'true'
        ],
    )

    jsb_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager']
    )

    ur_jtc_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_joint_trajectory_controller', '-c', '/controller_manager']
    )

    ur_gc_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_gripper_controller', '-c', '/controller_manager']
    )

    fer_jtc_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fer_joint_trajectory_controller', '-c', '/controller_manager']
    )

    fer_fvc_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fer_forward_velocity_controller', '-c', '/controller_manager', '--inactive']
    )

    return [
        gz_node,
        ros_gz_bridge_node,
        rsp_node,
        entity_spawner_node,
        jsb_spawner_node,
        ur_jtc_spawner_node,
        ur_gc_spawner_node,
        fer_jtc_spawner_node,
        fer_fvc_spawner_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('vgdtas_description'),
                'worlds',
                'empty_world.sdf'
            ]),
            description='Gazebo world file containing a custom world.',
        ),
        OpaqueFunction(function=launch_setup)
    ])
