import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    dof6arm_share = get_package_share_directory('dof6arm')
    arm_bringup_share = get_package_share_directory('arm_bringup')

    # Resource path for Ignition meshes
    resource_path = os.path.dirname(dof6arm_share)
    set_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path
    )

    # xacro → URDF
    xacro_file = os.path.join(dof6arm_share, 'urdf', 'dof6arm_gazebo.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 1. Gazebo (running)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 3. Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'dof6arm',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # 4. Bridge clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # 5/6. Spawners (sequenced)
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    spawn_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    delay_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[spawn_jsb])
    )
    delay_jtc = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_jtc])
    )

    # 7. IK node in trajectory mode (publishes JointTrajectory)
    ik_node = Node(
        package='arm_kinematics',
        executable='ik_node',
        parameters=[{'use_trajectory_controller': True}],
        output='screen'
    )

    # 8. FK node (subscribes /joint_states from joint_state_broadcaster)
    fk_node = Node(
        package='arm_kinematics',
        executable='fk_node',
        output='screen'
    )

    # 9. Home pose publisher (publishes /target_pose once)
    home_node = Node(
        package='arm_kinematics',
        executable='home_pose_node',
        output='screen'
    )

    # 10. rosbridge for the web UI
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('rosbridge_server').find('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    # 11. Open interface.html in browser (1 s after launch)
    interface_path = os.path.join(arm_bringup_share, 'web', 'interface.html')
    open_interface = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['xdg-open', interface_path],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        delay_jsb,
        delay_jtc,
        ik_node,
        fk_node,
        home_node,
        rosbridge,
        open_interface,
    ])