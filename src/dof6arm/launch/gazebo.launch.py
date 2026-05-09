import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    urdf_file = PathJoinSubstitution([
        FindPackageShare('dof6arm'),
        'urdf',
        'dof6arm_gazebo.urdf'
    ])

    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

   
        # 1. Start Gazebo with an empty world (paused — physics frozen)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'pause': 'true'}.items()
    )

    # 2. Publish /robot_description and /tf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 3. Spawn the arm into Gazebo from the URDF file directly
    #    (avoids spawn_entity.py's XML-encoding-declaration bug)
    urdf_path = os.path.join(
        get_package_share_directory('dof6arm'),
        'urdf',
        'dof6arm_gazebo.urdf'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_path,
            '-entity', 'dof6arm',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])