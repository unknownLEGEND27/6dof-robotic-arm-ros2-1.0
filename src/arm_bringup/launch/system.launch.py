from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    dof6arm_path = get_package_share_directory('dof6arm')

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dof6arm_path, 'launch', 'view.launch1.py')
        )
    )

    fk_node = Node(
        package='arm_kinematics',
        executable='fk_node',
        output='screen'
    )

    ik_node = Node(
        package='arm_kinematics',
        executable='ik_node',
        output='screen'
    )

    Pose_gui = Node(
        package='arm_kinematics',
        executable='pose_gui',
        output='screen'
    )

    home_node = Node(
        package='arm_kinematics',
        executable='home_pose_node',
        output='screen'
    )

    # ✅ Proper rosbridge include
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('rosbridge_server').find('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    # ✅ Open interface with 1 sec delay
    open_interface = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['xdg-open', os.path.expanduser('~/arm_ws/src/interface.html')],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        rviz_launch,
        fk_node,
        ik_node,
        home_node,
        # Pose_gui,(only for testing, can be removed later)
        rosbridge,
        open_interface
    ])