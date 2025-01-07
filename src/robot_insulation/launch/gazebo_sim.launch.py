from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('robot_insulation')

    # Gazebo launch with required plugins
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': '--verbose',
            'paused': 'false',
            'gui': 'true',
            'verbose': 'true',
            'debug': 'false',
            'physics': 'ode',
            'extra_plugin': 'libgazebo_ros_factory.so,libgazebo_ros_init.so'
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'kuka_iiwa',
            '-file', os.path.join(pkg_share, 'urdf', 'kuka_iiwa.urdf.xacro'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn wall
    spawn_wall = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wall',
            '-file', os.path.join(pkg_share, 'urdf', 'wall_model.sdf'),
            '-x', '1.5',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ROS2 control
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'iiwa7_upload.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'hardware_interface': 'PositionJointInterface',
            'robot_name': 'iiwa'
        }.items()
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        spawn_wall,
        ros2_control
    ])
