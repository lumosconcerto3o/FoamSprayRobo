from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    hardware_interface = 'PositionJointInterface'
    robot_name = 'iiwa'
    
    urdf_file = os.path.join(
        get_package_share_directory('iiwa_description'),
        'urdf',
        'iiwa7.urdf.xacro'
    )
    
    robot_description = Node(
        package='xacro',
        executable='xacro',
        name='xacro',
        output='screen',
        arguments=[
            urdf_file,
            f'hardware_interface:={hardware_interface}',
            f'robot_name:={robot_name}',
            'load_gazebo_ros_control:=false'
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    return LaunchDescription([
        robot_description
    ])
