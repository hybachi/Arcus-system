import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('arcus_description')
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    controller_yaml = os.path.join(pkg_dir, 'config', 'hw_controller.yaml')

    robot_desc = ParameterValue(
        Command([
            'xacro ',
            xacro_path,
            ' use_sim:=false',
        ]),
        value_type=str
    )

    return LaunchDescription([      
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_desc}, 
                controller_yaml
            ],
            output='screen'
        ),

        # Load controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ackermann_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

    ])