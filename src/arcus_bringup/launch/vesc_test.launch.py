import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('arcus_description')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'test_robot.urdf')
    controller_yaml = os.path.join(pkg_dir, 'config', 'test_controllers.yaml')

    # Load URDF content
    with open(urdf_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    return LaunchDescription([      
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
            ]
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