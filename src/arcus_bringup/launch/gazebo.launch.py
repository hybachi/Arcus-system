import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('arcus_description')
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    controller_yaml = os.path.join(pkg_dir, 'config', 'sim_controller.yaml')
    gazebo_world = os.path.join(pkg_dir, 'worlds', 'empty.sdf')
    # bridge_params = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r ', gazebo_world], 'on_exit_shutdown': 'true'}.items(),
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'test_robot', 
            '-topic', '/robot_description', 
            '-z', '0.1'
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_path, ' use_sim:=true']),
                value_type=str
            )
        }]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        gazebo_bridge,
        spawn_entity,
        robot_state_publisher,
        joint_state_broadcaster,
        robot_controller
    ])