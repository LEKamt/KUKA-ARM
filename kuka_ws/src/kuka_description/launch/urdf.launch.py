from launch_ros.actions import Node
from launch import LaunchDescription
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('kuka_description')

    # Procesar Xacro
    xacro_file = os.path.join(share_dir, 'urdf', 'kuka.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # robot_state_publisher -> toma tu urdf y publica /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Nodo puente
    joint_bridge_node = Node(
        package='fr_pkg',
        executable='rviz_bridge',
        name='rviz_bridge',
        output='screen'
    )


    # Visualización en RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_bridge_node
    ])
