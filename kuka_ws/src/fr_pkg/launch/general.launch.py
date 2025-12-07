from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Ruta al launch del URDF
    kuka_description_share = get_package_share_directory('kuka_description')
    urdf_launch = os.path.join(kuka_description_share, 'urdf.launch.py')

    # Nodos del paquete fr_pkg
    enfoque_1 = Node(
        package='fr_pkg',
        executable='enfoque_1',
        name='enfoque_1'
    )

    enfoque_2 = Node(
        package='fr_pkg',
        executable='enfoque_2',
        name='enfoque_2'
    )

    control_servidor = Node(
        package='fr_pkg',
        executable='control_servidor',
        name='control_servidor'
    )

    translator_node = Node(
        package='fr_pkg',
        executable='translator_node',
        name='translator_node'
    )

    return LaunchDescription([
        enfoque_1,
        enfoque_2,
        control_servidor,
        translator_node,
    ])
