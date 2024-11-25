from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # Nodo 1: control de los motores
    control_node = Node(
        package='motor_control_pkg',
        executable='master_control',
        name='master_control',
        output='screen'
    )

    #agregar el lidar
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('rplidar_ros').find('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        )
    )

    
    # Retornar la descripción del lanzamiento
    return LaunchDescription([
        control_node,
        lidar_node
    ])
