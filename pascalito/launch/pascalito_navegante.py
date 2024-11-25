from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declarar argumentos de lanzamiento
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('pascalito'),  # Cambia 'pascalito' por el nombre de tu paquete
            'config',
            'nav2_pascal_config.yaml'  # Tu archivo de configuración YAML
        ]),
        description='Ruta al archivo de configuración de parámetros para Nav2'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo simulado (necesario para simuladores como Gazebo)'
    )

    # Incluir el archivo de lanzamiento predeterminado de Nav2
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Retornar la descripción del lanzamiento
    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        navigation_launch,
    ])
