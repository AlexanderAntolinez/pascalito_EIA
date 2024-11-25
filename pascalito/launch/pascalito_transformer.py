from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():



    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('pascalito'),  # Cambia 'pascalito' por el nombre de tu paquete
            'config',
            'slam_toolbox_params.yaml'  # Tu archivo de configuración YAML
        ]),
        description='Ruta al archivo de configuración de parámetros para SLAM'
    )


    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="pascalito",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="pascalito.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher GUI automatically.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    gui = LaunchConfiguration("gui")

    # Nodo 1: Transformación estática entre base_footprint y base_link
    static_transform_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    # Nodo 2: Transformación estática entre base_link y laser
    static_transform_base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_laser',
        arguments=['0', '0', '0', '3.141516', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # Nodo 3: Nodo en C++ (odom)
    odom_node = Node(
        package='pascalito',
        executable='odom',
        name='odom',
        output='screen'
    )

    # Nodo 4: Nodo Python (cinematicaInversa_node.py)
    cinematica_inversa_node = Node(
        package='pascalito',
        executable='cinematicaInversa_node.py',
        name='cinematica_inversa',
        output='screen'
    )

    # Nodo 5: Nodo de SLAM Toolbox
    #slam_toolbox_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(
    #            FindPackageShare('slam_toolbox').find('slam_toolbox'),
    #            'launch',
    #            'online_async_launch.py',
    #        )
    #    )
    #)

    # Incluir el archivo de lanzamiento predeterminado de Nav2
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    # Archivo de configuración de RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "config_robot.rviz"]
    )

    # Nodo 6: RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Retornar la descripción del lanzamiento
    return LaunchDescription([
        *declared_arguments,  # <- Expande los argumentos declarados
        declare_params_file_cmd,
        static_transform_base_footprint_to_base_link,
        static_transform_base_link_to_laser,
        odom_node,
        cinematica_inversa_node,
        slam_toolbox_launch,
        rviz_node
    ])
