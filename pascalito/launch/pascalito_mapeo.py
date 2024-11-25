from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Incluir otro archivo de lanzamiento
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pascalito'),  # Paquete donde está el otro launch
                'launch',
                'slam_toolbox_pascalito.py'  # Nombre del otro archivo launch
            ])
        ])
    )

    # Nodo en C++ (odom)
    odom_node = Node(
        package='pascalito',
        executable='odom',  # Nombre del ejecutable de tu nodo en C++
        name='odom',
        output='screen'
    )

    # Nodo Python (cinematicaInversa.py)
    cinematica_inversa_node = Node(
        package='pascalito',
        executable='cinematicaInversa_node.py',  # Nombre del archivo del nodo Python
        name='cinematica_inversa',
        output='screen',
        #log_level='debug'
    )

    # Retornar la descripción de lanzamiento
    return LaunchDescription([
        included_launch,
        odom_node,
        cinematica_inversa_node,  # Agregar el nodo Python aquí
    ])
