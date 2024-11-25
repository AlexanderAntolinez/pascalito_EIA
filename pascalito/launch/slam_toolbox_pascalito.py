from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("pascalito"), "urdf", description_file]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "config_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    
    # Add SLAM Toolbox launch
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

    # Add static transform publisher if needed
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.05', '0', '0', '0', 'base_footprint','base_link'],
    )

    nodes = [
        declare_params_file_cmd,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        slam_toolbox_launch,
        #footprint_publisher,
    ]

    return LaunchDescription(declared_arguments + nodes)

#toolbox_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params.yaml')
#    slam_toolbox = Node(
#        parameters=[toolbox_params],
#        package='slam_toolbox',
#        executable='async_slam_toolbox_node',
#        name='slam_toolbox',
#        output='screen',
#        condition=launch.conditions.IfCondition(use_slam_toolbox)
#    )