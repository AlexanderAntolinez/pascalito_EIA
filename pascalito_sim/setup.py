from setuptools import find_packages, setup

package_name = 'pascalito_sim'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
## Launch Files
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/robot_camera_launch.py']))
## Resource files
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
#data_files.append(('share/' + package_name + '/resource', ['resource/robot_camera.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/rviz_config.rviz']))
#data_files.append(('share/' + package_name + '/resource', ['resource/rviz_camera_config.rviz']))
data_files.append(('share/' + package_name + '/resource', ['resource/ekf_params.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/slam_toolbox_params.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/cartographer_params.lua']))
## World files
data_files.append(('share/' + package_name + '/worlds', ['worlds/pascal_mecanum.wbt']))
#data_files.append(('share/' + package_name + '/worlds', ['worlds/mecanum_robot_camera.wbt']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexander',
    maintainer_email='alexander@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "odometry_publisher = pascalito_sim.odometry_publisher:main",
            "ekf_node = pascalito_sim.ekf_node:main"
        ],
    },
)
