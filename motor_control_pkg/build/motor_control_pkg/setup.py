from setuptools import find_packages, setup

package_name = 'motor_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Archivos de recursos requeridos por ROS 2
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Archivo package.xml
        ('share/' + package_name, ['package.xml']),
        # Archivos de launch
        ('share/' + package_name + '/launch', ['launch/pascalito_bringup.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pascalito',
    maintainer_email='pascalito@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control_pkg.motor_control_node:main',
            'motor_odom_node = motor_control_pkg.motor_odom_node:main',
            'master_control = motor_control_pkg.master_control:main',
        ],
    },
)
