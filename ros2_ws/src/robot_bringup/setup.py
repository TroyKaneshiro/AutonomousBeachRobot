from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/beach_robot.launch.py']),
        ('share/' + package_name + '/config', ['config/robot_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ttkan',
    maintainer_email='tkaneshiro2006@gmail.com',
    description='Launch and configuration files to bring up all Autonomous Beach Robot nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
