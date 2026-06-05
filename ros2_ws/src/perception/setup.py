from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ttkan',
    maintainer_email='tkaneshiro2006@gmail.com',
    description='Perception nodes for the Autonomous Beach Robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trash_detector = perception.trash_detector:main',
        ],
    },
)
