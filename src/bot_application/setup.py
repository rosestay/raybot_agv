from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "init_robot_pose=bot_application.init_robot_pose:main",
            "get_robot_pose=bot_application.get_robot_pose:main",
            "nav_to_pose=bot_application.nav_to_pose:main",
            "waypoint_flollower=bot_application.waypoint_flollower:main",
            "nav_to_point_client=bot_application.nav_to_point_client:main",
        ],
    },
)