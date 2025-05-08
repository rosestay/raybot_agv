from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'diff_drive_sdk'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='差速驱动机器人SDK',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_point_action_node = diff_drive_sdk.scripts.nav_to_point_action_node:main',
            'basic_control = diff_drive_sdk.scripts.basic_control:main',
            'navigation_client = diff_drive_sdk.scripts.navigation_client:main',
        ],
    },
)
