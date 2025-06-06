from setuptools import setup
import os
from glob import glob
package_name = 'diff_drive_robot'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        # 添加config目录
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='user@example.com',
    description='差速驱动机器人包，带键盘控制',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control_node = diff_drive_robot.keyboard_control:main',
            'diff_wheel_control = diff_drive_robot.diff_wheel_control:main',
            'wheel_odom = diff_drive_robot.diff_wheel:main',
            'hardware_interface = diff_drive_robot.hardware_interface:main',
        ],
    },
)