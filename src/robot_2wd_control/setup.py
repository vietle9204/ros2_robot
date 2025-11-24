from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robot_2wd_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='vietle9204',
    maintainer_email='vietle9204@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follow_test = robot_2wd_control.wall_follow_test:main',''
            'to_goals_test = robot_2wd_control.to_goals_test:main',
            'bug_alg_test = robot_2wd_control.bug_alg_test:main',
            'vfh_alg_test = robot_2wd_control.vfh_alg_test:main',
            'bug2_alg_test = robot_2wd_control.bug2_alg_test:main',
        ],
    },
)
