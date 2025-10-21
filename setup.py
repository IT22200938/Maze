from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_autonav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='TurtleBot3 Autonomous Navigation with SLAM, Nav2, and DQN',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_clicker = turtlebot3_autonav.goal_clicker_node:main',
            'dqn_node = turtlebot3_autonav.dqn_node:main',
            'recorder = turtlebot3_autonav.recorder_node:main',
        ],
    },
)

