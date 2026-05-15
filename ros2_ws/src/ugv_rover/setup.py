from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ugv_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics4',
    maintainer_email='robotics4@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "nav_goal=ugv_rover.nav2:main",
            "sim_real_sync_guard=ugv_rover.sim_real_sync_guard:main",
            "cube=ugv_rover.cube_joint_controller:main",
            # "cmd_vel_bridge=ugv_rover.cmd_vel_bridge:main",
        ],
    },
)
