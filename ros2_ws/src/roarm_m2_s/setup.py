from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'roarm_m2_s'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "roarm=roarm_m2_s.roarm_:main",
            "camera2=roarm_m2_s.camera2:main",
            "pick_place=roarm_m2_s.pick_place_script:main",
        ],
    },
)
