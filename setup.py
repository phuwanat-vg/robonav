from setuptools import setup

import os
from glob import glob
package_name = 'robonav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.lua')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
        (os.path.join('share',package_name,'maps'),glob('maps/*.yaml')),
        (os.path.join('share',package_name,'maps'),glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='phuwanat.aerod@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_core_node = robonav.robot_core:main",
            "nav2_cmd = robonav.navigation2_command:main",
            "imu_publisher = robonav.imu_publisher:main",
        ],
    },
)
