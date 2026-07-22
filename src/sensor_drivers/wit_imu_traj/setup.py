from setuptools import setup
import os
from glob import glob

package_name = 'wit_imu_traj'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package to compute and save IMU trajectory.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'imu_trajectory_node = wit_imu_traj.imu_trajectory_node:main',
        ],
    },
)