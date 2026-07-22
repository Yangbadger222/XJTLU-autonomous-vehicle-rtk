from setuptools import setup
import os
from glob import glob

package_name = 'gnss_global_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'map'), glob('map/*.geojson')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='your_email@example.com',
    description='GNSS Global Path Planner for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'global_path_planner = gnss_global_path_planner.global_path_planner:main'
        ],
    },
    scripts=[
        'scripts/global_path_planner.py',  # 告诉 ROS2 安装这个脚本
    ],
)
